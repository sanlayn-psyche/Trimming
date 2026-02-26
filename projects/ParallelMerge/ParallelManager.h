#pragma once

/**
 * @file ParallelManager.h
 * @brief 框架核心调度器
 * 
 * ParallelManager 是整个并行任务管理框架的入口点：
 * - 任务分发
 * - 状态树维护
 * - 生命周期管理
 * - 工作线程协调
 */

#include "ParallelTaskPolicy.h"
#include "TrunkNode.h"
#include "TrunkManager.h"
#include "StorageManager.h"

#include <atomic>
#include <thread>
#include <vector>
#include <functional>
#include <optional>
#include <bit>
#include <deque>

namespace parallel_merge {

/**
 * @brief 并行任务管理框架
 * 
 * @tparam P 满足 ParallelTaskPolicy 的策略类型
 * 
 * 使用示例：
 * @code
 * struct MyPolicy { ... };  // 实现 ParallelTaskPolicy
 * 
 * ParallelManager<MyPolicy> manager;
 * manager.Initialize(totalTasks, threadCount);
 * 
 * MyPolicy policy;
 * manager.Run(policy);
 * @endcode
 */
/**
 * @brief 并行任务执行器（内部实现类）
 * 
 * @tparam P 满足 ParallelTaskPolicy 的策略类型
 */
template<ParallelTaskPolicy P>
class ParallelExecutor {

protected:
    using NodeType = TrunkNode<P>;
    using ManagerType = TrunkManager<P>;
    uint64_t totalTasks_{0};
    size_t threadCount_{0};
    std::atomic<uint64_t> taskIdCounter_{0};
    std::atomic<uint64_t> completedTasks_{0};
    std::mutex globalLogMutex_;
    ManagerType trunkManager_;

public:
    ParallelExecutor() = default;
    ~ParallelExecutor() = default;
    
    ParallelExecutor(const ParallelExecutor&) = delete;
    ParallelExecutor& operator=(const ParallelExecutor&) = delete;

    bool Initialize(uint64_t totalTasks, size_t threadCount = 0);
    void Run(P& policy);
    [[nodiscard]] uint64_t GetCompletedTasks() const noexcept { return completedTasks_.load(std::memory_order_acquire);}

private:
    void WorkerLoop(P& policy, size_t threadId);
    bool UpdateResult(NodeType& local, NodeType* remote);
    bool UpdateLog(P& policy, NodeType& local, NodeType* remote);
    void RecursivePack(std::vector<typename P::TaskResult>& res, NodeType *node);
    void ReportCompletionUpward(NodeType* completedNode, std::vector<NodeType>& stack, P& policy);
};



template<ParallelTaskPolicy P>
inline bool ParallelExecutor<P>::Initialize(uint64_t totalTasks, size_t threadCount) {

    totalTasks_ = totalTasks;
    if (threadCount == 0) {
        return false;
    }
    threadCount_ = threadCount;
    taskIdCounter_.store(0, std::memory_order_relaxed);
    completedTasks_.store(0, std::memory_order_relaxed);
    trunkManager_.Initialize(totalTasks, threadCount);
    return true;
}

template<ParallelTaskPolicy P>
inline void ParallelExecutor<P>::Run(P &policy)
{
    policy.OnInit();
    std::vector<std::thread> workers;
    workers.reserve(threadCount_);
    for (size_t i = 0; i < threadCount_; ++i) {
        workers.emplace_back([this, &policy, i]() {
            WorkerLoop(policy, i);
        });
    }
    for (auto& worker : workers) {
        worker.join();
    }
    trunkManager_.ProcessCheck();
    policy.OnFinalize();
}

template<ParallelTaskPolicy P>
inline void ParallelExecutor<P>::WorkerLoop(P &policy, size_t threadId) {

    std::vector<NodeType*> remoteLog;
    std::vector<NodeType> localLog;
    remoteLog.resize(trunkManager_.GetMaxLevel() + 1);
    localLog.resize(trunkManager_.GetMaxLevel() + 1);

    for (uint32_t level = 0; level <= trunkManager_.GetMaxLevel(); ++level) {
        remoteLog[level] = trunkManager_.GetOrCreate(level, 0);
        localLog[level] = remoteLog[level]->CreateLocalLog();
    }

    while (true)
    {
        uint64_t taskId = taskIdCounter_.fetch_add(1, std::memory_order_relaxed);
        uint64_t trunkID = taskId / kNodeCapacity;
        typename P::TaskResult task;

        bool toRecord = false;
        bool toUpdate = false;

        if (taskId < totalTasks_)
        {
            task = policy.Process(taskId, localLog[0].nodeLog);
            if (trunkID == localLog[0].index) {
                auto loc = taskId % kNodeCapacity;
                localLog[0].mask.fetch_or(1ull << loc);
                localLog[0].results[loc] = task;
            }
            else {
                toUpdate = true;
                toRecord = true;
            }
        }
        else {
            toUpdate = true;
        }

        for (uint32_t level = 0; level <= trunkManager_.GetMaxLevel(); ++level) {

            if (toUpdate || trunkID != localLog[level].index) {
                // 任务结果与 mask 同步到远程
                toUpdate = false;
                auto loc = localLog[level].index % kNodeCapacity;

                bool is_complete = UpdateResult(localLog[level], remoteLog[level]);
                if (is_complete && level < trunkManager_.GetMaxLevel())
                {
                    localLog[level + 1].mask.fetch_or(1ull << loc);
                    printf("Thread#%x Detected Completed Node: (%d, %d)\n", std::this_thread::get_id(), level, localLog[level].index);
                }

                remoteLog[level]->WaitLogLock();
                bool is_packed = UpdateLog(policy, localLog[level], remoteLog[level]);
                if (is_packed && level < trunkManager_.GetMaxLevel()) {
                    localLog[level + 1].packedMask.fetch_or(1ull << loc);
                    remoteLog[level]->RealeaseLogLock();
                    trunkManager_.Extract(level, remoteLog[level]->index);
                }
                else if (is_complete && level < trunkManager_.GetMaxLevel()) {
                    policy.UpdateLog(localLog[level + 1].nodeLog, localLog[level].nodeLog);
                    localLog[level].ResetLog();
                }

                // 换表
                if (trunkID != localLog[level].index) {
                    remoteLog[level] = trunkManager_.GetOrCreate(level, trunkID);
                    if (remoteLog[level])
                    {
                        localLog[level] = remoteLog[level]->CreateLocalLog();
                    }
                }
                trunkID = trunkID / kNodeCapacity;
            }
            else
            {
                break;
            }
        }

        if (toRecord) {
            auto loc = taskId % kNodeCapacity;
            localLog[0].mask.fetch_or(1ull << loc);
            localLog[0].results[loc] = task;
        }

        if (taskId >= totalTasks_) {
            break;
        }
    }
}

template<ParallelTaskPolicy P>
inline bool ParallelExecutor<P>::UpdateLog(P& policy, NodeType& local, NodeType* remote) {

    // 任务记录到远程
    auto res = policy.UpdateLog(remote->nodeLog, local.nodeLog);
    remote->packedMask.fetch_or(local.packedMask, std::memory_order_release);
    if (policy.ShouldSync(res))
    {
        printf("Thread#%x Detected To Sync: (%d, %d)\n", std::this_thread::get_id(), local.level, local.index);
        std::vector<typename P::TaskResult> merged_task;
        RecursivePack(merged_task, remote);

        std::unique_lock lock(globalLogMutex_);
        policy.Sync(std::move(merged_task), remote->nodeLog);
        remote->ResetLog();
        lock.unlock();
    }
    return false;
}

template<ParallelTaskPolicy P>
inline bool ParallelExecutor<P>::UpdateResult(NodeType& local, NodeType* remote)
{
    if (local.level == 0)
    {
        uint64_t mask = 0b1;
        for (uint32_t loc = 0; loc < kNodeCapacity; ++loc)
        {
            if (mask & local.mask)
            {
                auto res = local.results[loc].value();
                remote->SetResult(loc, std::move(res));
            }
            mask = mask << 1;
        }
    }
    return remote->ReportAndCheckComplete(local.mask);
}


template<ParallelTaskPolicy P>
inline void ParallelExecutor<P>::RecursivePack(std::vector<typename P::TaskResult>& res, NodeType *node) {
    if (node->IsLeafLevel()) {
        // L0 节点：直接执行物理装箱
        uint64_t packMask = node->GetToBePackedMask();
        uint64_t mask = 1;
        for (uint32_t loc = 0; loc < kNodeCapacity; ++loc)
        {
            if (mask & packMask) {
                res.push_back(std::move(node->results[loc].value()));
            }
            mask = mask << 1;
        }
        node->packedMask.fetch_or(packMask);

    } else {
        // 中间节点：遍历所有"已完成但未打包"的子节点
        uint64_t pendingMask = node->GetToBePackedMask();
        while (pendingMask) {
            // 取出最低位的 1
            int i = std::countr_zero(pendingMask);
            uint64_t bit = 1ULL << i;

            uint32_t childLevel = node->level - 1;
            uint64_t childIndex = node->index * kNodeCapacity + i;

            // 获取子节点, 归档后应该删除，因为不再被访问了
            auto child = trunkManager_.Extract(childLevel, childIndex);

            if (child) {RecursivePack(res,child.get());}
            node->MarkPacked(bit);
            pendingMask &= (pendingMask - 1);
        }
    }
}


/**
 * @brief 框架入口点 (Static Helper)
 */
struct ParallelManager {
    /**
     * @brief 运行并行任务处理，支持自动类型推导
     * 
     * @tparam P 用户策略类型
     * @param policy 策略实例
     * @param totalTasks 总任务数
     * @param threadCount 线程数
     */
    template<ParallelTaskPolicy P>
    static void Run(P& policy, uint64_t totalTasks, size_t threadCount = 0) {
        ParallelExecutor<P> executor;
        if (executor.Initialize(totalTasks, threadCount)) {
            executor.Run(policy);
        }
    }
    
    // 如果需要更细粒度的控制，用户仍可以使用 ParallelExecutor<P> 直接操作
};

} // namespace parallel_merge
