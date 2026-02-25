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
    typename P::TaskLogGlobal globalLog_{};
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
    void ExecutePacking(P& policy, typename P::TaskLogGlobal &log, NodeType* node);
    bool UpdateResult(NodeType& local, NodeType* remote);
    void RecursivePack(P& policy, typename P::TaskLogGlobal &log, NodeType* node);
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
    globalLog_ = P::OnInit();
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
        uint64_t trunkID = taskId;

        for (uint32_t level = 0; level <= trunkManager_.GetMaxLevel(); ++level) {

            trunkID = trunkID / kNodeCapacity;
            if (trunkID != localLog[level].index) {

                // 任务结果与 mask 同步到远程
                bool is_complete = UpdateResult(localLog[level], remoteLog[level]);
                if (is_complete && level < trunkManager_.GetMaxLevel())
                {
                    auto loc = localLog[level].index % kNodeCapacity;
                    localLog[level + 1].mask.fetch_or(1ull << loc);
                }

                // 任务记录到远程
                remoteLog[level]->WaitLogLock();
                auto res = policy.UpdateLog(remoteLog[level]->nodeLog, localLog[level].nodeLog);
                if (policy.ShouldPack(res))
                {
                    std::unique_lock lock(globalLogMutex_);
                    auto globalLogSlice = policy.SyncToGlobal(&globalLog_, remoteLog[level]->nodeLog);
                    lock.unlock();
                    remoteLog[level]->ResetLog();
                    remoteLog[level]->RealeaseLogLock();
                    RecursivePack(policy, globalLogSlice, remoteLog[level]);
                    if (remoteLog[level]->IsAllPacked()) {
                        // 虽然很危险，但执行逻辑保证了此时不会有其它线程访问该节点，可以直接删除
                        auto child = trunkManager_.Extract(level, remoteLog[level]->index);
                    }
                }
                else {
                    if (is_complete && level < trunkManager_.GetMaxLevel()) {
                        // 尚有未打包的数据但已完成，只能被上层节点递归打包
                        policy.UpdateLog(localLog[level + 1].nodeLog, localLog[level].nodeLog);
                        localLog[level].ResetLog();
                    }
                    remoteLog[level]->RealeaseLogLock();
                }

                // 换表
                remoteLog[level] = trunkManager_.GetOrCreate(level, trunkID);
                if (remoteLog[level])
                {
                    localLog[level] = remoteLog[level]->CreateLocalLog();
                }
            }
            else
            {
                break;
            }
        }

        if (taskId < totalTasks_)
        {
            auto result = policy.Process(taskId, localLog[0].nodeLog);
            auto loc = taskId % kNodeCapacity;
            localLog[0].mask.fetch_or(1ull << loc);
            localLog[0].results[loc] = result;
        }
        else
        {
            break;
        }
    }
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
inline void ParallelExecutor<P>::ExecutePacking(P &policy, typename P::TaskLogGlobal &log ,NodeType *node) {

    uint64_t m = node->mask.load(std::memory_order_relaxed);
    uint64_t p = node->packedMask.load(std::memory_order_relaxed);
    uint64_t packMask = m & (~p);
    uint64_t mask = 1;
    auto taskID = node->index * kNodeCapacity;
    for (uint32_t loc = 0; loc < kNodeCapacity; ++loc, ++taskID)
    {
        if (mask & packMask) {
            policy.Pack(&log, &node->results[loc].value(), taskID);
        }
        mask = mask << 1;
    }
}


template<ParallelTaskPolicy P>
inline void ParallelExecutor<P>::RecursivePack(P& policy, typename P::TaskLogGlobal &log, NodeType* node) {
    if (node->IsLeafLevel()) {
        // L0 节点：直接执行物理装箱
        ExecutePacking(policy,log, node);
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

            // 递归处理子节点
            if (child) {RecursivePack(policy, log,child.get());}

            // 标记本层对应项已打包
            node->MarkPacked(bit);

            // 清除该位，继续下一位 (pendingMask &= ~bit 也可以)
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
