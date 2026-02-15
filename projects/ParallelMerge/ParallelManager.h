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
    void ExecutePacking(P& policy, NodeType* node);
    bool UpdateLog(NodeType& local, NodeType* remote);
    void RecursivePack(P& policy, NodeType* node);
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

                UpdateLog(localLog[level], remoteLog[level]);
                auto remote_status = remoteLog[level]->UpdateLogSafe(localLog[level].nodeLog, policy);

                if (policy.ShouldPack(remote_status))
                {
                    ExecutePacking(policy, remoteLog[level]);
                }

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
            auto result = policy.Process(taskId);
            auto loc = taskId % kNodeCapacity;
            localLog[0].mask.fetch_or(0x1 << static_cast<uint32_t>(loc));
            localLog[0].results[loc] = result;
        }
        else
        {
            break;
        }
    }
}

template<ParallelTaskPolicy P>
inline bool ParallelExecutor<P>::UpdateLog(NodeType& local, NodeType* remote)
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
inline void ParallelExecutor<P>::ExecutePacking(P &policy, NodeType *node)  {
    // 1. 获取本次需要搬运的位 (Manager 负责状态管理)
    uint64_t toBePacked = node->GetToBePackedMask();
    if (toBePacked == 0) return;

    // 2. 原子标记这些位已进入打包流程
    node->MarkPacked(toBePacked);

    // 3. 收集结果指针 (Manager 负责聚合数据)
    std::vector<typename P::TaskResult*> results;
    results.reserve(64);
    for (uint32_t i = 0; i < 64; ++i) {
        if ((toBePacked >> i) & 1) {
            if (node->results[i].has_value()) {
                results.push_back(&(*node->results[i]));
            }
        }
    }

    if (results.empty()) return;

    // 4. 调用策略执行落盘 (Policy 负责具体处理)
    policy.Pack(node->nodeLog, results);
}


template<ParallelTaskPolicy P>
inline void ParallelExecutor<P>::RecursivePack(P &policy, NodeType *node) {
    if (node->IsLeafLevel()) {
        // L0 节点：直接执行物理装箱
        ExecutePacking(policy, node);
    } else {
        // 中间节点：遍历所有"已完成但未打包"的子节点
        uint64_t pendingMask = node->GetToBePackedMask();
        while (pendingMask) {
            // 取出最低位的 1
            int i = std::countr_zero(pendingMask);
            uint64_t bit = 1ULL << i;

            uint32_t childLevel = node->level - 1;
            uint64_t childIndex = node->index * kNodeCapacity + i;

            // 获取子节点（理论上必然存在，因为 mask 位已置）
            NodeType* child = trunkManager_.GetOrCreate(childLevel, childIndex);

            // 递归处理子节点
            RecursivePack(policy, child);

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
