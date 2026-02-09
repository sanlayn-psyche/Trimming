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
public:
    using NodeType = TrunkNode<P>;
    using ManagerType = TrunkManager<P>;
    
    ParallelExecutor() = default;
    ~ParallelExecutor() = default;
    
    ParallelExecutor(const ParallelExecutor&) = delete;
    ParallelExecutor& operator=(const ParallelExecutor&) = delete;

    void Initialize(uint64_t totalTasks, size_t threadCount = 0) {
        totalTasks_ = totalTasks;
        if (threadCount == 0) {
            threadCount = std::thread::hardware_concurrency();
            if (threadCount == 0) threadCount = 4;
        }
        threadCount_ = threadCount;
        taskIdCounter_.store(0, std::memory_order_relaxed);
        completedTasks_.store(0, std::memory_order_relaxed);
        finalized_.store(false, std::memory_order_relaxed);
        trunkManager_.Initialize(totalTasks, threadCount);
    }
    
    void Run(P& policy) {
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
        if (!finalized_.exchange(true, std::memory_order_acq_rel)) {
            P::OnFinalize();
        }
    }
    
    [[nodiscard]] uint64_t GetCompletedTasks() const noexcept {
        return completedTasks_.load(std::memory_order_acquire);
    }

private:
    void WorkerLoop(P& policy, size_t threadId) {
        std::vector<LocalNodeEntry<P>> localStack;
        localStack.reserve(trunkManager_.GetMaxLevel() + 1);
        NodeType* currentL0Node = nullptr;
        uint64_t currentL0Index = UINT64_MAX;
        
        while (true) {
            uint64_t taskId = taskIdCounter_.fetch_add(1, std::memory_order_acq_rel);
            if (taskId >= totalTasks_) break;
            
            uint64_t l0NodeIndex = taskId / kNodeCapacity;
            uint32_t bitIndex = static_cast<uint32_t>(taskId % kNodeCapacity);
            
            if (l0NodeIndex != currentL0Index) {
                if (currentL0Node) SyncToParent(currentL0Node, localStack);
                currentL0Node = trunkManager_.GetOrCreate(0, l0NodeIndex);
                currentL0Index = l0NodeIndex;
                UpdateLocalStack(localStack, currentL0Node);
            }
            
            auto result = policy.Process(taskId);
            currentL0Node->SetResult(bitIndex, std::move(result));
            policy.Update(currentL0Node->nodeLog);
            
            bool nodeComplete = currentL0Node->ReportAndCheckComplete(bitIndex);
            completedTasks_.fetch_add(1, std::memory_order_acq_rel);
            
            // 更新 log.pendingCount（Manager 职责）
            uint64_t readyMask = currentL0Node->GetToBePackedMask();
            currentL0Node->nodeLog.pendingCount = std::popcount(readyMask);

            if (policy.ShouldPack(currentL0Node->nodeLog) || nodeComplete) {
                ExecutePacking(policy, currentL0Node);
                if (nodeComplete) {
                    ReportCompletionUpward(currentL0Node, localStack);
                    currentL0Node = nullptr;
                    currentL0Index = UINT64_MAX;
                }
            }
        }
        if (currentL0Node) SyncToParent(currentL0Node, localStack);
    }

    void ExecutePacking(P& policy, NodeType* node) {
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

    void UpdateLocalStack(std::vector<LocalNodeEntry<P>>& stack, NodeType* l0Node) {
        stack.clear();
        stack.emplace_back(l0Node->GetKey(), l0Node);
        NodeType* current = l0Node;
        for (uint32_t level = 1; level <= trunkManager_.GetMaxLevel(); ++level) {
            uint64_t parentIndex = current->index / kNodeCapacity;
            NodeType* parent = trunkManager_.GetOrCreate(level, parentIndex);
            stack.emplace_back(parent->GetKey(), parent);
            current = parent;
        }
    }

    void SyncToParent(NodeType* node, std::vector<LocalNodeEntry<P>>& stack) {
        // Implementation for partial sync if needed
    }

    void ReportCompletionUpward(NodeType* completedNode, std::vector<LocalNodeEntry<P>>& stack) {
        uint32_t level = completedNode->level;
        uint64_t nodeIndex = completedNode->index;
        while (level < trunkManager_.GetMaxLevel()) {
            uint32_t parentLevel = level + 1;
            uint64_t parentIndex = nodeIndex / kNodeCapacity;
            uint32_t bitInParent = static_cast<uint32_t>(nodeIndex % kNodeCapacity);
            NodeType* parent = trunkManager_.GetOrCreate(parentLevel, parentIndex);
            if (!parent->ReportAndCheckComplete(bitInParent)) break;
            level = parentLevel;
            nodeIndex = parentIndex;
            if (level == trunkManager_.GetMaxLevel()) {
                if (!finalized_.exchange(true, std::memory_order_acq_rel)) {
                    P::OnFinalize();
                }
                break;
            }
        }
    }

    uint64_t totalTasks_{0};
    size_t threadCount_{0};
    std::atomic<uint64_t> taskIdCounter_{0};
    std::atomic<uint64_t> completedTasks_{0};
    std::atomic<bool> finalized_{false};
    typename P::TaskLogGlobal globalLog_{};
    ManagerType trunkManager_;
};

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
        executor.Initialize(totalTasks, threadCount);
        executor.Run(policy);
    }
    
    // 如果需要更细粒度的控制，用户仍可以使用 ParallelExecutor<P> 直接操作
};

} // namespace parallel_merge
