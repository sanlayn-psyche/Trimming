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
template<ParallelTaskPolicy P>
class ParallelManager {
public:
    using NodeType = TrunkNode<P>;
    using ManagerType = TrunkManager<P>;
    
    ParallelManager() = default;
    ~ParallelManager() = default;
    
    // 禁用拷贝
    ParallelManager(const ParallelManager&) = delete;
    ParallelManager& operator=(const ParallelManager&) = delete;

    // ========== 配置与初始化 ==========
    
    /**
     * @brief 初始化管理器
     * 
     * @param totalTasks  总任务数量
     * @param threadCount 工作线程数量（0 = 使用硬件并发数）
     */
    void Initialize(uint64_t totalTasks, size_t threadCount = 0) {
        totalTasks_ = totalTasks;
        
        if (threadCount == 0) {
            threadCount = std::thread::hardware_concurrency();
            if (threadCount == 0) threadCount = 4;  // 回退默认值
        }
        threadCount_ = threadCount;
        
        // 重置计数器
        taskIdCounter_.store(0, std::memory_order_relaxed);
        completedTasks_.store(0, std::memory_order_relaxed);
        finalized_.store(false, std::memory_order_relaxed);
        
        // 初始化 TrunkManager
        trunkManager_.Initialize(totalTasks, threadCount);
    }
    
    // ========== 运行 ==========
    
    /**
     * @brief 运行并行任务处理
     * 
     * @param policy 用户策略实例
     * 
     * 此方法会：
     * 1. 调用 Policy::OnInit() 初始化
     * 2. 启动工作线程执行任务
     * 3. 等待所有任务完成
     * 4. 调用 Policy::OnFinalize() 收尾
     */
    void Run(P& policy) {
        // 1. 初始化
        globalLog_ = P::OnInit();
        
        // 2. 启动工作线程
        std::vector<std::thread> workers;
        workers.reserve(threadCount_);
        
        for (size_t i = 0; i < threadCount_; ++i) {
            workers.emplace_back([this, &policy, i]() {
                WorkerLoop(policy, i);
            });
        }
        
        // 3. 等待完成
        for (auto& worker : workers) {
            worker.join();
        }
        
        // 4. 收尾（由最后完成的线程执行，但这里确保一定执行）
        if (!finalized_.exchange(true, std::memory_order_acq_rel)) {
            P::OnFinalize();
        }
    }
    
    // ========== 状态查询 ==========
    
    [[nodiscard]] uint64_t GetTotalTasks() const noexcept { return totalTasks_; }
    [[nodiscard]] size_t GetThreadCount() const noexcept { return threadCount_; }
    [[nodiscard]] uint64_t GetCompletedTasks() const noexcept {
        return completedTasks_.load(std::memory_order_acquire);
    }
    [[nodiscard]] bool IsComplete() const noexcept {
        return completedTasks_.load(std::memory_order_acquire) >= totalTasks_;
    }
    
    /// 获取 TrunkManager 引用（高级用法）
    [[nodiscard]] ManagerType& GetTrunkManager() noexcept { return trunkManager_; }
    [[nodiscard]] const ManagerType& GetTrunkManager() const noexcept { return trunkManager_; }

private:
    // ========== 工作线程主循环 ==========
    
    /**
     * @brief 工作线程主循环
     * 
     * @param policy   策略实例
     * @param threadId 线程编号（用于调试）
     */
    void WorkerLoop(P& policy, [[maybe_unused]] size_t threadId) {
        // 线程本地的节点栈
        std::vector<LocalNodeEntry<P>> localStack;
        localStack.reserve(trunkManager_.GetMaxLevel() + 1);
        
        // 当前维护的 L0 节点
        NodeType* currentL0Node = nullptr;
        uint64_t currentL0Index = UINT64_MAX;
        
        while (true) {
            // 1. 获取下一个任务 ID
            uint64_t taskId = taskIdCounter_.fetch_add(1, std::memory_order_acq_rel);
            
            if (taskId >= totalTasks_) {
                // 没有更多任务
                break;
            }
            
            // 2. 计算任务所属的 L0 节点
            uint64_t l0NodeIndex = taskId / kNodeCapacity;
            uint32_t bitIndex = static_cast<uint32_t>(taskId % kNodeCapacity);
            
            // 3. 如果切换到新的 L0 节点，先处理旧节点
            if (l0NodeIndex != currentL0Index) {
                if (currentL0Node) {
                    // 同步旧节点状态到上层
                    SyncToParent(currentL0Node, localStack);
                }
                
                // 获取或创建新的 L0 节点
                currentL0Node = trunkManager_.GetOrCreate(0, l0NodeIndex);
                currentL0Index = l0NodeIndex;
                
                // 更新本地栈
                UpdateLocalStack(localStack, currentL0Node);
            }
            
            // 4. 执行任务处理
            auto result = policy.Process(taskId);
            
            // 5. 将结果暂存到 L0 节点
            currentL0Node->AddResult(std::move(result));
            
            // 6. 更新节点任务记录
            policy.Update(currentL0Node->nodeLog);
            
            // 7. 标记任务完成
            bool nodeComplete = currentL0Node->ReportAndCheckComplete(bitIndex);
            completedTasks_.fetch_add(1, std::memory_order_acq_rel);
            
            // 8. 检查是否需要装箱
            if (policy.ShouldPack(currentL0Node->nodeLog) || nodeComplete) {
                ExecutePacking(policy, currentL0Node);
                
                if (nodeComplete) {
                    // 向上汇报完成状态
                    ReportCompletionUpward(currentL0Node, localStack);
                    
                    // 当前 L0 节点已完成，创建下一个可能需要的节点
                    // （"即时补充"策略）
                    uint64_t nextL0Index = (totalTasks_ + kNodeCapacity - 1) / kNodeCapacity;
                    if (l0NodeIndex + threadCount_ < nextL0Index) {
                        trunkManager_.CreateAndRegisterL0Node(l0NodeIndex + threadCount_);
                    }
                    
                    currentL0Node = nullptr;
                    currentL0Index = UINT64_MAX;
                }
            }
        }
        
        // 处理线程退出前的最后一个节点
        if (currentL0Node) {
            SyncToParent(currentL0Node, localStack);
        }
    }
    
    // ========== 辅助方法 ==========
    
    /**
     * @brief 更新线程本地节点栈
     */
    void UpdateLocalStack(std::vector<LocalNodeEntry<P>>& stack, NodeType* l0Node) {
        stack.clear();
        
        // L0 入栈
        stack.emplace_back(l0Node->GetKey(), l0Node);
        
        // 逐层向上查找/创建父节点
        NodeType* current = l0Node;
        for (uint32_t level = 1; level <= trunkManager_.GetMaxLevel(); ++level) {
            uint64_t parentIndex = current->index / kNodeCapacity;
            NodeType* parent = trunkManager_.GetOrCreate(level, parentIndex);
            stack.emplace_back(parent->GetKey(), parent);
            current = parent;
        }
    }
    
    /**
     * @brief 将 L0 节点状态同步到父节点
     */
    void SyncToParent(NodeType* node, std::vector<LocalNodeEntry<P>>& stack) {
        if (stack.size() < 2) return;
        
        // 当前节点的父节点在栈中的位置
        // stack[0] = L0, stack[1] = L1, ...
        // 暂时不做实际同步，仅在节点完成时汇报
    }
    
    /**
     * @brief 向上汇报节点完成
     */
    void ReportCompletionUpward(NodeType* completedNode, std::vector<LocalNodeEntry<P>>& stack) {
        uint32_t level = completedNode->level;
        uint64_t nodeIndex = completedNode->index;
        
        while (level < trunkManager_.GetMaxLevel()) {
            // 计算父节点
            uint32_t parentLevel = level + 1;
            uint64_t parentIndex = nodeIndex / kNodeCapacity;
            uint32_t bitInParent = static_cast<uint32_t>(nodeIndex % kNodeCapacity);
            
            NodeType* parent = trunkManager_.GetOrCreate(parentLevel, parentIndex);
            bool parentComplete = parent->ReportAndCheckComplete(bitInParent);
            
            if (!parentComplete) {
                // 父节点未完成，停止向上汇报
                break;
            }
            
            // 父节点也完成了，继续向上
            level = parentLevel;
            nodeIndex = parentIndex;
            
            // 检查是否到达根节点
            if (level == trunkManager_.GetMaxLevel()) {
                // 整个任务树完成
                if (!finalized_.exchange(true, std::memory_order_acq_rel)) {
                    P::OnFinalize();
                }
                break;
            }
        }
    }
    
    /**
     * @brief 执行装箱操作
     */
    void ExecutePacking(P& policy, NodeType* node) {
        // 调用策略的装箱方法
        policy.Pack();
        
        // 清空已处理的结果（释放内存）
        // 注意：实际的序列化应该在 Pack() 中完成
        // 这里仅作为示例，具体实现取决于 Policy
    }

private:
    // 任务配置
    uint64_t totalTasks_{0};
    size_t threadCount_{0};
    
    // 全局状态
    std::atomic<uint64_t> taskIdCounter_{0};
    std::atomic<uint64_t> completedTasks_{0};
    std::atomic<bool> finalized_{false};
    
    // 全局任务记录
    typename P::TaskLogGlobal globalLog_{};
    
    // Trunk 管理器
    ManagerType trunkManager_;
};

} // namespace parallel_merge
