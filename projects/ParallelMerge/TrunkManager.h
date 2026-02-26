#pragma once

/**
 * @file TrunkManager.h
 * @brief 全局 Trunk Map 管理器
 * 
 * 负责管理所有层级的 TrunkNode：
 * - 节点的创建、查找、注册
 * - 预创建策略以减少运行时锁竞争
 * - 读写锁支持高并发读场景
 */

#include "TrunkNode.h"

#include <unordered_map>
#include <shared_mutex>
#include <mutex>
#include <memory>
#include <optional>
#include <thread>

namespace parallel_merge {

/**
 * @brief 全局 Trunk Map 管理器
 * 
 * @tparam P 满足 ParallelTaskPolicy 的策略类型
 * 
 * 管理策略：
 * - 初始化时为每层级预创建 N 个节点（N = 线程数）
 * - 运行时采用"即时补充"策略：装箱完成的线程创建新节点
 * - 使用 shared_mutex 支持读多写少的访问模式
 */
template<ParallelTaskPolicy P>
class TrunkManager {
public:
    using NodeType = TrunkNode<P>;
    using NodePtr = std::unique_ptr<NodeType>;
    using MapType = std::unordered_map<uint64_t, NodePtr>;

    TrunkManager() = default;
    ~TrunkManager() = default;
    
    // 禁用拷贝
    TrunkManager(const TrunkManager&) = delete;
    TrunkManager& operator=(const TrunkManager&) = delete;
    
    // 允许移动
    TrunkManager(TrunkManager&&) = default;
    TrunkManager& operator=(TrunkManager&&) = default;

    // ========== 初始化 ==========
    
    /**
     * @brief 初始化管理器
     * 
     * @param totalTasks   总任务数量
     * @param threadCount  工作线程数量
     */
    void Initialize(uint64_t totalTasks, size_t threadCount) {
        totalTasks_ = totalTasks;
        threadCount_ = threadCount;
        
        // 计算最大层级数
        maxLevel_ = CalculateMaxLevel(totalTasks);
        
        // 计算根节点的 targetMask
        rootTargetMask_ = CalculateRootTargetMask(totalTasks);
        
        // 预创建各层级节点
        PreCreateNodesForAllLevels(threadCount);
    }
    
    // ========== 节点访问 ==========
    
    /**
     * @brief 获取节点（只读，不创建）
     * 
     * @return 节点指针，若不存在返回 nullptr
     */
    [[nodiscard]] NodeType* Get(uint64_t key) const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        auto it = trunkMap_.find(key);
        return it != trunkMap_.end() ? it->second.get() : nullptr;
    }
    
    /**
     * @brief 获取节点，通过层级和索引
     */
    [[nodiscard]] NodeType* Get(uint32_t level, uint64_t nodeIndex) const {
        return Get(NodeType::MakeKey(level, nodeIndex));
    }

    std::unique_ptr<NodeType> Extract(uint32_t level, uint64_t nodeIndex) {

        printf("Thread#%x Delete Node: (%d, %d)\n", std::this_thread::get_id(), level, nodeIndex);

        const uint64_t key = NodeType::MakeKey(level, nodeIndex);
        std::unique_lock lock(mutex_);
        auto it = trunkMap_.find(key);
        if (it != trunkMap_.end()) {
            auto node = std::move(it->second);
            trunkMap_.erase(it);
            return node;
        }
        return nullptr;
    }
    
    /**
     * @brief 获取或创建节点
     * 
     * 若节点不存在，则创建并注册
     * 
     * @param level     层级
     * @param nodeIndex 节点索引
     * @return 节点指针（永不为 nullptr）
     */
    NodeType* GetOrCreate(uint32_t level, uint64_t nodeIndex) {
        const uint64_t key = NodeType::MakeKey(level, nodeIndex);
        
        // 快速路径：读锁检查
        {
            std::shared_lock<std::shared_mutex> lock(mutex_);
            auto it = trunkMap_.find(key);
            if (it != trunkMap_.end()) {
                return it->second.get();
            }
        }
        
        // 慢速路径：写锁创建
        std::unique_lock<std::shared_mutex> lock(mutex_);
        printf("Thread#%x Create Node: (%d, %d)\n", std::this_thread::get_id(), level, nodeIndex);
        // Double-check
        auto it = trunkMap_.find(key);
        if (it != trunkMap_.end()) {
            return it->second.get();
        }
        
        // 创建新节点
        uint64_t targetMask = CalculateTargetMask(level, nodeIndex);
        auto node = std::make_unique<NodeType>(level, nodeIndex, targetMask);
        NodeType* ptr = node.get();
        trunkMap_.emplace(key, std::move(node));
        
        return ptr;
    }
    
    /**
     * @brief 注册新节点
     * 
     * @param node 要注册的节点（转移所有权）
     * @return 成功返回 true，若 key 已存在返回 false
     */
    bool Register(NodePtr node) {
        if (!node) return false;
        
        const uint64_t key = node->GetKey();
        
        std::unique_lock<std::shared_mutex> lock(mutex_);
        auto [it, inserted] = trunkMap_.try_emplace(key, std::move(node));
        return inserted;
    }
    
    /**
     * @brief 创建并注册新的 L0 节点（用于"即时补充"策略）
     * 
     * @param nodeIndex 新节点的索引
     * @return 创建的节点指针
     */
    NodeType* CreateAndRegisterL0Node(uint64_t nodeIndex) {
        uint64_t targetMask = CalculateTargetMask(0, nodeIndex);
        auto node = std::make_unique<NodeType>(0, nodeIndex, targetMask);
        NodeType* ptr = node.get();
        
        std::unique_lock<std::shared_mutex> lock(mutex_);
        const uint64_t key = ptr->GetKey();
        trunkMap_.emplace(key, std::move(node));
        
        return ptr;
    }
    
    // ========== 状态查询 ==========
    
    /**
     * @brief 获取当前注册的节点数量
     */
    [[nodiscard]] size_t Size() const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return trunkMap_.size();
    }
    
    /**
     * @brief 获取最大层级数
     */
    [[nodiscard]] uint32_t GetMaxLevel() const noexcept {
        return maxLevel_;
    }
    
    /**
     * @brief 获取总任务数
     */
    [[nodiscard]] uint64_t GetTotalTasks() const noexcept {
        return totalTasks_;
    }
    
    /**
     * @brief 检查根节点是否完成
     */
    [[nodiscard]] bool IsRootComplete() const {
        NodeType* root = Get(maxLevel_, 0);
        return root && root->IsComplete();
    }

private:
    // ========== 辅助方法 ==========
    
    /**
     * @brief 计算给定任务数所需的最大层级
     * 
     * 层级 L 可以管理 64^(L+1) 个任务
     */
    static uint32_t CalculateMaxLevel(uint64_t totalTasks) {
        if (totalTasks <= kNodeCapacity) {
            return 0;
        }
        
        uint32_t level = 0;
        uint64_t capacity = kNodeCapacity;
        
        while (capacity < totalTasks) {
            capacity *= kNodeCapacity;
            ++level;
        }
        
        return level;
    }
    
    /**
     * @brief 计算根节点的 targetMask
     */
    uint64_t CalculateRootTargetMask(uint64_t totalTasks) const {
        if (maxLevel_ == 0) {
            // 只有一层，直接按任务数设置
            if (totalTasks >= kNodeCapacity) {
                return ~0ULL;
            }
            return (1ULL << totalTasks) - 1;
        }
        
        // 多层结构，计算最高层需要的子节点数
        uint64_t nodesAtPrevLevel = (totalTasks + kNodeCapacity - 1) / kNodeCapacity;
        for (uint32_t l = 1; l < maxLevel_; ++l) {
            nodesAtPrevLevel = (nodesAtPrevLevel + kNodeCapacity - 1) / kNodeCapacity;
        }
        
        if (nodesAtPrevLevel >= kNodeCapacity) {
            return ~0ULL;
        }
        return (1ULL << nodesAtPrevLevel) - 1;
    }
    
    /**
     * @brief 计算指定节点的 targetMask
     * 
     * 尾部节点可能不满 64 个子任务/子节点
     */
    uint64_t CalculateTargetMask(uint32_t level, uint64_t nodeIndex) const {
        // 计算该层级的总节点数
        uint64_t nodesAtLevel = totalTasks_;
        for (uint32_t l = 0; l <= level; ++l) {
            nodesAtLevel = (nodesAtLevel + kNodeCapacity - 1) / kNodeCapacity;
        }
        
        // 如果不是最后一个节点，使用完整 mask
        uint64_t lastNodeIndex = nodesAtLevel > 0 ? nodesAtLevel - 1 : 0;
        if (nodeIndex < lastNodeIndex) {
            return ~0ULL;
        }
        
        // 计算最后一个节点的子任务/子节点数
        uint64_t itemsAtPrevLevel;
        if (level == 0) {
            itemsAtPrevLevel = totalTasks_;
        } else {
            itemsAtPrevLevel = totalTasks_;
            for (uint32_t l = 0; l < level; ++l) {
                itemsAtPrevLevel = (itemsAtPrevLevel + kNodeCapacity - 1) / kNodeCapacity;
            }
        }
        
        uint64_t itemsInLastNode = itemsAtPrevLevel % kNodeCapacity;
        if (itemsInLastNode == 0 && itemsAtPrevLevel > 0) {
            return ~0ULL;  // 恰好填满
        }
        
        return (1ULL << itemsInLastNode) - 1;
    }
    
    /**
     * @brief 为所有层级预创建节点
     */
    void PreCreateNodesForAllLevels(size_t count) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        
        for (uint32_t level = 0; level <= maxLevel_; ++level) {
            for (size_t i = 0; i < count; ++i) {
                uint64_t targetMask = CalculateTargetMask(level, i);
                printf("Thread#%x Create Node: (%d, %d)\n", std::this_thread::get_id(), level, i);
                auto node = std::make_unique<NodeType>(level, i, targetMask);
                uint64_t key = node->GetKey();
                trunkMap_.emplace(key, std::move(node));
            }
        }
    }
    
private:
    mutable std::shared_mutex mutex_;
    MapType trunkMap_;
    
    uint64_t totalTasks_{0};
    size_t threadCount_{0};
    uint32_t maxLevel_{0};
    uint64_t rootTargetMask_{~0ULL};
};

} // namespace parallel_merge
