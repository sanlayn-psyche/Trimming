#pragma once

/**
 * @file TrunkNode.h
 * @brief 层次化树节点的模板类
 * 
 * TrunkNode 是层次化任务聚合的基本单元：
 * - 每个节点最多追踪 64 个子任务/子节点（使用 64-bit mask）
 * - L0 层节点直接持有 TaskResult 缓存
 * - 中间层节点仅维护 mask 和元信息
 */

#include "ParallelTaskPolicy.h"

#include <atomic>
#include <cstdint>
#include <vector>
#include <memory>

namespace parallel_merge {

/// 每个节点的最大子任务/子节点数量
constexpr uint32_t kNodeCapacity = 64;

/**
 * @brief 层次化树节点
 * 
 * @tparam P 满足 ParallelTaskPolicy 的策略类型
 * 
 * 节点地址计算：
 * - 给定任务 ID `i`，它属于第 L 层的第 `idx_L = i / (64^L)` 个节点
 * - 全局 Key = (level << 48) | nodeIndex
 * 
 * 节点状态：
 * - mask: 原子位掩码，记录已完成的子任务/子节点
 * - targetMask: 目标掩码，尾部节点可能不满 64 个
 */
template<ParallelTaskPolicy P>
struct TrunkNode {
    /// 原子位掩码，每 bit 代表一个子任务/子节点的完成状态
    std::atomic<uint64_t> mask{0};
    
    /// 目标掩码，当 mask == targetMask 时节点完成
    /// 完整节点为 ~0ULL，尾部节点根据实际子任务数设置
    uint64_t targetMask{~0ULL};
    
    /// 节点所属层级（0 = 最底层，直接管理任务）
    uint32_t level{0};
    
    /// 节点在本层级的索引
    uint64_t index{0};
    
    /// 节点任务记录（由 Policy 定义）
    typename P::TaskLogNode nodeLog{};
    
    /// L0 层独有：暂存任务结果
    /// 中间层级此容器为空，子节点通过 TrunkManager 查询
    std::vector<typename P::TaskResult> results;
    
    /// 预留结果容量（仅 L0 使用）
    static constexpr size_t kResultsReserveSize = kNodeCapacity;

    // ========== 构造与状态查询 ==========
    
    TrunkNode() = default;
    
    TrunkNode(uint32_t lvl, uint64_t idx, uint64_t target = ~0ULL)
        : targetMask(target), level(lvl), index(idx)
    {
        if (level == 0) {
            results.reserve(kResultsReserveSize);
        }
    }
    
    /// 检查节点是否已完成（mask 达到 targetMask）
    [[nodiscard]] bool IsComplete() const noexcept {
        return mask.load(std::memory_order_acquire) == targetMask;
    }
    
    /// 检查是否为 L0 层节点
    [[nodiscard]] bool IsLeafLevel() const noexcept {
        return level == 0;
    }
    
    // ========== 原子操作 ==========
    
    /**
     * @brief 报告子任务/子节点完成
     * 
     * @param bitIndex 子任务/子节点在本节点内的索引 [0, 63]
     * @return 更新后的 mask 值
     */
    uint64_t ReportCompletion(uint32_t bitIndex) noexcept {
        const uint64_t bit = 1ULL << bitIndex;
        return mask.fetch_or(bit, std::memory_order_acq_rel) | bit;
    }
    
    /**
     * @brief 检查并标记完成状态
     * 
     * @param bitIndex 子任务/子节点在本节点内的索引
     * @return true 如果本次操作导致节点完成（即最后一个子任务）
     */
    bool ReportAndCheckComplete(uint32_t bitIndex) noexcept {
        const uint64_t newMask = ReportCompletion(bitIndex);
        return newMask == targetMask;
    }
    
    // ========== L0 层结果管理 ==========
    
    /**
     * @brief 添加任务结果（仅 L0 层使用）
     * 
     * @note 调用方需确保线程安全（通常每个 L0 节点由单一线程维护）
     */
    void AddResult(typename P::TaskResult&& result) {
        results.emplace_back(std::move(result));
    }
    
    /**
     * @brief 清空并返回所有结果（用于装箱后释放）
     */
    std::vector<typename P::TaskResult> TakeResults() {
        return std::move(results);
    }

    // ========== Key 计算工具 ==========
    
    /**
     * @brief 计算节点的全局唯一 Key
     * 
     * Key 格式: (level << 48) | nodeIndex
     * 支持最多 65535 层级和 2^48 个节点索引
     */
    [[nodiscard]] uint64_t GetKey() const noexcept {
        return MakeKey(level, index);
    }
    
    /**
     * @brief 静态方法：根据层级和索引计算 Key
     */
    static constexpr uint64_t MakeKey(uint32_t lvl, uint64_t idx) noexcept {
        return (static_cast<uint64_t>(lvl) << 48) | (idx & 0x0000FFFFFFFFFFFF);
    }
    
    /**
     * @brief 计算父节点的 Key
     */
    [[nodiscard]] uint64_t GetParentKey() const noexcept {
        return MakeKey(level + 1, index / kNodeCapacity);
    }
    
    /**
     * @brief 计算本节点在父节点中的 bit 索引
     */
    [[nodiscard]] uint32_t GetBitInParent() const noexcept {
        return static_cast<uint32_t>(index % kNodeCapacity);
    }
};

/**
 * @brief 线程本地的节点栈条目
 * 
 * 线程持有一个栈式结构追踪当前维护的各层级节点
 */
template<ParallelTaskPolicy P>
struct LocalNodeEntry {
    uint64_t key;
    TrunkNode<P>* node;
    
    LocalNodeEntry(uint64_t k, TrunkNode<P>* n) : key(k), node(n) {}
};

} // namespace parallel_merge
