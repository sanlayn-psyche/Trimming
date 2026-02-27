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

#include <array>
#include <mutex>
#include <optional>

namespace parallel_merge {

/// 每个节点的最大子任务/子节点数量
constexpr uint64_t kNodeCapacity = 64;

/**
 * @brief 层次化树节点
 */
template<typename P>
struct TrunkNode {

    std::atomic<uint64_t> mask{0};
    std::atomic<uint64_t> packedMask{0};
    uint64_t targetMask{~0ULL};
    std::atomic<bool> packingToken { false };

    /// 节点所属层级
    uint32_t level{0};
    /// 节点索引
    uint64_t index{0};
    
    /// 节点任务记录
    typename P::TaskLogNode nodeLog{};
    
    /// L0 层结果缓存：
    /// 使用固定大小数组 + std::optional。
    /// 工作线程根据 taskId % 64 直接存入对应槽位，无需锁竞争。
    std::array<std::optional<typename P::TaskResult>, kNodeCapacity> results;

    /// 保护 nodeLog 的自旋锁
    std::atomic_flag logLock = ATOMIC_FLAG_INIT;

    // ========== 构造与状态查询 ==========
    
    TrunkNode() = default;
    
    TrunkNode(uint32_t lvl, uint64_t idx, uint64_t target = ~0ULL)
        : targetMask(target), level(lvl), index(idx)
    {}

    TrunkNode& operator = (TrunkNode&& other)
    {
        level = other.level;
        index = other.index;
        targetMask = other.targetMask;
        results = std::move(other.results);
        mask = other.mask.load(std::memory_order_relaxed);
        packedMask = other.packedMask.load(std::memory_order_relaxed);
        return *this;
    }

    TrunkNode(TrunkNode&& other)
    {
        level = other.level;
        index = other.index;
        targetMask = other.targetMask;
        results = std::move(other.results);
        mask = other.mask.load(std::memory_order_relaxed);
        packedMask = other.packedMask.load(std::memory_order_relaxed);
    }


    TrunkNode CreateLocalLog() const {
        //res.packedMask.store(packedMask.load(std::memory_order_acquire), std::memory_order_relaxed);
        //res.mask.store(mask.load(std::memory_order_acquire), std::memory_order_relaxed);
        return TrunkNode{level, index, targetMask};
    }

    /**
     * @brief 线程安全地更新日志
     */

    void WaitLogLock()
    {
        while (logLock.test_and_set(std::memory_order_acquire)) {
            // spin
#if defined(__cpp_lib_atomic_wait)
            logLock.wait(true, std::memory_order_relaxed);
#endif
        }
    }

    void RealeaseLogLock()
    {
        logLock.clear(std::memory_order_release);
#if defined(__cpp_lib_atomic_wait)
        logLock.notify_one();
#endif
    }

    void ResetLog()
    {
        nodeLog = typename P::TaskLogNode();
    }
    
    /// 检查节点所有任务是否已处理完成
    [[nodiscard]] bool IsComplete() const noexcept {
        return mask.load(std::memory_order_acquire) == targetMask;
    }

    /// 检查节点所有任务是否已打包落盘
    [[nodiscard]] bool IsAllPacked() const noexcept {
        return packedMask.load(std::memory_order_acquire) == targetMask;
    }
    
    [[nodiscard]] bool IsLeafLevel() const noexcept {
        return level == 0;
    }
    
    // ========== 原子操作 ==========
    
    /**
     * @brief 提取待打包的位掩码
     * 
     * @return 当前已完成但未打包的位掩码
     */
    uint64_t GetToBePackedMask() const noexcept {
        const uint64_t m = mask.load(std::memory_order_acquire);
        const uint64_t p = packedMask.load(std::memory_order_acquire);
        return m & (~p);
    }

    /**
     * @brief 标记指定位已打包
     */
    void MarkPacked(uint64_t bitMask) noexcept {
        packedMask.fetch_or(bitMask, std::memory_order_acq_rel);
    }

    uint64_t ReportCompletion(uint64_t bitMask) noexcept {
        return mask.fetch_or(bitMask, std::memory_order_acq_rel) | bitMask;
    }
    
    bool ReportAndCheckComplete(uint64_t& mask) noexcept {
        mask = ReportCompletion(mask);
        return mask == targetMask;
    }
    
    // ========== L0 层结果管理 ==========
    
    /**
     * @brief 设置任务结果（仅 L0 层使用）
     * 
     * @param slotIndex 槽位索引 [0, 63]
     */
    void SetResult(uint32_t slotIndex, typename P::TaskResult&& result) {
        results[slotIndex].emplace(std::move(result));
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

} // namespace parallel_merge
