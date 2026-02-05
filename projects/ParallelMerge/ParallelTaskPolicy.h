#pragma once

/**
 * @file ParallelTaskPolicy.h
 * @brief C++20 Concept 定义，约束用户策略类型必须满足的接口
 * 
 * 用户策略负责：
 * - 初始化（创建文件，维护 offset 记录表等）
 * - 单个任务的处理逻辑
 * - "装箱"条件判断与执行
 * - 结束收尾（文件收缩与保存）
 */

#include <concepts>
#include <cstdint>
#include <ostream>

namespace parallel_merge {

/**
 * @brief ParallelTaskPolicy Concept
 * 
 * 用户定义的策略类必须满足此 Concept 才能与 ParallelManager 框架配合使用。
 * 
 * 要求的类型成员：
 * - TaskResult:     单个任务在内存中的缓存结果
 * - TaskLogGlobal:  全局任务状态记录表
 * - TaskLogNode:    节点所属的任务状态记录表
 * 
 * 要求的静态方法：
 * - OnInit():       初始化回调，返回 TaskLogGlobal
 * - OnFinalize():   收尾回调
 * 
 * 要求的成员方法：
 * - Process(id):    处理单个任务，返回 TaskResult
 * - Update(log):    更新节点任务记录
 * - SerializeRecord(slot, result): 写入定长 Record 表
 * - SerializeData(os, result):     写入变长 Data 堆
 * - ShouldPack(log): 判断是否应该触发装箱
 * - Pack():         执行装箱动作
 */
template<typename P>
concept ParallelTaskPolicy = requires(
    P p,
    uint64_t id,
    typename P::TaskResult result,
    typename P::TaskLogNode& nodeLog,
    void* slot,
    std::ostream& os
) {
    // ========== 1. 必须定义的类型 ==========
    
    /// 每一个具体任务在内存中的缓存
    typename P::TaskResult;
    
    /// 全局任务状态记录表
    typename P::TaskLogGlobal;
    
    /// 节点所属的任务状态记录表
    typename P::TaskLogNode;

    // ========== 2. 生命周期回调 ==========
    
    /// 初始化（创建文件等），返回全局状态
    { P::OnInit() } -> std::same_as<typename P::TaskLogGlobal>;
    
    /// 收尾（Shrink 文件等）
    { P::OnFinalize() } -> std::same_as<void>;

    // ========== 3. 核心处理逻辑 ==========
    
    /// 输入任务 ID，输出处理结果
    { p.Process(id) } -> std::same_as<typename P::TaskResult>;

    // ========== 4. 节点任务记录更新 ==========
    
    /// 更新节点的任务记录状态
    { p.Update(nodeLog) } -> std::same_as<typename P::TaskLogNode&>;

    // ========== 5. 序列化行为 ==========
    
    /// 将结果写入定长 Record 表的指定槽位
    { p.SerializeRecord(slot, result) } -> std::same_as<void>;
    
    /// 将结果写入变长 Data 堆
    { p.SerializeData(os, result) } -> std::same_as<void>;

    // ========== 6. 装箱逻辑 ==========
    
    /// 判断是否应该触发装箱（基于节点的任务记录）
    { p.ShouldPack(nodeLog) } -> std::convertible_to<bool>;
    
    /// 执行装箱动作
    { p.Pack() } -> std::same_as<void>;
};

/**
 * @brief TaskResult 的基本要求 Concept
 * 
 * TaskResult 需要能够报告其数据大小，供框架进行空间预留
 */
template<typename T>
concept TaskResultType = requires(const T& t) {
    /// 获取变长数据的字节长度
    { t.GetDataSize() } -> std::convertible_to<size_t>;
};

} // namespace parallel_merge
