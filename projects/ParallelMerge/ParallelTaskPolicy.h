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
#include <memory>
#include <ostream>
#include <span>
#include <vector>

namespace parallel_merge {

// Policy no longer needs to know about TrunkNode internal structure

/**
 * @brief ParallelTaskPolicy Concept
 * 
 * 用户定义的策略类必须满足此 Concept 才能与 ParallelManager 框架配合使用。
 */
template<typename P>
concept ParallelTaskPolicy = requires(
    P p,
    uint64_t id,
    typename P::TaskResult result,
    typename P::TaskLogNode nodeLog,
    std::vector<std::unique_ptr<typename P::TaskResult>> result_list,
    void* slot,
    std::ostream& os
) {
    /// 每一个具体任务在内存中的缓存
    typename P::TaskResult;
    /// 节点所属的任务状态记录表
    typename P::TaskLogNode;

    { p.OnInit() } -> std::same_as<void>;
    { p.OnFinalize() } -> std::same_as<void>;
    { p.Process(id, nodeLog) } -> std::same_as<typename P::TaskResult>;
    { p.Sync(result_list, nodeLog) } -> std::same_as<void>;
    { p.ShouldSync(nodeLog) } -> std::convertible_to<bool>;
    { p.UpdateLog(nodeLog, nodeLog) } -> std::same_as<void>;
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
