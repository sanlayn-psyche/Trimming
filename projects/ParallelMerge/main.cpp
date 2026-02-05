/**
 * @file main.cpp
 * @brief ParallelMerge 框架入口点
 * 
 * 此文件提供一个最小化的演示入口。
 * 实际使用时，应在具体项目中引入框架头文件并实现自定义 Policy。
 */

#include "ParallelManager.h"

#include <iostream>
#include <vector>
#include <sstream>

namespace parallel_merge {

// ============================================================================
// 示例 Policy 实现（仅用于演示框架编译正确性）
// ============================================================================

/**
 * @brief 示例策略：模拟简单任务处理
 * 
 * 此示例满足 ParallelTaskPolicy Concept 的全部要求，
 * 可用于验证框架的编译时类型检查。
 */
struct DemoPolicy {
    // ========== 类型定义 ==========
    
    /// 任务结果：简单的字符串缓存
    struct TaskResult {
        uint64_t taskId{0};
        std::vector<char> data;
        
        [[nodiscard]] size_t GetDataSize() const { return data.size(); }
    };
    
    /// 全局任务状态
    struct TaskLogGlobal {
        uint64_t totalProcessed{0};
    };
    
    /// 节点任务状态
    struct TaskLogNode {
        uint64_t dataSize{0};
        uint64_t taskCount{0};
    };
    
    // ========== 生命周期 ==========
    
    static TaskLogGlobal OnInit() {
        std::cout << "[DemoPolicy] Initialized\n";
        return TaskLogGlobal{};
    }
    
    static void OnFinalize() {
        std::cout << "[DemoPolicy] Finalized\n";
    }
    
    // ========== 核心处理 ==========
    
    TaskResult Process(uint64_t id) {
        TaskResult result;
        result.taskId = id;
        
        // 模拟生成一些数据
        std::ostringstream oss;
        oss << "Task_" << id << "_data";
        std::string str = oss.str();
        result.data.assign(str.begin(), str.end());
        
        return result;
    }
    
    TaskLogNode& Update(TaskLogNode& log) {
        log.taskCount++;
        return log;
    }
    
    // ========== 序列化 ==========
    
    void SerializeRecord(void* slot, const TaskResult& result) {
        // 示例：写入 taskId 到槽位
        if (slot) {
            *static_cast<uint64_t*>(slot) = result.taskId;
        }
    }
    
    void SerializeData(std::ostream& os, const TaskResult& result) {
        os.write(result.data.data(), 
                 static_cast<std::streamsize>(result.data.size()));
    }
    
    // ========== 装箱逻辑 ==========
    
    bool ShouldPack(const TaskLogNode& log) {
        // 每处理 64 个任务触发一次装箱
        return log.taskCount >= 64;
    }
    
    void Pack() {
        // 示例：实际场景中这里会执行文件写入
        std::cout << "[DemoPolicy] Pack triggered\n";
    }
};

// 编译时验证 DemoPolicy 满足 Concept
static_assert(ParallelTaskPolicy<DemoPolicy>, 
              "DemoPolicy must satisfy ParallelTaskPolicy concept");

} // namespace parallel_merge


int main() {
    std::cout << "========================================\n";
    std::cout << "ParallelMerge Framework v1.0.0\n";
    std::cout << "========================================\n";
    std::cout << "\n";
    std::cout << "This is a header-only template library for\n";
    std::cout << "hierarchical parallel task management.\n";
    std::cout << "\n";
    std::cout << "Components:\n";
    std::cout << "  - ParallelTaskPolicy.h : C++20 Concept definition\n";
    std::cout << "  - TrunkNode.h          : Hierarchical node template\n";
    std::cout << "  - TrunkManager.h       : Global trunk map manager\n";
    std::cout << "  - StorageManager.h     : High-performance I/O utilities\n";
    std::cout << "  - ParallelManager.h    : Main framework orchestrator\n";
    std::cout << "\n";
    std::cout << "DemoPolicy concept check: PASSED\n";
    std::cout << "\n";
    std::cout << "Ready for integration with your project.\n";
    std::cout << "========================================\n";
    
    return 0;
}
