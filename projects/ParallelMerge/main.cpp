/**
 * @file main.cpp
 * @brief ParallelMerge 框架验证程序
 */

#include "ParallelManager.h"
#include "StorageManager.h"

#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <fstream>
#include <span>
#include <chrono>
#include <cassert>

namespace parallel_merge {

/**
 * @brief 验证策略：实现具体的文件写入逻辑
 */
struct TestPolicy {
    struct TaskResult {
        uint64_t taskId;
        std::vector<char> data;
        size_t offset{0};

        [[nodiscard]] size_t GetDataSize() const { return data.size(); }
    };

    struct TaskLogGlobal {
        std::string recordPath;
        std::string dataPath;
        size_t offset{0};
    };

    struct TaskLogNode {
        uint64_t totalDataSize{0};
        uint64_t pendingCount{0};  
        uint64_t pendingBytes{0};  // 新增：待打包数据量
    };

    // 静态成员用于跨线程访问存储（仅用于测试）
    static inline RecordFile recordFile;
    static inline DataHeap dataHeap;
    static inline std::string baseDir = "test_output";

    static TaskLogGlobal OnInit() {
        std::filesystem::create_directories(baseDir);
        TaskLogGlobal log;
        log.recordPath = baseDir + "/records.bin";
        log.dataPath = baseDir + "/data.bin";

        const uint64_t kMaxTasks = 1000000;
        const uint64_t kMaxData = kMaxTasks * 256; // 假设平均 256 字节

        // 初始化存储
        if (!recordFile.Create(log.recordPath, 8, kMaxTasks)) {
            throw std::runtime_error("Failed to create record file");
        }
        if (!dataHeap.Create(log.dataPath, kMaxData)) {
            throw std::runtime_error("Failed to create data heap");
        }

        std::cout << "[TestPolicy] Storage initialized at " << baseDir << "\n";
        return log;
    }

    static void OnFinalize() {
        dataHeap.ShrinkToFit();
        recordFile.Flush();
        
        // 重要：必须关闭句柄以允许校验代码读取文件
        recordFile.Close();
        dataHeap.Close();
        
        std::cout << "[TestPolicy] Finalized and storage closed.\n";
    }

    TaskResult Process(uint64_t id, TaskLogNode& localLog) {
        TaskResult result;
        result.taskId = id;
        
        // 生成变长数据：随机长度 100-200 字节
        size_t len = 100 + (id % 101);
        result.data.resize(len);
        // ...
        localLog.pendingBytes += len;
        return result;
    }

    TaskLogNode& Update(TaskLogNode& log) {
        return log;
    }

    // 新增：层级聚合逻辑
    TaskLogNode UpdateLog(TaskLogNode& parent, const TaskLogNode& child) {
        parent.pendingCount += child.pendingCount;
        parent.pendingBytes += child.pendingBytes;
        parent.totalDataSize += child.totalDataSize;
        return parent;
    }

    // 同步到全局状态，由管理器保证原子性。
    TaskLogGlobal SyncToGlobal(TaskLogGlobal* global, const TaskLogNode& result) {
        TaskLogGlobal localLog(*global);
        global->offset += result.pendingBytes;
        return localLog;
    }

    // 简化签名：只依赖 Log 状态
    bool ShouldPack(const TaskLogNode& log) {
        // 自适应策略：8个任务 或 2KB 就打包
        return log.pendingCount >= 8 || log.pendingBytes >= 2048; 
    }

    // New decoupled signature: receives raw results
    void Pack(TaskLogGlobal* localLog, TaskResult* results, uint64_t task_id) {

        size_t currentPos = localLog->offset;
    }
};

} // namespace parallel_merge

/**
 * @brief Simple verification (Basic Count)
 */
bool VerifyResults(uint64_t totalTasks) {
    // Phase 2 will add more profound verification later
    return true; 
}

int main() {
    std::cout << "=== ParallelMerge Phase 2 Test ===\n";
    
    try {
        parallel_merge::TestPolicy policy;
        uint64_t totalTasks = 500;
        size_t threads = 2;
        
        std::cout << "Running " << totalTasks << " tasks with " << threads << " threads...\n";
        
        auto start = std::chrono::high_resolution_clock::now();
        
        // 使用简化的静态调用，无需填写模板参数
        parallel_merge::ParallelManager::Run(policy, totalTasks, threads);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = end - start;

        std::cout << "\nExecution Summary:\n";
        std::cout << " - Total Time: " << diff.count() << " s\n";
        std::cout << " - Throughput: " << (totalTasks / diff.count()) / 1000.0 << " K tasks/s\n";
        
        std::cout << "Result: PASSED (Basic Completion)\n";

    } catch (const std::exception& e) {
        std::cerr << "CRITICAL ERROR: " << e.what() << "\n";
        return 1;
    }
    
    return 0;
}
