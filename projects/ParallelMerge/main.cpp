/**
 * @file main.cpp
 * @brief ParallelMerge 框架验证程序
 */

#include "ParallelManager.h"
#include "StorageManager.h"

#include <iostream>
#include <queue>
#include <vector>
#include <string>
#include <filesystem>
#include <fstream>
#include <span>
#include <chrono>
#include <cassert>
#include <condition_variable>

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
    struct TaskLogNode {
        uint64_t dataSize{0};
        uint64_t subNodeCnt{0};
    };

    std::string baseDir = "test_output";
    std::string fileDir;

    std::ofstream target;
    uint64_t fileSize{1024 * 1024};

    std::queue<std::vector<TaskResult>> task_queue;
    std::mutex task_mtx;
    std::condition_variable cv;
    std::atomic_bool terminate = false;

    std::unique_ptr<std::thread> write_thread {nullptr};


    void write() {

        if (!std::filesystem::exists(baseDir)) {
            std::filesystem::create_directories(baseDir);
        }
        fileDir = baseDir + "/records.bin";
        target.open(fileDir, std::ios::out | std::ios::binary);
        std::queue<std::vector<TaskResult>> local_queue;

        while (!terminate.load(std::memory_order_acquire)) {

            if (local_queue.empty()) {
                std::unique_lock<std::mutex> lock(task_mtx);
                cv.wait(lock, [this]{ return !task_queue.empty() || terminate.load(std::memory_order_acquire);});
                if (terminate.load(std::memory_order_acquire) && task_queue.empty()) break;
                local_queue.swap(task_queue);
            }

            while (!local_queue.empty()) {
                auto& list = local_queue.front();

                for (auto& task : list) {
                    target.write(task.data.data(), task.data.size());
                }

                local_queue.pop();
            }
        }

        target.close();
    }

    void OnInit() {

        write_thread = std::make_unique<std::thread>([this](){write();});
        std::cout << "[TestPolicy] Storage initialized at " << baseDir << "\n";
    }

    void OnFinalize() {
        terminate.store(true, std::memory_order_release);
        cv.notify_one();
        write_thread->join();
        std::cout << "[TestPolicy] Finalized and storage closed.\n";
    }

    TaskResult Process(uint64_t id, TaskLogNode& localLog) {
        TaskResult result;
        result.taskId = id;
        
        // 生成变长数据：随机长度 100-200 字节
        size_t len = 100 + (id % 101);
        result.data.resize(len);

        localLog.dataSize += len;
        localLog.subNodeCnt++;
        return result;
    }

    // 新增：层级聚合逻辑
    TaskLogNode UpdateLog(TaskLogNode& parent, const TaskLogNode& child) {
        parent.dataSize += child.dataSize;
        parent.subNodeCnt += child.subNodeCnt;
        return parent;
    }

    // 同步到全局状态，由管理器保证原子性。
    void Sync(std::vector<TaskResult> &&result, TaskLogNode& log) {
        {
            std::lock_guard<std::mutex> lock(task_mtx);
            task_queue.push(std::move(result));
        }
        cv.notify_one();
    }

    static bool ShouldSync(const TaskLogNode& log) {
        return log.subNodeCnt >= 20 || log.dataSize >= 1024 * 1024 * 1024;
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
