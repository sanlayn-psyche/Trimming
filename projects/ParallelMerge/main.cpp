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
    };

    struct TaskLogNode {
        uint64_t totalDataSize{0};
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

        // 初始化存储
        // 假设最大 1024 个任务，每个记录 8 字节（存储 offset）
        if (!recordFile.Create(log.recordPath, 8, 1024)) {
            throw std::runtime_error("Failed to create record file");
        }
        if (!dataHeap.Create(log.dataPath, 1024 * 1024)) {
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

    TaskResult Process(uint64_t id) {
        TaskResult result;
        result.taskId = id;
        
        // 生成变长数据：随机长度 100-200 字节
        size_t len = 100 + (id % 101);
        result.data.resize(len);
        for (size_t i = 0; i < len; ++i) {
            result.data[i] = static_cast<char>((id + i) % 256);
        }
        
        return result;
    }

    TaskLogNode& Update(TaskLogNode& log) {
        return log;
    }

    void SerializeRecord(void* slot, const TaskResult& result) {
        if (slot) {
            *static_cast<uint64_t*>(slot) = result.offset;
        }
    }

    void SerializeData(std::ostream& os, const TaskResult& result) {
    }

    bool ShouldPack(TrunkNode<TestPolicy>& node) {
        uint64_t readyCount = 0;
        uint64_t toBePacked = node.GetToBePackedMask();
        for (int i = 0; i < 64; ++i) {
            if ((toBePacked >> i) & 1) readyCount++;
        }
        return readyCount >= 8 || node.IsComplete();
    }

    void Pack(TrunkNode<TestPolicy>& node) {
        uint64_t toBePacked = node.GetToBePackedMask();
        if (toBePacked == 0) return;

        node.MarkPacked(toBePacked);

        size_t batchSize = 0;
        std::vector<uint32_t> activeIndices;
        for (uint32_t i = 0; i < 64; ++i) {
            if ((toBePacked >> i) & 1) {
                if (node.results[i].has_value()) {
                    batchSize += node.results[i]->GetDataSize();
                    activeIndices.push_back(i);
                }
            }
        }

        if (batchSize == 0) return;

        size_t startOffset = dataHeap.Reserve(batchSize);
        size_t currentPos = startOffset;

        for (uint32_t idx : activeIndices) {
            auto& res = *node.results[idx];
            res.offset = currentPos;
            dataHeap.Write(currentPos, res.data.data(), res.data.size());

            void* slot = recordFile.GetSlotPtr(res.taskId);
            SerializeRecord(slot, res);

            currentPos += res.data.size();
        }

        std::cout << "[TestPolicy] Packed node " << node.index << ", tasks: " << activeIndices.size() 
                  << ", bytes: " << batchSize << "\n";
    }
};

} // namespace parallel_merge

/**
 * @brief 数据正确性校验函数
 */
bool VerifyResults(uint64_t totalTasks) {
    using namespace parallel_merge;
    std::cout << "\nStarting verification...\n";

    // 1. 验证记录表
    std::string recPath = TestPolicy::baseDir + "/records.bin";
    std::ifstream recIn(recPath, std::ios::binary);
    if (!recIn) {
        std::cerr << "Failed to open record file for verification: " << recPath << "\n";
        return false;
    }
    
    std::vector<uint64_t> offsets(totalTasks);
    recIn.read(reinterpret_cast<char*>(offsets.data()), totalTasks * sizeof(uint64_t));
    if (recIn.gcount() != static_cast<std::streamsize>(totalTasks * sizeof(uint64_t))) {
        std::cerr << "Incomplete read from record file. Expected " << totalTasks * 8 << " bytes, got " << recIn.gcount() << "\n";
        return false;
    }
    recIn.close();

    // 2. 验证数据堆
    std::string dataPath = TestPolicy::baseDir + "/data.bin";
    std::ifstream dataIn(dataPath, std::ios::binary);
    if (!dataIn) {
        std::cerr << "Failed to open data file for verification: " << dataPath << "\n";
        return false;
    }
    
    for (uint64_t i = 0; i < totalTasks; ++i) {
        uint64_t offset = offsets[i];
        size_t len = 100 + (i % 101);
        
        std::vector<char> actualData(len);
        dataIn.clear();
        dataIn.seekg(static_cast<std::streamoff>(offset));
        dataIn.read(actualData.data(), len);

        if (dataIn.gcount() != static_cast<std::streamsize>(len)) {
            std::cerr << "Failed to read data for task " << i << " at offset " << offset << ". Expected " << len << " bytes.\n";
            return false;
        }

        for (size_t j = 0; j < len; ++j) {
            char expected = static_cast<char>((i + j) % 256);
            if (actualData[j] != expected) {
                std::cerr << "Data mismatch at task " << i << " byte " << j 
                          << " (Offset " << offset << ", Expected: " << (int)(unsigned char)expected 
                          << ", Actual: " << (int)(unsigned char)actualData[j] << ")\n";
                return false;
            }
        }
    }
    std::cout << "Verification SUCCESS: All " << totalTasks << " tasks verified.\n";
    return true;
}

int main() {
    std::cout << "=== ParallelMerge Functional Test ===\n";
    
    try {
        parallel_merge::TestPolicy policy;
        parallel_merge::ParallelManager<parallel_merge::TestPolicy> manager;

        uint64_t totalTasks = 256; // 4 个完整的 L0 节点
        size_t threads = 4;
        
        manager.Initialize(totalTasks, threads);

        std::cout << "Running " << totalTasks << " tasks with " << threads << " threads...\n";
        manager.Run(policy);

        std::cout << "\nExecution Summary:\n";
        std::cout << " - Tasks Completed: " << manager.GetCompletedTasks() << "/" << totalTasks << "\n";
        
        // 执行最终校验
        if (VerifyResults(totalTasks)) {
            std::cout << "Result: PASSED\n";
        } else {
            std::cout << "Result: FAILED\n";
            return 1;
        }

    } catch (const std::exception& e) {
        std::cerr << "CRITICAL ERROR: " << e.what() << "\n";
        return 1;
    }
    
    return 0;
}
