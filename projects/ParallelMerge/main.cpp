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

// 临时
#include "../Trimming/PDTrimming/TrimManager.h"
#include "../Trimming/PDTrimming/Patch.h"

struct ParallelTrimmingPolicy : public TrimManager {

    ParallelTrimmingPolicy(const char* config): TrimManager(config) {}

    struct TaskResult {
        std::unique_ptr<Patch> patch;
    };
    struct TaskLogNode {
        uint64_t dataSize{0};
        uint64_t bezierCnt{0};
    };
    std::queue<std::vector<TaskResult>> task_queue;
    std::mutex task_mtx;
    std::condition_variable cv;
    std::atomic_bool terminate = false;

    std::unique_ptr<std::thread> write_thread {nullptr};

    void write() {

        string namePrefix;
        (*m_modelInfo)["Name"].get_to(namePrefix);

        namePrefix = m_jsonRoot + namePrefix + "_";
        std::vector<std::ofstream> fouts(8);
        fouts[0].open(namePrefix + "roots.bin", std::ios::binary);
        fouts[1].open(namePrefix + "tree.bin", std::ios::binary);
        fouts[2].open(namePrefix + "curveset.bin", std::ios::binary);
        fouts[3].open(namePrefix + "curvedetail.bin", std::ios::binary);

        vector<int> offsets(4, 0);
        if (m_patch_prop.m_eval_ptr->m_type == EvalType::BinEval)
        {
            offsets[3] = sizeof(binomial_comb) / 4;
            fouts[3].write((char*)binomial_comb, sizeof(binomial_comb));
        }

        fouts[4].open(namePrefix + "ctrl.bin", std::ios::binary);
        fouts[5].open(namePrefix + "geom.bin", std::ios::binary);
        fouts[6].open(namePrefix + "trim.bin", std::ios::binary);
        fouts[7].open(namePrefix + "flag.bin", std::ios::binary);

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
                    target.write((char*)task.data.data(), task.data.size() * sizeof(int));
                }

                local_queue.pop();
            }
        }

        for (auto& fin : fouts)
        {
            fin.close();
        }
    }

    void WriteHelper() {

    }

    void OnInit() {
        write_thread = std::make_unique<std::thread>([this](){write();});
    }

    void OnFinalize() {
        terminate.store(true, std::memory_order_release);
        cv.notify_one();
        write_thread->join();

        for (auto& fout : fouts)
        {
            fout.close();
        }

        (*m_modelInfo)["Name"].get_to(namePrefix);

        (*m_modelInfo)["Trimming"]["Roots_Size"] = static_cast<int>(offsets[0] * 4);
        (*m_modelInfo)["Trimming"]["Tree_Size"] = static_cast<int>(offsets[1] * 4);
        (*m_modelInfo)["Trimming"]["CurveSet_Size"] = static_cast<int>(offsets[2] * 4);
        (*m_modelInfo)["Trimming"]["CurveDetail_Size"] = static_cast<int>(offsets[3] * 4);

        (*m_modelInfo)["Trimming"]["BezierCurve_DataSize"] = bezier_curve_data;
        (*m_modelInfo)["Trimming"]["BezierCurve_Cnt"] = bezier_curve_cnt;

        (*m_modelInfo)["Trimming"]["Roots"] = namePrefix + "_roots.bin";
        (*m_modelInfo)["Trimming"]["Tree"] = namePrefix + "_tree.bin";
        (*m_modelInfo)["Trimming"]["CurveSet"] = namePrefix + "_curveset.bin";
        (*m_modelInfo)["Trimming"]["CurveDetail"] = namePrefix + "_curvedetail.bin";
        (*m_modelInfo)["Trimming"]["Domain"] = namePrefix + "_domain.bin";

        (*m_modelInfo)["Geometry"]["Trim"] = namePrefix + "_trim.bin";
        (*m_modelInfo)["Geometry"]["Ctrl"] = namePrefix + "_ctrl.bin";
        (*m_modelInfo)["Geometry"]["Geom"] = namePrefix + "_geom.bin";
        (*m_modelInfo)["Geometry"]["Samp"] = namePrefix + "_samp.bin";
        (*m_modelInfo)["Geometry"]["Flag"] = namePrefix + "_flag.bin";
        (*m_modelInfo)["Geometry"]["BezierNum"] = bezier_cnt_total;
        (*m_modelInfo)["Geometry"]["TrimmedBezierNum"] = trimmed_bezier_cnt;

        (*m_modelInfo)["Geometry"]["BezierSurf_DataSize"] = bezier_surf_data;
        (*m_modelInfo)["Geometry"]["BezierSurf_Cnt"] = bezier_surf_cnt;

        (*m_modelInfo)["MergedPatchNum"] = tmg.m_totalNumber - mis_patch_cnt;

        std::cout << std::endl;
        std::cout << "File merging finished!" << std::endl << std::endl;
        std::cout << "#NURBS patch: " << tmg.m_totalNumber << std::endl;
        std::cout << "#NURBS patch with wrong trimming: " << mis_patch_cnt << std::endl << std::endl;

        (*m_modelInfo)["Trimming"]["MergeDone"] = 1;

        this->act_updataModleInfo();
    }

    TaskResult Process(uint64_t id, TaskLogNode& localLog) const {
        TaskResult result;
        result.taskId = id;

        // 生成变长数据：随机长度 100-200 字节
        size_t len = 100 + (id % 101);
        result.data.resize(len);
        for (int i = 0; i < len; ++i) {
            result.data[i] = i + 1;
        }

        localLog.dataSize += len;
        localLog.subNodeCnt++;
        return result;
    }

    TaskLogNode UpdateLog(TaskLogNode& parent, const TaskLogNode& child) {
        parent.dataSize += child.dataSize;
        parent.bezierCnt += child.bezierCnt;
        return parent;
    }

    void Sync(std::vector<TaskResult> &&result, TaskLogNode& log) {
        {
            std::lock_guard<std::mutex> lock(task_mtx);
            task_queue.push(std::move(result));
        }
        cv.notify_one();
    }

    static bool ShouldSync(const TaskLogNode& log) {
        return log.bezierCnt >= 1024 || log.dataSize >= 1024 * 1024 * 512;
    }

};


namespace parallel_merge {

/**
 * @brief 验证策略：实现具体的文件写入逻辑
 */
struct TestPolicy {

    struct TaskResult {
        uint64_t taskId;
        uint64_t offset{0};
        std::vector<int> data;
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
                    target.write((char*)task.data.data(), task.data.size() * sizeof(int));
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

    TaskResult Process(uint64_t id, TaskLogNode& localLog) const {
        TaskResult result;
        result.taskId = id;
        
        // 生成变长数据：随机长度 100-200 字节
        size_t len = 100 + (id % 101);
        result.data.resize(len);
        for (int i = 0; i < len; ++i) {
            result.data[i] = i + 1;
        }

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
    setvbuf(stdout, NULL, _IONBF, 0);
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
