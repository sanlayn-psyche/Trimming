#pragma once

/**
 * @file ParallelManager.h
 * @brief 框架核心调度器
 * 
 * ParallelManager 是整个并行任务管理框架的入口点：
 * - 任务分发
 * - 状态树维护
 * - 生命周期管理
 * - 工作线程协调
 */

#include "ParallelTaskPolicy.h"
#include "TrunkNode.h"
#include "TrunkManager.h"
#include "StorageManager.h"

#include <atomic>
#include <thread>
#include <vector>
#include <functional>
#include <optional>
#include <bit>
#include <deque>

namespace parallel_merge {
    /**
     * @brief 并行任务管理框架
     *
     * @tparam P 满足 ParallelTaskPolicy 的策略类型
     *
     * 使用示例：
     * @code
     * struct MyPolicy { ... };  // 实现 ParallelTaskPolicy
     *
     * ParallelManager<MyPolicy> manager;
     * manager.Initialize(totalTasks, threadCount);
     *
     * MyPolicy policy;
     * manager.Run(policy);
     * @endcode
     */
    /**
     * @brief 并行任务执行器（内部实现类）
     *
     * @tparam P 满足 ParallelTaskPolicy 的策略类型
     */
    template<ParallelTaskPolicy P>
    class ParallelExecutor {

    protected:
        using NodeType = TrunkNode<P>;
        using ManagerType = TrunkManager<P>;
        uint64_t totalTasks_{0};
        size_t threadCount_{0};
        std::atomic<uint64_t> taskIdCounter_{0};
        std::atomic<uint64_t> completedTasks_{0};
        std::mutex globalLogMutex_;
        ManagerType trunkManager_;

    public:
        ParallelExecutor() = default;
        ~ParallelExecutor() = default;

        ParallelExecutor(const ParallelExecutor&) = delete;
        ParallelExecutor& operator=(const ParallelExecutor&) = delete;

        bool Initialize(uint64_t totalTasks, size_t threadCount = 0);
        void Run(P& policy);
        [[nodiscard]] uint64_t GetCompletedTasks() const noexcept { return completedTasks_.load(std::memory_order_acquire);}

    private:
        void WorkerLoop(P& policy, size_t threadId);
        bool UpdateResult(NodeType& local, NodeType* remote);
        bool UpdateLog(P& policy, NodeType& local, NodeType* remote);
        void RecursivePack(std::vector<typename P::TaskResult>& res, NodeType *node);
    };



    template<ParallelTaskPolicy P>
    inline bool ParallelExecutor<P>::Initialize(uint64_t totalTasks, size_t threadCount) {

        totalTasks_ = totalTasks;
        if (threadCount == 0) {
            return false;
        }
        threadCount_ = threadCount;
        taskIdCounter_.store(0, std::memory_order_relaxed);
        completedTasks_.store(0, std::memory_order_relaxed);
        trunkManager_.Initialize(totalTasks, threadCount);
        return true;
    }

    template<ParallelTaskPolicy P>
    inline void ParallelExecutor<P>::Run(P &policy)
    {
        policy.OnInit();
        std::vector<std::thread> workers;
        workers.reserve(threadCount_);
        for (size_t i = 0; i < threadCount_; ++i) {
            workers.emplace_back([this, &policy, i]() {
                WorkerLoop(policy, i);
            });
        }
        for (auto& worker : workers) {
            worker.join();
        }

        printf("Sync last\n");
        std::vector<typename P::TaskResult> merged_task;
        auto root = trunkManager_.Extract(trunkManager_.GetMaxLevel(), 0);
        RecursivePack(merged_task, root.get());

        std::unique_lock lock(globalLogMutex_);
        policy.Sync(std::move(merged_task), root->nodeLog);
        lock.unlock();
        root->ResetLog();

        trunkManager_.ProcessCheck();
        policy.OnFinalize();
    }

    template<ParallelTaskPolicy P>
    inline void ParallelExecutor<P>::WorkerLoop(P &policy, size_t threadId) {

        std::vector<NodeType*> remoteLog;
        std::vector<NodeType> localLog;
        remoteLog.resize(trunkManager_.GetMaxLevel() + 1);
        localLog.resize(trunkManager_.GetMaxLevel() + 1);

        for (uint32_t level = 0; level <= trunkManager_.GetMaxLevel(); ++level) {
            remoteLog[level] = trunkManager_.GetOrCreate(level, 0);
            localLog[level] = remoteLog[level]->CreateLocalLog();
        }

        while (true)
        {
            for (uint32_t level = 0; level <= trunkManager_.GetMaxLevel(); ++level) {

                if (policy.ShouldSync(localLog[level].nodeLog)) {
                    // 同步
                    printf("Thread#%x\t Detected To Sync: (%d, %d)\n", std::this_thread::get_id(), level, localLog[level].index);
                    std::vector<typename P::TaskResult> merged_task;
                    RecursivePack(merged_task, &localLog[level]);

                    std::unique_lock lock(globalLogMutex_);
                    policy.Sync(std::move(merged_task), localLog[level].nodeLog);
                    lock.unlock();

                    localLog[level].ResetLog();
                }
            }

            uint64_t taskId = taskIdCounter_.fetch_add(1, std::memory_order_relaxed);
            uint64_t trunkID = taskId;

            for (uint32_t level = 0; level <= trunkManager_.GetMaxLevel(); ++level) {
                trunkID = trunkID / kNodeCapacity;
                if (trunkID != localLog[level].index || taskId >= totalTasks_) {

                    bool has_value = localLog[level].mask != 0;
                    remoteLog[level]->WaitLogLock();

                    remoteLog[level]->Update(localLog[level]);
                    policy.UpdateLog(remoteLog[level]->nodeLog, localLog[level].nodeLog);
                    localLog[level].nodeLog = remoteLog[level]->nodeLog;

                    remoteLog[level]->ReleaseLogLock();

                    if (has_value && localLog[level].IsComplete() && level < trunkManager_.GetMaxLevel()) {
                        // 获取到所有权
                        printf("Thread#%x\t Get Ownership of Node: (%d, %d)\n", std::this_thread::get_id(), level, localLog[level].index);

                        policy.UpdateLog(localLog[level + 1].nodeLog, localLog[level].nodeLog);
                        auto mask_bit = 1ull << (localLog[level].index % kNodeCapacity);
                        localLog[level + 1].mask |= mask_bit;
                        if (localLog[level].IsAllPacked()) {
                            printf("Thread#%x\t Detected all packed, Node: (%d, %d)\n", std::this_thread::get_id(), level, localLog[level].index);
                            localLog[level + 1].packedMask |= mask_bit;
                            trunkManager_.Extract(level, localLog[level].index);
                        }
                    }
                }
                if (trunkID != localLog[level].index) {
                    // 这里可能导致一个错误，下一轮访问 remoteLog 可能遇到空指针，但是逻辑上又确保了不会。
                    remoteLog[level] = trunkManager_.GetOrCreate(level, trunkID);
                    if (remoteLog[level])
                    {
                        localLog[level] = remoteLog[level]->CreateLocalLog();
                    }
                }
            }

            if (taskId < totalTasks_)
            {
                auto task = policy.Process(taskId, localLog[0].nodeLog);
                auto loc = taskId % kNodeCapacity;
                localLog[0].mask |= (1ull << loc);
                localLog[0].results[loc] = task;
            }
            else {
                break;
            }
        }
    }

    template<ParallelTaskPolicy P>
    inline bool ParallelExecutor<P>::UpdateLog(P& policy, NodeType& local, NodeType* remote) {

        // 任务记录到远程
        local.nodeLog = policy.UpdateLog(remote->nodeLog, local.nodeLog);;
        if (policy.ShouldSync(local.nodeLog))
        {
            std::vector<typename P::TaskResult> merged_task;
            RecursivePack(merged_task, remote);

            std::unique_lock lock(globalLogMutex_);
            policy.Sync(std::move(merged_task), remote->nodeLog);
            lock.unlock();
            remote->ResetLog();
        }

        local.packedMask.store(remote->packedMask.load(std::memory_order_relaxed), std::memory_order_relaxed);
        if (local.IsAllPacked()) {
            return true;
        }
        return false;
    }

    template<ParallelTaskPolicy P>
    inline bool ParallelExecutor<P>::UpdateResult(NodeType& local, NodeType* remote)
    {
        if (local.level == 0)
        {
            uint64_t mask = 0b1;
            for (uint32_t loc = 0; loc < kNodeCapacity; ++loc)
            {
                if (mask & local.mask)
                {
                    auto res = local.results[loc].value();
                    remote->SetResult(loc, std::move(res));
                }
                mask = mask << 1;
            }
        }
        uint64_t localmask = local.mask.load(std::memory_order_relaxed);
        bool is_complete = remote->ReportAndCheckComplete(localmask);
        local.mask.store(localmask, std::memory_order_relaxed);
        return is_complete;
    }


    template<ParallelTaskPolicy P>
    void ParallelExecutor<P>::RecursivePack(std::vector<typename P::TaskResult>& res, NodeType *node) {

        uint64_t packMask = node->GetToBePackedMask();
        if (node->IsLeafLevel()) {
            // L0 节点：直接执行物理装箱
            IterateByMask(packMask, [&](uint64_t idx) {
                res.push_back(std::move(node->results[idx].value()));
            });
        } else {
            uint32_t childLevel = node->level - 1;
            IterateByMask(packMask, [&](uint64_t idx) {
                uint64_t childIndex = node->index * kNodeCapacity + idx;
                auto child = trunkManager_.Extract(childLevel, childIndex);
                if (child) {RecursivePack(res,child.get());}
            });
        }
        node->packedMask |= (packMask);
    }



/**
 * @brief 框架入口点 (Static Helper)
 */
struct ParallelManager {
        /**
         * @brief 运行并行任务处理，支持自动类型推导
         *
         * @tparam P 用户策略类型
         * @param policy 策略实例
         * @param totalTasks 总任务数
         * @param threadCount 线程数
         */
        template<ParallelTaskPolicy P>
        static void Run(P& policy, uint64_t totalTasks, size_t threadCount = 0) {
            ParallelExecutor<P> executor;
            if (executor.Initialize(totalTasks, threadCount)) {
                executor.Run(policy);
            }
        }
};
    
    // 如果需要更细粒度的控制，用户仍可以使用 ParallelExecutor<P> 直接操作
}

