## 优化计划 

* 时间：2026-1-29；
* 问题：Merge 开销过大。对于数 GB 的模型，Cache 文件数可能为千万级，文件系统的开销成为绝对瓶颈；
* 解决方案（草稿）：
    * 不要为每一个 NURBS 生成 Cache，必须合并。线程不仅要获取 NURBS-ID，还需要获取 Chunk-ID (暂定一个 Chunk 是 128 张 nurbs)，线程解析完毕后，对 chunk-bitmap 执行 atomic or 操作，并将 data 的指针移入相应的容器内（这里可以是无锁写，不过要在 atomic 之前），持有 Chunk-ID 的解析线程每完成一个 nurbs 的解析，都进行一次 atomic load 操作，将 bitmap 里新增；
    * 解析结束后，atomic 获取一个 finish order (也就是完成的时候的计数器);
    * 收尾时，可能持有 chunk id 的线程处理完最后一个 nurbs 时另一个线程还未处理完，这时存在两种情况：
        * 另一个线程正在处理中；
        * 另一个线程已经处理完了，正在将 data 指针移入容器，但是尚未执行 atomic_or.

    * 考虑一个层次化的递归 Trunk 模板工具？每一级 64 个 Trunk。
        * 抽象层：任务数量。需要注入一个 lamda，接受任务 id 为输入；处理结束后需要把给出一个任务记录表，同时需要注入一个处理任务记录表的函数；这个任务记录表本身应该是一个类型，具有 save to、read 的静态方法；
        * 目前的结构是否允许这种记录？order？
        * 完成一个 mask 的任务后，需要向上层汇报，如果不存在上层时？可能好几个线程需要创建上层节点，它们拥有同一个上层节点。
        * 用一个 map 储存每一级的表单对象。访问分为两步，首先是确认是否能找到（这一步是并行的，之后的操作是原子化的，不会有冲突），找不到时获取锁，尝试创建。
        * 每一个表单对象需要处理：
        * 每一级的上层节点需要一个数据结构，描述当前节点已收集的规模。当达到某个规模以后需要进行归档；局部 atomic 归档/全局归档/检测都应该是这个类的成员。这个类注入到层次化递归模板中生成一个任务调度器。

    * 文件结构：
        * offset 表：7 * 7 的 tree offset 格子，分别要包含 tree、curveSet、curveDetail 的 offset, 那么就是 (49 + 3) * 4 byte = 208 byte; 如果 offset 是 64 位的，则要升级成 (49 + 6) * 4 byte = 220 byte. 之前似乎没有使用 64 位 offset，可能是因为最大的模型的 offset 也没有超过 32 位，就放着没有动，但对于一个 4-6 GB 的模型，很有可能会超，那么相应地，shader 跟 cuda 代码也需要改动；
        * 既然有 offset 表的话，其它所有数据都可以是乱序的，只有 offset 本身必须对齐 Bezier 的顺序；
        * 而 offset 表又是定长的，可以提前锁定 size。



### 详细设计：基于策略注入的层次化并行任务管理与存储方案

#### 1. 动态自底向上任务状态追踪 (Dynamic Bottom-Up Task Aggregator)
为了处理极大规模任务且无需预先构建完整的树结构，采用动态按需增长的架构：
*   **自底向上生长 (On-demand Growth)**：
    *   **任务分发**：全局维护一个 `std::atomic<int> g_task_id_counter`，线程通过 `fetch_add` 领取一个任务包（例如 64 个连续 ID）。
    *   **节点寻址**：任何任务 ID `i` 在第 `L` 层的分箱 ID 为 `idx_L = i / (64^L)`。
    *   **Trunk Map**：使用全局注册表（如 `std::unordered_map<uint64_t, TrunkNode*>`，Key 为 `(Level << 56) | Index`）保存存活的节点信息。
*   **原子化节点创建与汇报**：
    *   **延迟原子操作 (Lazy Sync)**：线程在处理完本地 64 个任务前，仅更新 TLS (Thread Local Storage) 的 `local_mask`。当一个 Block 处理完或线程需要跨越 64 进制边界时，同步至 Level 1 节点。
    *   **递归汇报逻辑 (Ascending Report)**：
        1.  当 Level `L` 的节点 `Mask` 被 `fetch_or` 填满后，计算父节点（Level `L+1`）的 ID。
        2.  **原子创建父节点**：在全局 Map 中查找父节点，若不存在，则加锁并再次确认（Double-Checked Locking）或使用原子插入，确保父节点被唯一创建。
        3.  将当前节点的“完成信号”上报给父节点：执行 `parent_node->mask.fetch_or(1ULL << (current_node_id % 64))`。
        4.  递归此过程，直到上报至最高层。
*   **全局终点判定**：
    *   由于总任务数 `N` 已知，可以预计算根节点的 `TargetMask`（例如任务总数为 100，则根节点 Mask 只有前面的几位被置 1）。
    *   当根节点达到 `TargetMask` 时，由该线程宣告整个解析与聚合过程完成。


#### 2. 高性能并行存储架构 (Storage Management)
针对变长数据与定长索引的不同特性：
*   **定长 Record 表 (如 Offset 表)**：
    *   **槽位寻址**：由于每个槽位定长（如 220 字节），且 BezierID 已知，每个任务的物理写入空间为 `Offset = BezierID * 220`。
    *   **内存映射 (Mmap)**：通过 `CreateFileMapping` / `MapViewOfFile` 等 API 将文件映射到内存空间，多线程直接根据偏移指针写入，利用操作系统的虚拟内存管理实现高效的异步页落盘。
*   **变长 Data 堆 (具体二进制数据)**：
    *   **原子占坑 (Space Reservation)**：使用一个全局 `std::atomic<size_t> g_data_pointer`。线程写之前，计算自身 Buffer 长度 `len`，执行 `start_offset = g_data_pointer.fetch_add(len)`。
    *   **无锁并行写入**：拿到 `start_offset` 后，面向 Windows 使用 `WriteFile` 配合 `OVERLAPPED` 结构直接向文件的指定位置写入数据。由于已提前占坑，多线程写入区域完全隔离，无需互斥锁。
    *   **扩容与收缩**：
        *   采用 1.5x-2x 扩张策略：当占坑后的偏移超过当前文件大小时，触发统一的文件 Resize 动作。
        *   **Shrink to fit**：所有并行任务结束后，调用一次 `SetEndOfFile` 将文件截断到 `g_data_pointer` 的最终确切位置，确保文件紧凑。

#### 3. 架构拆分：管理模板与用户策略 (Framework vs. Policy)
为了保证高度解耦，整个系统分为 **通用管理框架 (ParallelManager)** 与 **具体业务策略 (UserPolicy)** 两个核心部分。

##### 3.1 通用管理框架 (ParallelManager - 框架侧)
框架负责“怎么运行”和“数据流转”，对具体业务逻辑完全透明。其核心职责包括：
*   **任务分发器**：管理 `atomic<int>` 计数器，负责 Batch ID 的分配。
*   **状态树维护**：实现上述的“动态自底向上生长”逻辑，维护 Trunk Map 和原子 Mask 更新。
*   **物理存储管理**：
    *   管理 Data 堆文件和 Record 表文件的生命周期。
    *   提供原子占坑接口（`ReserveDataSpace(size)`）。
    *   处理文件扩张（1.5x 策略）和最终收缩（Shrink）。
*   **生命周期回调**：在任务开始前调用 `Policy::OnInit()`，所有完成后调用 `Policy::OnFinalize()`。

##### 3.2 用户策略约束 (ParallelTaskPolicy Concept)
采用 C++20 Concept 对用户定义的策略类进行形式化约束，确保类型安全与编译期检查。

```cpp
template<typename P>
concept ParallelTaskPolicy = requires(P p, int id, typename P::TaskResult res, void* slot, std::ostream& os) {
    // 1. 必须定义嵌套的结果类型
    typename P::TaskResult;

    // 2. 核心处理逻辑：输入 ID，输出结果
    { p.Process(id) } -> std::same_as<typename P::TaskResult>;

    // 3. 结果大小查询
    { p.GetRecordSize() } -> std::same_as<size_t>;             // 定长槽位大小
    { p.GetDataSize(res) } -> std::same_as<size_t>;            // 变长数据大小

    // 4. 序列化行为
    { p.SerializeRecord(slot, res) } -> std::same_as<void>;    // 写入 Record 表
    { p.SerializeData(os, res) } -> std::same_as<void>;        // 写入 Data 堆
};
```

*   **Policy 类实现示例**：
    ```cpp
    struct BezierPolicy {
        struct TaskResult {
            std::vector<char> raw_data;
            uint64_t offsets[52];
        };

        TaskResult Process(int id) { /* 具体的解析逻辑 */ }
        size_t GetRecordSize() const { return 220; }
        size_t GetDataSize(const TaskResult& res) const { return res.raw_data.size(); }
        void SerializeRecord(void* slot, const TaskResult& res) { memcpy(slot, res.offsets, 52 * 8); }
        void SerializeData(std::ostream& os, const TaskResult& res) { os.write(res.raw_data.data(), res.raw_data.size()); }
    };
    ```


##### 3.3 策略与数据分离原则 (Data-Logic Separation)
*   **分离原因**：`TaskResult` 会频繁地在并行线程与存储管理模块间传递，而 `Policy` 通常作为单例或通过模板实例化存在于框架中。
*   **交互流程**：
    1.  线程调用 `Policy::Process(id)` 得到 `TaskResult`。
    2.  框架调用 `TaskResult::GetDataSize()` 获取变长数据长度。
    3.  框架 `atomic_add` 预留 Data 空间，并分配 Record 槽地址。
    4.  框架并发调用 `Policy::SerializeData` 和 `Policy::SerializeRecord` 完成写磁盘。
    5.  线程最后调用一次 `TrunkAggregator` 记录完成状态。


#### 4. 关键技术细节
*   **64-bit Offset 全链支持**：所有寻址逻辑强制使用 `uint64_t`。对应的 CUDA Kernel 和渲染 Shader 需要同步修改常量缓冲区布局，以支持超过 4GB 的模型寻址。
*   **Cache Line 对齐**：`TrunkNode` 结构体确保 64 字节对齐，防止不同 Trunk 之间的 `atomic_or` 引起 CPU 缓存行的频繁失效震荡。
