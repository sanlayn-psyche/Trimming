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
    *   **任务分发**：全局维护一个 `std::atomic<uint64_t> g_task_id_counter`，线程通过 `fetch_add` 领取一个任务（任务与任务 ID 一一对应）。
    *   **节点寻址**：通过任务 ID `i` 可以计算出它属于第 `L` 层的第 `idx_L = i / (64^L)` 个箱子。
    *   **Trunk Map**：使用全局注册表（如 `std::unordered_map<uint64_t, TrunkNode*>`，Key 为 `(L << 48) | idx_L`）保存当前尚未存盘的节点信息。
*   **原子化节点创建与汇报**：
    *   **分级本地任务状态栈**：线程持有一个 `std::vector<std::tuple<uint64_t, TrunkNode*>>`，表示局部任务状态。这是一个**栈式结构**，栈底为最高层级，栈顶始终代表当前正在处理的最低层级（L0）节点。
    *   **分级延迟原子操作 (Lazy Sync)**：当线程取不到属于当前维护的 `0` 层的任务时， 向 `1` 层的节点同步本线程已处理的 `0` 层任务；这种检查是递归的，当线程取不到属于 `1` 层的任务时，需要向 `2` 层同步；
    *   **递归汇报逻辑 (Ascending Report)**：
        1.  在进行任务同步时，可以同时检测到 Level `L` 的节点 `Mask` 是否被 `fetch_or` 填满（尾部节点不一定会被填满，所以 TrunkNode 还需要有一个 `uint64_t TargetMask`成员标记填满状态）。如果被填满，必然导致取不到 `L` 层任务，需要向 Level `L+1` 的父节点汇报。
        2.  **无锁节点创建策略**：为避免节点创建时的锁竞争，采用"预创建 + 即时补充"策略：
            *   **预创建**：初始化时为每个层级预创建 `N` 个节点（`N` = 线程数），注册到全局 Trunk Map。
            *   **即时补充**：当某线程触发本层级节点"装箱"完成后，由该线程立即创建一个新的同层级空节点并注册。由于每个线程同一时刻最多只会"装满"一个节点，此操作几乎无竞争。
            *   **父节点查找**：在向上汇报时，父节点指针可能已在本地栈中；若不在，则到全局 Map 中使用 `try_emplace` 或读写锁 (`std::shared_mutex`) 进行查找/插入，优先使用无锁或细粒度锁方案。
        3.  将当前节点的“完成信号”上报给父节点：执行 `parent_node->mask.fetch_or(1ULL << (current_node_id % 64))`。
*   **全局终点判定**：
    *   由于总任务数 `N` 已知，可以预计算根节点的 `TargetMask`（例如任务总数为 100，则根节点 Mask 只有前面的几位被置 1）。
    *   总会有一个线程会触发最后一个节点被填满的信号，所以整个任务聚合可以确保完成；
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
        *   **扩容权争抢**：触发扩容的线程需要通过 CAS 标志位或 mutex 争抢"扩容权"，其他线程在扩容期间需自旋等待，防止多个线程重复调用 `SetEndOfFile`。
        *   **Shrink to fit**：所有并行任务结束后，调用一次 `SetEndOfFile` 将文件截断到 `g_data_pointer` 的最终确切位置，确保文件紧凑。

#### 3. 架构拆分：管理模板与用户策略 (Framework vs. Policy)
为了保证调度架构可复用，整个系统分为 **通用管理框架 (ParallelManager)** 与 **具体业务策略 (UserPolicy)** 两个核心部分。

##### 3.1 通用管理框架 (ParallelManager - 框架侧)
框架负责“怎么运行”和“数据流转”，对具体业务逻辑完全透明。其核心职责包括：
*   **任务分发器**：管理所有的计数器，负责任务 ID 的分配。
*   **状态树维护**：实现上述的“动态自底向上生长”逻辑，维护 Trunk Map 和原子 Mask 更新。

*   **生命周期回调**：在任务开始前调用 `Policy::OnInit()`，所有完成后调用 `Policy::OnFinalize()`。

##### 3.2 用户策略约束 (ParallelTaskPolicy Concept)

用户策略负责：初始化，单个线程“怎么运行”，以及“装箱”时要做什么（这里是“数据存盘”），“装箱”的条件，结束。对于本例，其核心职责包括：

*   **初始化**
    *   创建文件，维护 offset 记录表等；

*   **单个线程做什么**：
    *   解析 NURBS 曲面，生成相应的二进制输出文件，暂存到内存；

*   **“装箱”做什么**：
    *   算 offset 表，处理文件扩张（1.5x 策略）
    *   将暂存的数据写入文件；
    *   将当前文件 size 同步给其它线程；

*   **“装箱”条件**：
    *   暂存到内存的文件 size or Bezier 数；

*   **结束**：
    *   文件最终收缩（Shrink）与保存。


采用 C++20 Concept 对用户定义的策略类进行形式化约束，确保类型安全与编译期检查。约束示例：

```cpp
template<typename P>
concept ParallelTaskPolicy = requires(P p, int id, typename P::TaskResult res, void* slot, std::ostream& os) {
    // 1. 必须定义的数据
    typename P::TaskResult; // 每一个具体的任务在内存中的缓存
    typename P::TaskLogGlobal; // 全局任务状态记录表
    typename P::TaskLogNode; // 节点所属的任务状态记录表


    // 2. 生命周期回调
    { P::OnInit() } -> std::same_as<P::TaskLogGlobal>;         // 初始化（创建文件等）
    { P::OnFinalize() } -> std::same_as<void>;                 // 收尾（Shrink 文件等）

    // 3. 核心处理逻辑：输入 ID，输出结果
    { p.Process(id) } -> std::same_as<typename P::TaskResult>;

    // 4. 更新节点任务记录
    { p.Update(P::TaskLogNode&) } -> std::same_as<P::TaskLogNode&>;

    // 5. 序列化行为
    { p.SerializeRecord(slot, res) } -> std::same_as<void>;    // 写入 Record 表
    { p.SerializeData(os, res) } -> std::same_as<void>;        // 写入 Data 堆

    // 6. 装箱逻辑
    { p.ShouldPack(P::TaskLogNode&) } -> std::convertible_to<bool>; // 装箱条件判断
    { p.Pack() } -> std::same_as<void>;                             // 执行装箱动作
};
```

*   **Policy 类实现示例**：
    ```cpp
    struct BezierPolicy {
        struct TaskResult {
            std::vector<char> raw_data;
            uint64_t offsets[52];
        };
        struct TaskLogGlobal {
            uint64_t offset;
        };
        struct TaskLogNode {
            uint64_t data_size;
            uint64_t task_cnt;
        };

        //...
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


#### 4. 运行流程描述

*   **初始化**：
    *   单一线程（主线程）为每个层级预创建 `N` 个 `TrunkNode`（`N` = 工作线程数），注册到全局 Trunk Map。这确保了线程启动后能立即领取节点，无需争抢创建锁。
    *   `L0` 级 `TrunkNode` 是最特殊的，它应该包含一个 `std::vector<TaskResult>` 列表，用来暂存任务结果。中间层级节点不需要此列表，因为其子节点已在全局 Map 中，可通过 Key 计算遍历。
    *   `TrunkNode` 应设计为模板类 `template<ParallelTaskPolicy P> struct TrunkNode`，以便 `L0` 节点能持有 `typename P::TaskResult` 类型的列表。
*   **处理过程**：
    1.  线程通过 `atomic_fetch_add` 获取任务 ID，执行 `Policy::Process(id)` 生成 `TaskResult`。
    2.  将 `TaskResult` 暂存到当前 `L0` 节点的 `results` 列表中。
    3.  向上层同步任务进展（更新 Mask），检查是否触发"装箱"条件（`Policy::ShouldPack()`）。
    4.  若触发装箱，执行 `Policy::Pack()`，遍历当前节点的 `results` 进行序列化写盘。
    5.  装箱完成后，该线程立即创建一个新的同层级空节点，注册到 Trunk Map，供后续任务使用。
*   **结束**：
    *   取不到任务后，线程依次退出。
    *   最终有且仅有一个线程会触发最上层节点的填满条件（`Mask == TargetMask`），由该线程调用 `Policy::OnFinalize()` 执行收尾操作（Shrink 文件、关闭句柄等）。
