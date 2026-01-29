## 优化计划 

* 时间：2026-1-29；
* 问题：Merge 开销过大。对于数 GB 的模型，Cache 文件数可能为千万级，文件系统的开销成为绝对瓶颈；
* 解决方案：
    * 不要为每一个 NURBS 生成 Cache，必须合并。线程不仅要获取 NURBS-ID，还需要获取 Chunk-ID (暂定一个 Chunk 是 128 张 nurbs)，线程解析完毕后，对 chunk-bitmap 执行 atomic or 操作，并将 data 的指针移入相应的容器内（这里可以是无锁写，不过要在 atomic 之前），持有 Chunk-ID 的解析线程每完成一个 nurbs 的解析，都进行一次 atomic load 操作，将 bitmap 里新增；
    * 解析结束后，atomic 获取一个 finish order (也就是完成的时候的计数器);
    * 收尾时，可能持有 chunk id 的线程处理完最后一个 nurbs 时另一个线程还未处理完，这时存在两种情况：
        * 另一个线程正在处理中；
        * 另一个线程已经处理完了，正在将 data 指针移入容器，但是尚未执行 atomic_or.

    * 考虑一个层次化的递归 Trunk 模板工具？每一级 64 个 Trunk。
        * 抽象层：任务数量。需要注入一个 lamda，接受任务 id 为输入；处理结束后需要把给出一个任务记录表，同时需要注入一个处理任务记录表的函数；这个任务记录表本身应该是一个类型，具有 save to、read 的静态方法；
        * 目前的结构是否允许这种记录？order？
    

    * 文件结构：
        * offset 表：7 * 7 的 tree offset 格子，分别要包含 tree、curveSet、curveDetail 的 offset, 那么就是 (49 + 3) * 4 byte = 208 byte; 如果 offset 是 64 位的，则要升级成 (49 + 6) * 4 byte = 220 byte. 之前似乎没有使用 64 位 offset，可能是因为最大的模型的 offset 也没有超过 32 位，就放着没有动，但对于一个 4-6 GB 的模型，很有可能会超，那么相应地，shader 跟 cuda 代码也需要改动；
        * 既然有 offset 表的话，其它所有数据都可以是乱序的，只有 offset 本身必须对齐 Bezier 的顺序；
        * 而 offset 表又是定长的，可以提前锁定 size。
