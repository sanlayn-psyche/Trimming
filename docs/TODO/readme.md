# ParallelMerge 项目迁移/归档说明

本项目目前已完成核心功能的开发与验证，准备迁移到新电脑继续后续工作。

## 1. 当前进度摘要
- **框架核心**: 完全实现，包括 `ParallelManager`, `TrunkManager`, `TrunkNode`, `StorageManager`。
- **验证状态**: **PASSED**。已通过 256 任务/4 线程的大压力功能测试，数据 100% 完整。
- **文档**: 已完成详细架构设计 (`architecture_design.md`) 和开发记录 (`walkthrough.md`)。

## 2. 存档内容说明 (docs/TODO/)
- `architecture_design.md`: 包含类结构图、执行序列图及核心设计要点（双掩码装箱、无竞争结果槽）。
- `walkthrough.md`: 详细的开发历程、关键决策及验证结果展示。
- `task_progress.md`: 任务检查单的最终状态。
- `readme.md`: 本说明文件。

## 3. 下一步计划 (TODO)
- **集成测试**: 将 `ParallelMerge` 引入到实际的业务逻辑中进行更大规模的基准测试（Benchmark）。
- **性能优化**: 针对 `DataHeap` 的内存预留和异步写入在不同磁盘（NVMe vs SSD）上的表现进行微调。
- **跨平台完善**: 目前针对 Windows `HANDLE` 优化较多，POSIX 兼容层虽已实现但需进一步实机测试。

## 4. 环境迁移备注
- 依赖项：无外部库依赖（Header-only），需编译器支持 C++20。
- 构建方式：使用 `CMakeTool` 配合 `gen_cmake.py` 重新生成新环境下的 `CMakeLists.txt` 即可。

---
存档时间: 2026-02-05
归档版本: v1.0-Verified
