# Trimming 

## Build 方法

1. 在 projects/Trimming/Project.json 设置依赖库路径

需要配置的项目

* include_dirs:
    * stb;
    * eigen;
    * nlohmann;

* dependencies:
    * opencascade (https://dev.opencascade.org/release) 及其依赖项 jemalloc, tcltk, tbb, vtk;

2. 使用 CMakeTool 生成 CMakeLists.txt 文件;

在当前文件夹执行（包含 Solution.json 的文件夹）：
```ps
python CMakeTool/gen_cmake.py
```
> 注意，每次修改 Solution.json 或 Project.json 后都需要重新执行。

3. 使用 CMake 工具生成项目文件。推荐使用 CLion;


4. 调用入口在 projects/Trimming/main.cpp 或 projects/Test/main.cpp;