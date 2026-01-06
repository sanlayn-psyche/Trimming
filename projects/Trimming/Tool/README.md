
# Tool

通用的工具函数包, 使用时只需include 相应的头文件。

## ```log.h```

### 简介

全局的调试信息输出宏.
* LF_LOG: 自适应的输出流
  * 使用了LF_LOG_OPEN 打开文件, 未使用 LF_LOG_CLOSE 关闭: 向打开的文件写入.
  * 未打开文件, 或打开失败: 
    * 使用过LF_LOG_APPEND, 未使用 LF_LOG_CLOSE 关闭: 向Append 的目标写入.
    * 未使用LF_LOG_APPEND 或已使用LF_LOG_CLOSE: 向标准输出写入.
* LF_LOG_APPEND: 向某单独写入
* LF_LOG_OPEN: 打开文件, 重新写入
* LF_LOG_CLOSE: 关闭所有文件, 包括OPEN 和 APPEND.
``` cpp

LF_LOG << 1; // 标准输出
LF_LOG_OPEN(“output.txt”);
LF_LOG << 2; // 向output输出
LF_LOG_APPEND("output2.txt") << 3; // 单独添加输出到output2.txt文件末尾
LF_LOG << 4; // 向output输出
LF_LOG_CLOSE; // 关闭所有文件
LF_LOG << 5; // 标准输出
LF_LOG_APPEND("output2.txt") << 6; // 继续添加输出到output2.txt文件末尾
LF_LOG << 7; // 继续添加输出到output2.txt文件末尾
LF_LOG_CLOSE; // 关闭所有文件

```

## ```output.h```

### 简介

数组、vector数据输出模版. 使用名称空间 ot.
```cpp
vector<Point> res; 
......
ot::print(res, "output.txt", "\n");

Point *parray = (Point *)malloc(10 * sizeof(Point));
ot::print(parray, 10, "output.txt", "\n");

```
后两个参数可选, 没有路径时输出到标准输出流, 第三个参数为分隔符号, 默认为空格.

## ```tool.h```

### 简介

其它未分类工具. 使用名称空间 ts.

> 把能使用 << 输出的数据转换为字符串, 尽管这里使用了容易被误会的函数名.
> ```cpp
> double * darray =  (double *)malloc(10 * sizeof(double));
> string s1{ts::num2str(darray, 10, "output.txt")}; // 如果参数包含了路径, 则数组还会额外输出到该路径. 默认不包含也不会输出.
> vector<int> iarray(10, 2);
> string s2{ts::num2str(iarray.data(), 10)};
> string s3{ts::num2str(iarray)}; // s2 与 s3 是相同的 
> ```

> 统计任意函数的运行时间, 返回值以毫秒为单位
> ```cpp
> void func1(int a, int b);
> double func2();
>
>......
> double time1 = timeFuncInvocation(func1, 1.0, 2.0); // 计算 func1(1.0, 2.0) 的运行时间.
> double time2 = timeFuncInvocation(func2); // 计算 func2 的运行时间.
>
> ```

> 获取三角形内点
> ```cpp
> vector<Point> get_innerOfTri(const Point& p1, const Point& p2, const Point& p3, double stepsize = 0.01);
> ```

## 其它未在此处的辅助函数

### `GeometryShared.h`
>```cpp
>// 去重模版
>template<typename T>
>void __deDulplicate(vector<T> &A);
>```

>```cpp
>//按升序添加
>template<typename T>
>void __addInOrder(vector<T>& list, T&& elem)
>```

>```cpp
>//快速排序模版
>template<typename T>
>void __vqsort(vector<T> &V, int s, int e, std::function<bool(T&, T&)> ifbigger);
> ......
> vector<Point> v;
> __vqsort(v, 0, v.size(), std::function([](Point &p, Point &q){return p[0] > q[0];}));
> // 把v中的点按x方向升序排序
>```

### `NurbsCurve.h`
* Nurbs 演算模版
* Nurbs 控制点重建模版
* Nurbs 任意阶导数模版
