

# CGAL

![CGAL](https://github.com/CGAL/cgal/raw/master/Installation/doc_html/images/cgal_2013_grey.png)

[github](https://github.com/CGAL/cgal)

> CGAL ，计算几何算法库，是一个大型 C + + 库的几何数据结构和算法，如 Delaunay 三角网，网格生成，布尔运算的多边形，以及各种几何处理算法。 CGAL 是用来在各个领域：计算机图形学，科学可视化，计算机辅助设计与建模，地理信息系统，分子生物学，医学影像学，机器人学和运动规划，和数值方法。
>



[zhihu](https://zhuanlan.zhihu.com/p/579168502)

> CGAL由三部分组成
>
> 1. 内核（Kernel）
>
> 内核包含了固定大小（constant size，推测为内存大小），无法更改的几何对象（geometric primitive objects），以及这些对象之间的运算。几何对象由两部分共同表达，第一部分是由表达类参数化的独立类，独立类负责指定用于计算的基础数据类型，另一部分是作为内核类的成员以获得来自内核更强的灵活性和适配性
>
> 2. 基础几何数据结构和算法
>
> 基础几何数据结构和算法由特征类（traits classes）参数化，特征类定义数据结构或是算法与他们使用的图元（primitives）之间的接口。很多情况下，CGAL的内核类可以被数据结构和算法当作特征类使用
>
> 3. 与几何无关的组件
>
> 例如循环器，随机数源，用于调试的I/O支持，同可视化工具的接口等
>


---

## CGAL的编译与使用

### 安装CGAL [csdn](https://blog.csdn.net/summer_dew/article/details/107811371)

[github](https://github.com/CGAL/cgal/releases)

![image-20230326212456526](https://raw.githubusercontent.com/akingse/my-picbed/main/x1e4/image-20230326212456526.png)

**配置环境变量**

```
CGAL_DIR = C:\Users\Aking\source\repos\TPL\CGAL-5.5.2
CGAL_DIR = C:\Users\wangk\source\repos\TPL\CGAL-5.5.2
```

### 安装boost

[安装依赖](https://blog.csdn.net/qq_39784672/article/details/125839069) 

> Boost.Polygon是一个非常好的求多边形运算的库。
>
> Boost.Geometry是极为强大的地理空间库之一。Boost.Polygon能做的图元运算Boost.Geometry都能做，尤其是带孔的多边形这一方面Boost.Geometry的API比Boost.Polygon人性化多了。
>

[GitHub](https://github.com/boostorg/boost/releases/tag/boost-1.81.0)

双击bootstrap.bat启动，生成b2.exe后继续安装；

vs配置

```
VC++\包含目录 $(SolutionDir)..\TPL\boost-1.81.0
VC++\库目录 $(SolutionDir)..\TPL\boost-1.81.0\stage\lib
链接器\附加库目录 $(SolutionDir)..\TPL\boost-1.81.0\stage\lib
```

添加环境变量

```shell
#使用命令行
setx 
BOOST_LIBRARYDIR = C:\Users\Aking\source\repos\TPL\boost-1.81.0\libs
Boost_INCLUDEDIR = C:\Users\Aking\source\repos\TPL\boost-1.81.0
PATH = C:\Users\Aking\source\repos\TPL\boost-1.81.0\libs

```




### vs项目配置

```shell
1 添加包含目录（通用属性->VC++ 目录->包含目录）
$(SolutionDir)..\TPL\boost-1.81.0
$(SolutionDir)..\TPL\CGAL-5.5.2\include
$(SolutionDir)..\TPL\CGAL-5.5.2\auxiliary\gmp\include

2 添加库目录（通用属性->VC++ 目录->库目录）
$(SolutionDir)..\TPL\boost-1.81.0\libs
$(SolutionDir)..\TPL\CGAL-5.5.2\auxiliary\gmp\lib

3 添加依赖项（通用属性->链接器->输入>附加依赖项）
libgmp-10.lib
libmpfr-4.lib

4 加载dll（添加到环境变量，调试->环境）
PATH=$(SolutionDir)..\TPL\CGAL-5.5.2\auxiliary\gmp\lib;C:\Qt\5.15.2\msvc2019_64\bin;
也可以添加到系统环境变量中（全局），这样就能大大减少我们工程中配置环境的烦恼。
加载dll，绝对路径，可以添加到系统环境变量
环境变量
%HOMEPATH%  =   C:\Users\用户名(wangk)

# head
C:\Users\Aking\source\repos\TPL\boost-1.81.0
C:\Users\Aking\source\repos\TPL\CGAL-5.5.2\include
C:\Users\Aking\source\repos\TPL\CGAL-5.5.2\auxiliary\gmp\include
# lib
C:\Users\Aking\source\repos\TPL\boost-1.81.0\libs
C:\Users\Aking\source\repos\TPL\\CGAL-5.5.2\auxiliary\gmp\lib
# dll
PATH=C:\Users\Aking\source\repos\TPL\CGAL-5.5.2\auxiliary\gmp\lib

CGAL编不过，注释掉version文件的//5.5.2信息
```



---

### 安装Qt5

安装qt开源个人版，下载器下载 [download](https://www.qt.io/download-thank-you)，需要注册个人账户，注意关闭翻墙软件；

> Qt作为一个支持GUI的大型C++库，计算几何也是有相关支持的。QRegion、QPoint、QLine、QRect、QPolygon……还有一个从渲染到到碰撞检测全包的常用2D图元库QGraphicsScene。

![image-20230324215459927](https://raw.githubusercontent.com/akingse/my-picbed/main/image-20230324215459927.png)

环境变量

```
QTDIR = C:\Qt ，帮助cmake找到Qt
PATH = C:\Qt\6.4.3\msvc2019_64\bin
为了避免与另一个文件夹中具有相同名称的另一个dll发生任何冲突，请将此路径添加为列表中的第一个。
```



### Qt项目配置

```shell
添加环境变量
Windows
QTDIR=C:\Qt\5.15.2
vs-Path
C:\Qt\5.15.2\msvc2019_64\bin

缺少宏定义
Impossible to draw, CGAL_USE_BASIC_VIEWER is not defined.
#define  CGAL_USE_BASIC_VIEWER

无法打开包括文件: "QApplication":
cmake找不到Qt5，打开CMakeLists.txt，添加
set(Qt5_DIR "C:/Qt/5.15.2/msvc2019_64/lib/cmake/Qt5")



```



简单配置cgal.cpp和文件

![image-20230426101017089](https://raw.githubusercontent.com/akingse/my-picbed/main/x1e4/image-20230426101017089.png)



---



### 使用vcpkg源码安装第三方库

```shell
指定路径 D:\Alluser\akingse\TPL
克隆 git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
编译vcpkg
.\bootstrap-vcpkg.bat
集成到全局
.\vcpkg integrate install
.\vcpkg.exe integrate install 
安装各种库
.\vcpkg.exe install sophus:x64-windows
.\vcpkg.exe install g2o:x64-windows --x-use-aria2
.\vcpkg.exe install opencv4:x64-windows
（注意exe打开路径）
PS C:\Users\Aking\source\repos\TPL\vcpkg> .\vcpkg.exe install opencv4:x64-windows

```



```
win安装GiNaC
https://www.ginac.de/CLN/
https://www.ginac.de/tutorial/
```





## Delaunay得劳内三角形算法

使用几个第三方库，写一个得劳内三角形算法，造个小轮子；

include

- eigon
- cgal
- boost

```
C:\Users\Aking\source\repos\TPL\eigen-3.4.0
```



---

Delaunay性质

- 最接近于规则化的的三角网
- 三角形外接圆内部没有其他点
- 如果不存在四点共圆则唯一



> 德劳内三角（DelaunayTriangulation）空圆特性和最大化最小角特性
> 维诺图（Voronoi diagram）：对相邻的两个点之间作垂直频平分线，所构成的图叫又称泰森多边形。
> 它和德劳内三角网是对偶（Duality）的关系。 维诺图中的点是德劳内三角网三角形外接圆的圆心。

[blog](https://blog.csdn.net/qq_49838656/article/details/119641392)





eigon构造矩阵和行列式
