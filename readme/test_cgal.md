

# CGAL

![CGAL](https://github.com/CGAL/cgal/raw/master/Installation/doc_html/images/cgal_2013_grey.png)

[github](https://github.com/CGAL/cgal)

CGAL ，计算几何算法库，是一个大型 C + + 库的几何数据结构和算法，如 Delaunay 三角网，网格生成，布尔运算的多边形，以及各种几何处理算法。 CGAL 是用来在各个领域：计算机图形学，科学可视化，计算机辅助设计与建模，地理信息系统，分子生物学，医学影像学，机器人学和运动规划，和数值方法。



[zhihu](https://zhuanlan.zhihu.com/p/579168502)

CGAL由三部分组成

1. 内核（Kernel）

内核包含了固定大小（constant size，推测为内存大小），无法更改的几何对象（geometric primitive objects），以及这些对象之间的运算。几何对象由两部分共同表达，第一部分是由表达类参数化的独立类，独立类负责指定用于计算的基础数据类型，另一部分是作为内核类的成员以获得来自内核更强的灵活性和适配性

2. 基础几何数据结构和算法

基础几何数据结构和算法由特征类（traits classes）参数化，特征类定义数据结构或是算法与他们使用的图元（primitives）之间的接口。很多情况下，CGAL的内核类可以被数据结构和算法当作特征类使用

3. 与几何无关的组件

例如循环器，随机数源，用于调试的I/O支持，同可视化工具的接口等





---

### CGAL的编译与使用

[csdn](https://blog.csdn.net/summer_dew/article/details/107811371)

安装依赖 

### boost

Boost.Polygon是一个非常好的求多边形运算的库。

Boost.Geometry是极为强大的地理空间库之一。Boost.Polygon能做的图元运算Boost.Geometry都能做，尤其是带孔的多边形这一方面Boost.Geometry的API比Boost.Polygon人性化多了。

[GitHub](https://github.com/boostorg/boost/releases/tag/boost-1.81.0)

双击bootstrap.bat启动，生成b2.exe后继续安装；

配置

VC++\包含目录

VC++\库目录

链接器\附加库目录



---

### Qt

Qt作为一个支持GUI的大型C++库，计算几何也是有相关支持的。QRegion、QPoint、QLine、QRect、QPolygon……还有一个从渲染到到碰撞检测全包的常用2D图元库QGraphicsScene。







