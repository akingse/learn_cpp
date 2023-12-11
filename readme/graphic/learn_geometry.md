[TOC]



## 计算几何 computational geometry

计算几何——邓俊辉

计算几何：算法设计与分析几何版 Agoritium Design & Analysis

打开计算几何的大门，一起领略这位女神的风采；

fractlonal Cascading

 

1. ### Convex Hull 凸包

设定 λ>0, Σλ=1

凸相关/凸无关，线性相关

避免除法和三角函数，运算复杂，引入误差；

重要判断方法： ToLeft Tese, InTriangle Test

为什么3条边只能分出7个部分，大概第8个不存在

Extreme Points 极点算法 O(n^4)

Extreme Edges 极边算法 O(n^3)

Decrease and Conquer 分治 算法 O(n^2)

sorted sequence插入排序思想

In-Convex-Polygon Test 点在多边形内（仅限凸包）

Pattern Of Turns 2*2 区分顶点类型

 

Exterior/Interior 对凸包edge遍历进行toleft测试，顶点保留去除缝合；

Jarvis March 算法  O(最多n^2)

Gift-Wrapping

Selectionsort选择排序

 

Coherence

 

共线问题，起点问题 lowest-then-leftmost=LTL

Output Sensitivity 输出敏感性

reduction归约 推导可知最优算法

Graham Scan 平面扫描  O(nlogn)

 

 

 

 

\02. Geometric Intersection

\03. Triangulation

\04. Voronoi Diagram

\05. Delaunay Triangulation

\06. Point Location

\07. Geometric Range Search

\08. Windowing Query



---

计算几何研究的对象是几何图形。早期人们对于图像的研究一般都是先建立坐标系，把图形转换成函数，然后用插值和逼近的数学方法，特别是用样条函数作为工具来分析图形，取得了可喜的成功。然而，这些方法过多地依赖于坐标系的选取，缺乏几何不变性，特别是用来解决某些大挠度曲线及曲线的奇异点等问题时，有一定的局限性。

 

 

几何算法，包括几何求交、三角剖分、线性规划；

几何结构，包括几何查找、kd树、区域树、梯形图、Voronoi图、排列、Delaunay三角剖分、区间树、优先查找树以及线段树等

几何算法及其数据结构，包括高维凸包、空间二分及BSP树、运动规划、网格生成及四叉树、最短路径查找及可见性图、单纯性区域查找及划分树和切分树等

 

### 一.凸集&凸包

(下文中所有的集合 若不作特殊说明 都是指欧氏空间上的集合)

凸集(Convex Set):任意两点的连线都在这个集合内的集合就是一个凸集.

 

二.Graham扫描算法(Graham Scan Algorithm)

Graham扫描算法维护一个凸壳 通过不断在凸壳中加入新的点和去除影响凸性的点 最后形成凸包

复杂度是排序O(Nlog2N) 扫描O(N) {每个点仅仅出入栈一次}

合起来是一个O(Nlog2N)的算法 很优秀

 

三.快速凸包算法(Quickhull Algorithm)

对比Graham扫描算法和卷包裹算法

我们发现 Graham扫描算法在凸包上的点很密集时仍然适用

卷包裹算法在凸包上点集随机分布时是很高效的

那么有没有两个优点都具备的算法呢?是有的! 快速凸包算法(Quickhull Algorithm)就是这样的一个算法

快速凸包算法是一个和快速排序(Quicksort Algorithm)神似的算法



基础概念

凸包 canvas 凸多边形的（凹的 concave）

卷包裹法(Gift Wrapping)

卷包裹算法从一个必然在凸包上的点开始向着一个方向依次选择最外侧的点，当回到最初的点时 所选出的点集就是所要求的凸包。

Graham扫描算法(Graham Scan Algorithm)

Graham扫描算法维护一个凸壳 通过不断在凸壳中加入新的点和去除影响凸性的点 最后形成凸包；

---

### 拓扑

![image-20230325221720559](https://raw.githubusercontent.com/akingse/my-picbed/main/image-20230325221720559.png)

拓扑关系是指满足拓扑几何学原理的各空间数据间的相互关系。即用结点、弧段和多边形所表示的实体之间的邻接、关联、包含和连通关系。如：点与点的邻接关系、点与面的包含关系、线与面的相离关系、面与面的重合关系等。

1 隐式曲面(Implicit Surface)与显示曲面(Explicit Surface)

所谓隐式曲面指的是并不会告诉你任何点的信息，只会告诉你该曲面上所有点满足的关系

如果使用隐式曲面方程，将会十分容易的判断出一点与曲面的关系，这是一种非常具有吸引力的特性(能够轻易的判定光线与物体是否相交)。

对于显式曲面来说是与隐式曲面相对应的，所有曲面的点被直接给出，或者可以通过映射关系直接得到

区别隐式曲面与显示曲面的关键就在于是否可以直接表示出所有的点；

隐式曲面难以采样曲面上的点，但是可以轻易判断点与曲面的关系，对于显式曲面来说恰恰相反，我们可以很轻易的采样到所有的点，但是给予你任意一点却很难判断它与曲面的关系！

 

2 具体的几种隐式曲面

2.1 代数曲面(Algebraic Surfaces)

2.2 Constructive Solid Geometry(CSG)

CSG指的是可以对各种不同的几何做布尔运算，如并，交，差

![image-20230325221802854](https://raw.githubusercontent.com/akingse/my-picbed/main/image-20230325221802854.png)

2.3 符号距离函数(Signed Distance Function)

2.4 水平集(Level Set)

2.5 分型几何(Fractals)

 

3 具体的几种显式曲面

3.1 点云(Point Cloud)

3.2 多边形网格(Polygon Mesh)

 

---

### Voronoi diagram

Thiessen Polygon

泰森多边形又叫冯洛诺伊图（Voronoi diagram），得名于Georgy Voronoi，是一组由连接两邻点线段的垂直平分线组成的连续多边形。一个泰森多边形内的任一点到构成该多边形的控制点的距离小于到其他多边形控制点的距离。

Delaunay三角形是由与相邻Voronoi多边形共享一条边的相关点连接而成的三角形。Delaunay三角形的外接圆圆心是与三角形相关的Voronoi多边形的一个顶点。

Voronoi图是Delaunay三角剖分的对偶图，生成它的方法有很多，比较有名的有分治算法，扫描线算法，增量法等。但利用Delaunay三角剖分生成Voronoi图的算法是最快的。

但最快的方法则是构造Delaunay三角剖分，再连接相邻三角形的外接圆圆心，即可以到Voronoi图。

 

[算法]( https://blog.csdn.net/K346K346/article/details/52244123)

[第三方库]( https://docs.scipy.org/doc/scipy-0.18.1/reference/generated/scipy.spatial.Voronoi.html)

 （Voronoi图）主要由两部分组成：1）四个数据结构；2）一个基本性质

Voronoi Site； 

Voronoi Edge；

Voronoi Vertex；

Voronoi Cell；

 

Voronoi图在计算几何里的实现一般是用双向循环链表维护Voronoi图的有向边；生成时候利用分治思想，不过内部超级复杂，要求正切线（复杂度O(n)），构造穿梭的折线确定相交的边，删除释放、合并。有一种很巧妙的构造折线的方法（三角形顶点转移法）有利于降低代码编写难度。