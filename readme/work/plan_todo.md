## 碰撞检测



常用的侵入距离算法包括球体切剖法、点对距离法、基于凸壳的方法等。

符号距离场

### GJK算法

GJK算法是由 Gilbert,johnson和 Keerthi 3人在1988年共同开发的一类迭代算法。GJK算法的输入为两物体的顶点集，通过有限次数的迭代后，最后输出结果为两物体之间的欧氏距离。根据两物体之间 的欧氏距离，可进行碰撞检测。当两物体之间的距离等于或者小于零时，可判定两物体发生碰撞。






### 数据结构

kd树

R-tree

[tree](https://zhuanlan.zhihu.com/p/32300891)

---

### PQP-TriDist

两个polyface求最小距离（三角面片求交）

[pqp](https://gamma.cs.unc.edu/SSV/)

邻近查询包 Proximity Query Package

   * collision detection - detect whether two models overlap, and
                           optionally, which triangles of the models  overlap. 是否碰撞，具体到三角面片
                          
   * distance computation - compute the distance between two models, 
                            i. e., the length of the shortest translation  that makes the models overlap 距离计算，最近距离
                           
   * tolerance verification - detect whether two models are closer or
                              farther than a tolerance value. 公差验证，距离与公差值比较



### paper

基于rssobb混合层次包围盒精确碰撞检测研究

计算三角形之间的精确距离以及确切的碰撞位置。与上文计算长方形之间距离相似，两个三角形之间的最近点,或者落在角形边上,或者在三角形内部。三角形边与边之间的距离运算同样用到Gilbert算法;三角形顶点与三角形面之间的距离则需要精确计算。

a fast procedure for computing the distance between complex object in 3d space

On fast computation of distance between line segments.

- 两个线段之间的最小距离；Gilbert
- 三角形顶点到三角形面之间的距离；

估计凸多面体间穿透深度的双空间展开——gamma团队

我们提出了一种增量算法来估计三维凸多面体之间的穿透深度。该算法通过在Minkowski和的表面上行走来逐步寻求“局部最优解”。Minkowski和的曲面是通过构造局部高斯映射来隐式计算的。在实践中，当环境中存在高运动相干时，该算法工作良好，并且在大多数情况下能够计算最优解。



### 穿透深度

penetration depth (PD) 穿透深度 (PD) 定义为使两个多面体的内部不相交的最小平移距离。

interpenetration



### lMinkowski Sum and Minkowski Difference

如果两个凸多边形有重叠部分，则它们的Minkowski sums也会存在一个包含原点的重叠区域。因此，在计算Minkowski sums时，我们通常将其中一个凸多边形反转一下（即取相反数），这样它的形状就成为了原凸多边形包围盒（Axis-Aligned Bounding Box或AABB）。然后我们将该AABB和另一个凸多边形作为输入，求出它们的Minkowski sums，并判断是否包含原点。

如果结果包含原点，则说明两个凸多边形之间存在重叠部分，即发生了碰撞。否则它们不重叠，即没有碰撞。



设平面图的顶点数、边数 和区域数分别为 v，e 和 f，则由欧拉公式有 v-e+f=2。

四面体，4-6+4

六面体 8-12+6



### 算法的局限性

 Dual-space Expansion for Estimating Penetration depth between convex polytopes 估计凸多面体间穿透深度的双空间展开

lMinkowski Sum 复杂度太大



当问题变得复杂时候，解决方案就会变得学术，论文很难啃，学术论文背后有很多专业的概念，比如闵可夫斯基和，还有一些作者自己定的概念，比如RSS(矩形扫掠球)，看懂论文，分析解决方案可行性，再到写实现，有很长的路要走；





---

## BV

往往包裹性越好的BV，相交测试越复杂，但是包裹性好可以减少相交测试的次数。

BVH建树困难也就意味着这个技术不适合运行时期动态更新。BVH和BSP一样往往是在非运行时期对静态物体进行预计算。

性能测试公式揭示了碰撞性能的消耗：
$$
T = NvCv + NpCp + NuCu + Co
$$
Nv是相交测试BV的数量，Cv是每次相交测试的消耗，Np是相交测试图元的数量，Cp是每次相交测试的消耗，Nu是需要更新节点的数量，Cu是更新节点的消耗，Co是一个常量消耗。

### BVH 层次包围盒(Bounding Volume Hierarchy)

BVH的核心思想就是用体积略大而几何特征简单的包围盒来近似描述复杂的几何对象，并且这种包围盒是嵌套的，我们只需要对包围盒进行进一步的相交测试，就可以越来越逼近实际对象（很明显这个功能需要用到树形的层次结构）。

![img](https://pic4.zhimg.com/80/v2-5bff610798835ab95d23a5a591e47ccb_1440w.webp)



### OBB包围盒

Oriented Bounding Box

用包围球做第一回合的快速测试，用OBB进行第二回合的测试。第一回合的测试可以剔除大多数不可见或不必裁剪的物体，这样不必进行第二回合测试的几率就会大得多。同时，OBB包围盒测试所得的结果更精确，最终要绘制的物体会更少。这种混合式的包围盒也适用于其他方面，如碰撞检测、物理/力学等

要计算两个OBB是否碰撞，只需要计算他们在图3上的4个坐标轴上的投影是否有重叠，如果有，则两多边形有接触。

判定方式：两个多边形在所有轴上的投影都发生重叠，则判定为碰撞；否则，没有发生碰撞。

OBB存在多种的表达方式，（二维）最常用的一种：一个中心点、2个矩形的边长、两个旋转轴（该轴垂直于多边形自身的边，用于投影计算）

OBB树





---



**kd-tree搜索**

kd-tree（k-dimensional树的简称），是一种对k维空间中的实例点进行存储以便对其进行快速检索的树形数据结构。 [1] 主要应用于多维空间关键数据的搜索（如：范围搜索和最近邻搜索）。K-D树是二进制空间分割树的特殊的情况。





### 体素化

三维像素化（pixel）

体素(Voxel)



平面三角形的像素化采样方法，也可以看作是三角面片与平面包围盒的求交算法，基于三角面片的求交算法也可以完成3D图形的体素化。基于空间采样的方式划分3维体素数组，使用三角形与空间AABB包围盒的求交算法确定这些基本体元所能影响到的体素单元，将这些体素标记为非空，这样就完成了对3D模型表面的体素化操作。对于内部的体素化，计算三角形到体素化网格中心点的距离，设定阈值进行判断。

