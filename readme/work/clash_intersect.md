## 两个集合的AABB包围盒求交

### AABB包围盒



AABB包围盒(Axis-Aligned Bounding Box) 是一种简单的包围体,它使用两个点来表示一个轴对齐的矩形区域。
AABB包围盒具有以下特点:

1. 两个点分别表示包围盒的最小点(min)和最大点(max)。
2. 包围盒的边界与坐标轴的方向一致,边界线与坐标轴之间的夹角为90度。因此AABB包围盒也称作坐标轴对齐的包围盒。
3. AABB包围盒简单易构造,Checking是否包含某点的计算量小。所以非常适合用于快速碰撞检测等。
4. 能够很好地近似许多物体的外形,具有一定的适用性。
5. 当物体发生平移、缩放变换时,AABB包围盒也可以很简单地进行相应变换。
    AABB包围盒经常用于3D空间中的物体包围和碰撞检测。
    其中蓝色点表示包围盒的8个顶点,红色点表示包围盒的最小点min和最大点max。
    所以,简而言之,AABB包围盒是一种简单的包围盒,它使用两个轴对齐的点来表示包围区域,广泛用于3D物体包围和碰撞检测领域。



### eigen 

使用Eigen库计算两个AABB包围盒的相交可以遵循下面的步骤：

1. 首先定义两个AABB包围盒，每个AABB由其最小顶点(minPoint)和最大顶点(maxPoint)定义。

```c++
#include <Eigen/Core>
Eigen::Vector3f minPointA, maxPointA, minPointB, maxPointB;
// 然后分别为两个AABB的最小顶点和最大顶点赋值，例如：
minPointA << -1, -1, -1;
maxPointA << 1, 1, 1;
minPointB << 0, 0, 0;
maxPointB << 2, 2, 2;
```

2. 通过使用Eigen的min和max函数，取出两个AABB包围盒的重合部分的最小顶点和最大顶点。

```c++
Eigen::Vector3f minIntersection(maxPointA.cwiseMax(minPointB));
Eigen::Vector3f maxIntersection(minPointA.cwiseMin(maxPointB));
```

这里使用了`cwiseMax`和`cwiseMin`函数，它们会对每个元素求最大值和最小值，以支持对向量和矩阵中的每个元素执行逐元素操作。

3. 最后，检查两个重合部分的最小顶点和最大顶点是否相等，以确定两个AABB包围盒是否相交。

```c++
bool isIntersected = ((maxIntersection - minIntersection).array() > 0).all();
```



如果使用Eigen的AlignedBox类，可以更简洁地计算两个AABB包围盒的相交，代码如下:

```c++
#include <Eigen/Geometry>

int main()
{
    Eigen::AlignedBox3f boxA(Eigen::Vector3f(-1, -1, -1), Eigen::Vector3f(1, 1, 1));
    Eigen::AlignedBox3f boxB(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(2, 2, 2));

    bool isIntersected = boxA.intersects(boxB);

    return 0;
}
```

这里使用AlignedBox3f类定义了两个AABB包围盒，方法intersects()返回一个bool值，表示两个AABB包围盒是否相交。

intersects()方法的返回值基于以下定理：如果两个AABB相交，则它们沿每个轴至少有一个重叠段，例如在x轴上的重叠段为[Ax1, Ax2]和[Bx1, Bx2]，其中Ax1<=Ax2，Bx1<=Bx2。在这些重叠段中选择长度最小的，所有轴上重叠长度最小的重叠段的交集即是两个AABB的重叠部分。如果存在任何重叠段长度为零，则两个AABB不相交。

因此，使用AlignedBox类计算两个AABB包围盒的相交，可以减少代码数量并保证正确性。



### 快速求交

两个AABB包围盒的集合，想要快速求出它们相交的部分，你可以考虑使用空间分割数据结构，例如kd-tree或octree。这些数据结构可以将空间划分成不同的块或箱子，以有效地提高搜索和查询速度。



### 平面扫面算法

平面扫面算法是一种广泛用于3D环境中的碰撞检测和物体间相交计算的算法。它的基本思想是:
1. 将3D空间划分为多个平面,每个平面上放置一组物体或物体的包围体(通常是AABB包围盒)。
2. 选择一个轴(通常是y轴或z轴),沿着该轴方向逐渐"扫过"每个平面。
3. 在扫过每个平面时,检测该平面上的物体或包围体之间是否有碰撞或相交。
4. 通过扫面顺序,我们只需要在相邻平面之间检测碰撞,这减少了整个3D空间的碰撞检测量。
5. 由于物体只在某个平面上移动,所以我们可以通过追踪物体在上一平面上的状态来预测并优化当前平面上的碰撞检测。
这个算法的优点是:
1. 将3D碰撞检测问题转换为2D,降低了计算复杂度。
2. 采用增量式检测,避免重复检测,提高效率。
3. 可预测并优化相邻平面之间的碰撞检测。
4. 空间上的连续物体,在平面扫面过程中也是连续的,这有利于相交计算。
它的基本实现过程是:
1. 定义平面和轴,初始化平面上的物体和包围体。
2. 选择第一个平面,检测平面上的碰撞和相交。
3. 移动到下一个平面,预测并优化与上一平面碰撞相关的检测。同时检测新产生的碰撞。
4. 重复步骤3,逐层扫过每个平面。
5. 所有平面扫过后结束,得到最终的碰撞和相交结果。



### 效率测试 

1e7，未启用omp

| time                                                         | debug                                                        | 1e8<br />array数组                                           | release<br />1e7                                             |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 临时变量                                                     | 10.366s <br />10.338s <br />10.294s                          | 开辟空间时间 <br />*2 time = 14.042s<br />装填随机数总时间<br /> time = 37.972s | 4.357s <br />4.432s <br />4.345s                             |
| 索引                                                         | 10.425s <br />10.496s <br />10.423s                          |                                                              | 4.331s <br />4.322s <br />4.354s                             |
| 整体索引                                                     | time = 9.224s<br/>time = 9.212s<br/>time = 9.216s            |                                                              | time = 4.564s<br/>time = 4.521s<br/>time = 4.555s            |
| 换_isPointInTriangular                                       | time = 9.23s<br/>time = 9.081s<br/>time = 9.117s             |                                                              | time = 4.538s<br/>time = 4.613s<br/>time = 4.674s            |
| 换eigen                                                      | debug下的eigen（1/10）<br />time = 7.032s<br/>time = 7.709s<br/>time = 8.263s |                                                              | time = 3.667s<br/>time = 3.651s<br/>time = 3.752s            |
| TriangularIntersectionTest                                   |                                                              | time = 21.059s<br/>time = 20.971s<br/>time = 20.953s         | time = 5.743s<br/>time = 5.713s<br/>time = 5.753s            |
| isTwoTrianglesIntersection1                                  |                                                              | time = 4.588s<br/>time = 4.574s<br/>time = 4.575s<br />time = 15.401s<br/>time = 15.658s<br/>time = 15.485s | time = 3.771s<br/>time = 3.758s<br/>time = 3.856s            |
| isTwoTrianglesIntersection2                                  | 调用 fun 18041503*3=54140554<br />time = 3.031s<br/>time = 2.983s<br/>time = 2.903s | time = 10.846s<br/>time = 10.909s<br/>time = 10.802s         | time = 3.723s<br/>time = 3.578s<br/>time = 3.984s<br />latest<br />time = 4.498s<br/>time = 4.484s<br/>time = 4.358s<br /> |
| _isPointInTriangle<br />(randData3[i][0], randData3_[i])     | 随机数 1e7<br />3*24197379=72587976                          | time = 1.252s<br/>time = 1.266s<br/>time = 1.279s            | time = 1.686s<br/>time = 1.671s<br/>time = 1.675s            |
| double<br />_isSegmentCrossTriangleSurface                   |                                                              | time = 1.732s<br/>time = 1.703s<br/>time = 1.805s            | time = 5.654s<br/>time = 5.792s<br/>time = 5.812s            |
| double<br />getTriangleBoundingCircle                        |                                                              | 先判钝角<br />time = 4.735s<br/>time = 4.797s<br/>time = 4.817s |                                                              |
| _isTwoTriangles<br />BoundingBoxIntersect<br />三角面的包围盒求交 |                                                              | eigen<br />time = 1.617s<br/>time = 1.568s<br/>time = 1.495s<br />手写<br />time = 1.228s<br/>time = 1.242s<br/>time = 1.22s |                                                              |
| 软碰撞<br />_getTriDist                                      |                                                              | time = 56.686s<br/>time = 57.677s<br/>time = 57.643s<br />   | time = 9.058s<br/>time = 9.336s<br/>time = 9.22s<br />分开三角形<br />time = 6.302s<br/>time = 6.497s<br/>time = 6.521s |
| 包围盒求交<br />AlignedBox3d::intersection                   | 1e8                                                          | time = 0.658s<br/>time = 0.616s<br/>time = 0.626s            |                                                              |
| 分离轴定理                                                   | laptop strix<br />time = 4.271s<br/>time = 4.36s<br/>time = 4.311s | time = 6.034s<br/>time = 6.144s<br/>time = 6.192s            |                                                              |
| isTriangleAndBoundingBox<br />三角面与包围盒                 |                                                              | time = 7.706s<br/>time = 7.596s<br/>time = 8.258s            |                                                              |
| 三角面与包围盒 SAT                                           |                                                              | time = 11.168s<br/>time = 11.176s<br/>time = 11.663s<br />pre-judge<br />time = 5.512s<br/>time = 5.461s<br/>time = 5.476s |                                                              |





### 算法流程 TwoTrianglesIntersect

1 判断边是否穿越面，包括点在面上（点在面左侧 isLeftTest*3），全部不穿越 return false

2 求边与三角形平面的交点

​	2.1 边与三角形共面，判断判线段是否与三角形相交 return true

（共面，预判，3点在线段左侧 isLeftTest* 3）
（预判，共面点在三角形内部）
（共面，两线段相交，isLeftTest2* 3）

​	2.2 算出交点，判断交点是否在三角形内部 return true

（共面，点在边左侧 isLeftTest2* 3）

3 两三角形相互判断，共6次，return false

```
dot 3*,2+
cross 6*,3-
```



### 两点在面两侧

```c
//alg
Vector3d veczL = (triL[1] - triL[0]).cross(triL[2] - triL[0]); //面法向量
bool acrossR2L_A = (veczL.dot(triR[0] - triL[0])) * (veczL.dot(triR[1] - triL[0])) < eps;
bool acrossR2L_B = (veczL.dot(triR[1] - triL[0])) * (veczL.dot(triR[2] - triL[0])) < eps;
bool acrossR2L_C = (veczL.dot(triR[2] - triL[0])) * (veczL.dot(triR[0] - triL[0])) < eps;
//critical include
(veczL.dot(triR[0] - triL[0]))==0
(veczL.dot(triR[1] - triL[0]))==0
bool acrossR2L_A = true
    
//illegal collinear
if (veczL.norm()==0)
	bool acrossR2L_A = (veczL.dot(triR[0] - triL[0]))==0 && (veczL.dot(triR[1] - triL[0]))==0 
    
```

### 点在线段左侧

```c
//alg
Vector3d vecX = (segment[1] - segment[0]).cross((trigon[1] - trigon[0]).cross(trigon[2] - trigon[0])); // axisY cross axisZ
double dot0 = vecX.dot(trigon[0] - segment[0]);
double dot1 = vecX.dot(trigon[1] - segment[0]);
double dot2 = vecX.dot(trigon[2] - segment[0]);

//critical include



```



### 两点在线段两侧（跨立实验）

```c
//double straddling test
Vector3d vecSeg = segment[1] - segment[0];
if (!((trigon[0] - segment[0]).cross(vecSeg).dot(vecSeg.cross(trigon[1] - segment[0])) > _eps ||
      (segment[0] - trigon[0]).cross(trigon[1] - trigon[0]).dot((trigon[1] - trigon[0]).cross(segment[1] - trigon[0])) < _eps))
    return true;

```



### 分离轴定理 [SAT(Separating Axis Theorem)](https://dyn4j.org/2010/01/sat/#sat-axes)

凸多边形

如果某个形状与任何穿过该形状的直线只交叉两次，则该形状被称为凸多边形。如果某个形状与穿过该形状的直线交叉两次以上，则该形状为非凸（或凹）。

SAT只能处理凸多边形，不过，非凸形状可以由凸形状的组合来表示（凸分解）

SAT指出：“**如果两个凸面物体没有穿透，则存在一根轴，使得这两个物体在该轴上的投影不重叠。**”



测试多个轴判断是否重叠。只要投影不重叠，算法可以立即确定形状不相交，从而退出循环。

```c
Axis[] axes = // get the axes to test;
// loop over the axes
for (int i = 0; i < axes.length; i++) 
{
  Axis axis = axes[i];
  // project both shapes onto the axis
  Projection p1 = shape1.project(axis);
  Projection p2 = shape2.project(axis);
  // do the projections overlap?
  if (!p1.overlap(p2)) {
    // then we can guarantee that the shapes do not overlap
    return false;
  }
}
// if we get here then we know that every axis had overlap on it
// so we can guarantee an intersection
return true;

分离轴定理，三维空间中两个三角形是否相交
1共面，计算法向量和顶点之间的距离
2不共面，15个分离轴，6个边各自法向量和对边叉积
    每个分离轴，计算三角形在轴上的投影，有分离则返回false，最后返回true
    
    
```



### 分离轴定理

- 分离轴定理算法十分得快

- 分离轴定理算法十分得准

- 分离轴定理算法只适用于凸多边形，除非你把它们分成一些小的凸多边形，然后依次检验这些小的多边形。
- 分离轴定理算法无法告诉你是那条边发生的碰撞——仅仅是告诉你重叠了多少和分开它们所需的最短距离。

#### 凸特性

  如前所述，SAT是一种确定两个凸多边形是否相交的方法。如果某个形状与任何穿过该形状的直线只交叉两次，则该形状被称为凸多边形。如果某个形状与穿过该形状的直线交叉两次以上，则该形状为非凸（或凹）。

SAT指出：“**如果两个凸面物体没有穿透，则存在一根轴，使得这两个物体在该轴上的投影不重叠。**”

二维凸包，获取分离轴
  **只要测试每个多边形的边的法线即可**





---



bug修复

1，测试当前硬碰撞结果 

关优化 total="4270"

开优化 total="4030"

使用SAT

关优化 total="4031"



```c
//顶点缓冲对象：Vertex Buffer Object，VBO
    std::vector<Eigen::Vector3d> vbo_; //所有顶点

//索引缓冲对象： Index Buffer Object，IBO
    std::vector<std::array<int, 3>> ibo_; //索引组


5749.9999999999054
5750.000000
```



破案了，测试程序当 if(f1()!=f2())时，装填数据，但是导出xml的时候手动粘贴数据去测试，总是f1()==f2()

原因是，程序内存里的数据跟xml导出的数据不一致，std::to_string时将数字取整了 









