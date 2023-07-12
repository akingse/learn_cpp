## 两个集合的AABB包围盒求交



### 函数效率测试

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
| _isPointInTriangle<br />2D<br />3D                           |                                                              | time = 1.273s<br/>time = 1.271s<br/>time = 1.295s<br />3D<br />time = 1.337s<br/>time = 1.328s<br/>time = 1.371s |                                                              |
| double<br />_isSegmentCrossTriangleSurface                   |                                                              | time = 1.732s<br/>time = 1.703s<br/>time = 1.805s            | time = 5.654s<br/>time = 5.792s<br/>time = 5.812s            |
| double<br />getTriangleBoundingCircle                        |                                                              | 先判钝角<br />time = 4.735s<br/>time = 4.797s<br/>time = 4.817s |                                                              |
| _isTwoTriangles<br />BoundingBoxIntersect<br />三角面的包围盒求交 |                                                              | eigen<br />time = 1.617s<br/>time = 1.568s<br/>time = 1.495s<br />手写<br />time = 1.228s<br/>time = 1.242s<br/>time = 1.22s |                                                              |
| 软碰撞<br />_getTriDist                                      |                                                              | time = 56.686s<br/>time = 57.677s<br/>time = 57.643s<br />prejudge<br />time = 43.753s<br/>time = 44.354s<br/>time = 45.596s | time = 9.058s<br/>time = 9.336s<br/>time = 9.22s<br />分开三角形<br />time = 6.302s<br/>time = 6.497s<br/>time = 6.521s |
| 包围盒求交<br />AlignedBox3d::intersection                   | 1e8                                                          | time = 0.658s<br/>time = 0.616s<br/>time = 0.626s            |                                                              |
| 分离轴定理                                                   | laptop strix<br />time = 4.271s<br/>time = 4.36s<br/>time = 4.311s | time = 6.034s<br/>time = 6.144s<br/>time = 6.192s            |                                                              |
| isTriangleAndBoundingBox<br />三角面与包围盒                 |                                                              | time = 7.706s<br/>time = 7.596s<br/>time = 8.258s            |                                                              |
| 三角面与包围盒 SAT                                           |                                                              | time = 11.168s<br/>time = 11.176s<br/>time = 11.663s<br />pre-judge<br />time = 5.512s<br/>time = 5.461s<br/>time = 5.476s |                                                              |
| isPointRayAcrossTriangle                                     | SAT分离轴<br />求交点                                        | time = 1.576s<br/>time = 1.444s<br/>time = 1.447s<br />求交点<br />time = 1.512s<br/>time = 1.387s<br/>time = 1.436s |                                                              |
|                                                              |                                                              |                                                              |                                                              |



### 整体效率测试

| 模型           | 金鼓郡                                                 | 君启小区                                        |
| -------------- | ------------------------------------------------------ | ----------------------------------------------- |
| 文件大小       | 27M-p3d                                                | 35M-rvt                                         |
| entity数量     | 5233                                                   |                                                 |
| polyface数量   | 1982041                                                |                                                 |
| bimbase用时    | 硬碰撞d=0：1.3s<br />软碰撞：1.6s                      | 硬碰撞d=0：30s<br />软碰撞：31s                 |
| 康博接口       | 硬碰撞d=0：31s<br />软碰撞：3分钟05s                   |                                                 |
| navisworks用时 | 导入用时：120s<br />硬碰撞d=0：0.8s <br />软碰撞：0.8s | 导入用时：<br />硬碰撞d=0：25s<br />软碰撞：34s |

测试机型：天选4笔记本，i5-11400H,16Gddr4,512SSD,RTX3050







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







---



bug修复

1，测试当前硬碰撞结果 

关优化 total="4270"

开优化 total="4030"

使用SAT

关优化 total="4030" count_caltime2="7.944000s"/>，其中count_caltime1="1.273000s"

开优化 total="4030" 修复完成 count_caltime2="1.485000s"/>

未优化，TriangularIntersectC="132430306"，count_err_degen_tri="13442" 

优化，count_soft_exclude_tris="920799"，TriangularIntersectC="124977"

```
硬碰撞bug，退化的三角形
			if (clash_info.object_1.m_entityId== 6509 && clash_info.object_2.m_entityId == 6134)
				startT = clock();
				
软碰撞bug，是否存在d=0的漏检，或者getTrianglesDist计算错误
#ifdef DETECTION_DEBUG_SOFT // error with precision
				if (temp == 0) //there is no contact
					count_err_dist++; //break
#endif 
```





2 软碰撞 tolerance="0.001000"

关优化 total="5099" 

开优化

优化策略1，_getReducedIntersectTrianglesOfMesh，mesh的三角面与pre-box包围盒预碰撞（包含公差），排除不相交的三角面

优化策略2，isTwoTrianglesBoundingBoxIntersect，距离计算之前先计算两三角面是否相交（包含公差），排除不相交的三角面

开优化 total="5099"，count_caltime2="1.626000s"/>









```c
//顶点缓冲对象：Vertex Buffer Object，VBO
    std::vector<Eigen::Vector3d> vbo_; //所有顶点

//索引缓冲对象： Index Buffer Object，IBO
    std::vector<std::array<int, 3>> ibo_; //索引组


5749.9999999999054
5750.000000
```



破案了，测试程序当 if(f1()!=f2())时，装填数据，但是导出xml的时候手动粘贴数据去测试，总是f1()==f2()

原因是，程序内存里的数据跟xml导出的数据不一致，std::to_string时将数字取整了，更换函数stringstream，精度也不是100%还原，中间的算法对精度太敏感了，导致测试和输出结果对不上；

使用写入二进制文件来还原数据，可以精确定位到bug，两个优化策略，第一个生效，第二个暂时有bug

优化1，getReduced，count_caltime1="1.184000s" count_caltime2="1.352000s"/>

当三角面有拓扑错误，顶点重合时，SAT有bug



### 程序流程 ClashDetection

1 LoadModel 加载模型

​	得到world_bounding整体包围盒，BPEntityId及对应ModelInfo（mesh，select，box）

2 BoundingClashDetection 预碰撞

​	得到包围盒碰撞了的BPEntityId对（包含公差）；

​	is_soft=false 无公差，硬碰撞
​	is_soft=false 有公差，但原始包围盒碰撞
​	is_soft=true，有公差，原始box separate分离，但是在公差内（enlarge box）碰撞

3 构造ClashInfo碰撞信息（BPEntityId，box，distance，position）

4 处理hard-clash硬碰撞，MeshIntrusionTesting

​	侵入距离计算MeshIntrusionDistance，装填ClashInfo

5 处理soft-clash软碰撞，MeshStandoffDistance

​	得到distance和两个mesh的position，装填ClashInfo



### 使用FlatBuffers

定义数据结构：创建一个`.fbs`文件（变量命名有要求，注意ulong）

生成代码：使用FlatBuffers编译器（`flatc`）将`.fbs`文件编译为对应的C++或其他语言的代码

```shell
./flatc --cpp inter_triangels_info.fbs
```



