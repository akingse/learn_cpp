## 两个集合的AABB包围盒求交



### 函数效率测试

| time                                                         | debug                                                        | 1e8<br />array数组                                           | release<br />1e7，未启用omp                                  |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 临时变量                                                     | 10.366s <br />10.338s <br />10.294s                          | 开辟空间时间 <br />*2 time = 14.042s<br />装填随机数总时间<br /> time = 37.972s | 4.357s <br />4.432s <br />4.345s                             |
| 索引                                                         | 10.425s <br />10.496s <br />10.423s                          |                                                              | 4.331s <br />4.322s <br />4.354s                             |
| 整体索引                                                     | time = 9.224s<br/>time = 9.212s<br/>time = 9.216s            |                                                              | time = 4.564s<br/>time = 4.521s<br/>time = 4.555s            |
| 换_isPointInTriangular                                       | time = 9.23s<br/>time = 9.081s<br/>time = 9.117s             |                                                              | time = 4.538s<br/>time = 4.613s<br/>time = 4.674s            |
| 换eigen                                                      | debug下的eigen（1/10）<br />time = 7.032s<br/>time = 7.709s<br/>time = 8.263s |                                                              | time = 3.667s<br/>time = 3.651s<br/>time = 3.752s            |
| TriangularIntersectionTest(YQ)                               |                                                              | time = 21.059s<br/>time = 20.971s<br/>time = 20.953s         | time = 5.743s<br/>time = 5.713s<br/>time = 5.753s            |
| isTwoTrianglesIntersection1                                  |                                                              | time = 15.401s<br/>time = 15.658s<br/>time = 15.485s         | time = 3.771s<br/>time = 3.758s<br/>time = 3.856s            |
| isTwoTrianglesIntersection2                                  | 调用 fun 18041503*3=54140554<br />time = 3.031s<br/>time = 2.983s<br/>time = 2.903s | time = 10.846s<br/>time = 10.909s<br/>time = 10.802s         | time = 4.498s<br/>time = 4.484s<br/>time = 4.358s<br />      |
|                                                              |                                                              |                                                              |                                                              |
| _isPointInTriangle<br />2D<br />3D                           | 2D，去if优化                                                 | time = 0.901s<br/>time = 0.903s<br/>time = 0.907s<br />3D<br />time = 1.201s<br/>time = 1.202s<br/>time = 1.175s |                                                              |
| double<br />_isSegmentCrossTriangleSurface                   |                                                              | time = 1.732s<br/>time = 1.703s<br/>time = 1.805s            | time = 5.654s<br/>time = 5.792s<br/>time = 5.812s            |
| double<br />getTriangleBoundingCircle                        |                                                              | 先判钝角<br />time = 4.735s<br/>time = 4.797s<br/>time = 4.817s |                                                              |
| _isTwoTriangles<br />BoundingBoxIntersect<br />三角面的包围盒求交 |                                                              | eigen<br />time = 1.617s<br/>time = 1.568s<br/>time = 1.495s<br />手写<br />time = 1.228s<br/>time = 1.242s<br/>time = 1.22s |                                                              |
| 软碰撞<br />getTriDist                                       | using SAT<br />time = 47.853s<br/>time = 47.723s<br/>time = 47.657s<br />openMP<br />time = 7.771s<br/>time = 8.011s<br/>time = 8.128s | time = 56.686s<br/>time = 57.677s<br/>time = 57.643s<br />openMP<br />time = 33.12s<br/>time = 38.761s<br/>time = 39.656s | time = 9.058s<br/>time = 9.336s<br/>time = 9.22s<br />分开三角形<br />time = 6.302s<br/>time = 6.497s<br/>time = 6.521s |
| 包围盒求交<br />AlignedBox3d::intersection                   | 1e8                                                          | time = 0.658s<br/>time = 0.616s<br/>time = 0.626s            |                                                              |
| isTwoTrianglesIntersectSAT<br />分离轴定理                   | laptop strix<br />time = 4.271s<br/>time = 4.36s<br/>time = 4.311s | time = 6.034s<br/>time = 6.144s<br/>time = 6.192s<br />openmp<br />time = 1.17s<br/>time = 1.281s<br/>time = 1.277s |                                                              |
| isTriangleAndBoundingBox<br />三角面与包围盒                 |                                                              | time = 7.706s<br/>time = 7.596s<br/>time = 8.258s            |                                                              |
| 三角面与包围盒 SAT                                           |                                                              | time = 11.168s<br/>time = 11.176s<br/>time = 11.663s<br />pre-judge<br />time = 5.512s<br/>time = 5.461s<br/>time = 5.476s |                                                              |
| isPointRayAcrossTriangle                                     | SAT分离轴<br />求交点                                        | time = 1.576s<br/>time = 1.444s<br/>time = 1.447s<br />求交点<br />time = 1.512s<br/>time = 1.387s<br/>time = 1.436s |                                                              |
|                                                              |                                                              |                                                              |                                                              |



### 整体效率测试

| 模型           | 金鼓郡                                                       | 君启小区                                                     | 安装演示                                 |
| -------------- | ------------------------------------------------------------ | ------------------------------------------------------------ | ---------------------------------------- |
| 文件大小       | 27M-p3d<br />35M-rvt<br />                                   | 981M-p3d<br />81M-nwd                                        | 9.3M-p3d                                 |
| entity数量     | 5233                                                         | 67059                                                        | 14                                       |
| polyface数量   | 1982041                                                      | 36895577                                                     | 306656                                   |
| bimbase用时    | 硬碰撞：1.3s，4031，4027<br />软碰撞：1.6s，5099，5096       | 硬碰撞：30s，48718<br />不使用内部判断48677<br />使用内部判断48696<br />软碰撞：31s，49162 | 硬碰撞：0.196，17<br />软碰撞：0.187，17 |
| 康博接口       | 硬碰撞：31s，3944<br />软碰撞：185s，5180                    | 硬碰撞：18:21s，54377<br />软碰撞：--s，                     |                                          |
| navisworks用时 | 导入用时：120s<br />硬碰撞：0.8s，3954 <br />软碰撞：0.8s，4599 | 导入用时：72s<br />硬碰撞：25s，160267<br />软碰撞：34s，387139 |                                          |
| 软碰撞d=1mm    |                                                              |                                                              |                                          |

测试机型：华硕天选4笔记本，i5-11400H,16Gddr4,512SSD,RTX3050



### 阶段用时

|                          | 金鼓郡          |      |
| ------------------------ | --------------- | ---- |
| 总用时 release           | 1.601000s       |      |
| 总用时 openMP            |                 |      |
| getEntitiesOfModel       | 0.015000s       |      |
| LoadModel 转mesh最多用时 | 1.277000s-0.015 |      |
| 判convex                 | 0.037           |      |
| 预碰撞 preClash          | 0.097           |      |
| 硬碰撞                   | 0.324           |      |
| 软碰撞                   | -               |      |
|                          |                 |      |





## 碰撞检查应用场景和需求

碰撞检查在计算机图形学、虚拟现实、物理仿真和游戏开发等领域有广泛的应用，在建筑行业主要是防止冲突，并可降低建筑变更的风险。

1. 游戏开发：在游戏中，需要进行角色与环境、角色与物体之间的碰撞检查。例如，在冒险类游戏中，可以通过碰撞检查来判断角色是否跳到了一个平台上或与敌人产生了碰撞。
2. 3D建模和动画：在创建三维模型和动画时，需要进行多个对象之间的碰撞检查。例如，在电影制作中，可以利用碰撞检查来保证特效和物体的合理交互。
3. 虚拟现实和增强现实：在虚拟现实和增强现实应用中，需要将虚拟对象与真实世界进行交互。碰撞检查使得虚拟对象能够与用户的身体或真实对象进行碰撞，并给出相应的反馈。
4. 物理仿真：在物理仿真中，需要模拟各种物体之间的力和碰撞。通过碰撞检查，可以模拟物体之间的碰撞行为，包括弹性碰撞、摩擦力、动量守恒等。
5. 机器人路径规划：在机器人的运动控制中，需要检查机器人所处位置与障碍物之间是否存在碰撞。通过碰撞检测可以有效避免机器人与障碍物发生碰撞，并规划出安全可行的路径。

### 建筑行业

1. 建筑设计和规划：在建筑设计过程中，需要确保各个构件、设备和管道之间不存在冲突或碰撞。通过进行碰撞检查，可以及早发现并解决潜在的冲突问题，避免出现安装难题。
2. 施工和制造：在实际施工和制造过程中，需要考虑材料和设备的占据空间，并与其他物体进行合理组织。通过碰撞检查，可以确认不同元素的位置和运动路径，避免发生碰撞而损坏现有结构或设备。
3. 安全性分析：碰撞检查还可以用于做建筑结构的安全性分析。例如，在火灾疏散分析中，需要考虑是否存在紧急通道被堵塞的情况，通过对人流路径进行碰撞检测可以评估疏散效果和优化路径规划。
4. 模型展示和交互：在建筑项目的模型展示和虚拟现实应用中，可以利用碰撞检查来实现用户与虚拟环境的交互。例如，在虚拟漫游中，用户可以通过碰撞检测来避免与虚拟建筑或物体发生穿越。

### 市面常见的软件：

1. Autodesk Navisworks：这是一种广泛应用于建筑、工程和施工项目的3D协同审核和可视化软件。它可以进行模型合并、冲突检测和协调等功能。
2. Revit：Revit是由Autodesk开发的智能建筑信息建模 (BIM) 软件。除了提供建筑设计和绘图工具，它还具备碰撞检测功能来发现构件之间的冲突。
3. SOLIDWORKS：SOLIDWORKS是一款三维机械CAD软件，广泛应用于设计领域。其中包含了配套的碰撞检查工具，用于识别和解决机械部件之间的干涉问题。
4. Rhino/Grasshopper：Rhino是一种流行的三维建模软件，而Grasshopper是其插件之一，用于生成算法和参数化建模。通过使用自定义脚本和插件，可以实现具有碰撞检测功能的定制化工作流程。
5. SimScale：SimScale是一种基于云计算的仿真平台，支持各种物理仿真应用。它也提供碰撞检测功能，可以帮助设计和工程团队分析不同部件之间的冲突。

### 对标Navisworks

目前我们的碰撞检查功能，采用离散mesh的策略（与Navisworks相同），对模型兼容性更好。具有硬碰撞检测，和指定公差值的软碰撞检测功能（侵入距离计算功能正在研发中），支持选择树，支持进度条显示，支持碰撞结果的可视化查看，支持导出xml碰撞信息。我们的优势在于，省去导入时间，直接在BIMBase内部使用，速度快而精准，学习曲线简单，免费，电脑配置要求低。目前对于大型模型，在包含模型导入时间的前提下，检测速度基本对标Navisworks，还有进一步优化空间。检测精度已通过自测模型验证。

目前我们在碰撞检查的核心算法方面有突破，在碰撞检测环节所需时间上与Navisworks相比是接近甚至于要更快。我们希望知道，在哪些领域我们有机会超越该领域的主流软件，以及主流软件尚存的痛点问题是我们可以尝试解决的。最终希望可以将技术成果有效地转化为产值。



---

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
// triangle-point is all left
if ((dot0 > eps && dot1 > eps && dot2 > eps) || (dot0 < _eps && dot1 < _eps && dot2 < _eps))
    return false;
```

### 两线段相交（跨立实验）

```c
//double straddling test
Vector3d vecSeg = segment[1] - segment[0];
if (!((trigon[0] - segment[0]).cross(vecSeg).dot(vecSeg.cross(trigon[1] - segment[0])) > _eps ||
      (segment[0] - trigon[0]).cross(trigon[1] - trigon[0]).dot((trigon[1] - trigon[0]).cross(segment[1] - trigon[0])) < _eps))
    return true;

```

### 点在面左侧

```c
Vector3d normal = (plane[1] - plane[0]).cross(plane[2] - plane[1]);
bool isLeft = (point - plane[0]).dot(normal) < 0.0;

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
./flatc --cpp convert_to_mesh.fbs

./flatc --python convert_to_mesh.fbs
```







新bug，mesh为空，君启小区模型，count_err_empty_mesh="44345"
