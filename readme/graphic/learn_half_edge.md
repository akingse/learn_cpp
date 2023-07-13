## 几何数据结构

Boundary Representation

三维物体在计算机内的表示需要用具体的数据结构来实现,实体造型系统对物体的各种分析运算和操作依赖于一定的数据结构的支持。



边界表示的一个重要特征是描述形体的信息包括几何信息（Geometry）和拓朴信息（Topology）两个方面。拓朴信息描述形体上的顶点、边、面的连接关系，它形成物体边界表示的“骨架”。形体的几何信息犹如附着在“骨架”上的肌肉。例如，形体的某个面位于某一个曲面上，定义这一曲面方程的数据就是几何信息。此外，边的形状、顶点在三维空间中的位置（点的坐标）等都是几何信息，一般来说，几何信息描述形体的大小、尺寸、位置和形状等。

Brep表示需要对定义形体的面、环、边、点及其属性进行存取、直接查找、间接查找和逆向查找等操作；





---



### 翼边结构（winged-edge）

![image-20230325224222009](https://raw.githubusercontent.com/akingse/my-picbed/main/image-20230325224222009.png)

### 半边数据结构（The_Half-Edge_Data_Structure）

[flipcode](https://www.flipcode.com/archives/The_Half-Edge_Data_Structure.shtml)

![image-20230325224314569](https://raw.githubusercontent.com/akingse/my-picbed/main/image-20230325224314569.png)

Vertex 顶点
	编号 index
	坐标 point
	半边 halfedge*

HalfEdge 半边
	编号 index
	起点  vertex*
	全边 edge*
	三角面 polyface*
	半边前驱后继 prev* next*
	另半边 halfedge*
Edge 全边
	编号 index
	顶点 vertex* vertex*
	半边 halfedge*
三角面 Polyface
	编号 index
	开始半边 halfedge*

实体 SolidMesh
    std::vector<MHalfedge *> half_edges_;
    std::vector<MVert *> vertices_;
    std::vector<MEdge *> edges_;
    std::vector<MPolyFace *> polygons_;