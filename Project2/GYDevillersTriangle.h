#pragma once
//GYDevillersTriangle.h
/*      快速检测空间三角形相交算法的代码实现(Devillers & Guigue算法)
博客原地址:http://blog.csdn.net/fourierfeng/article/details/11969915#

Devillers & Guigue算法(简称Devillers 算法) 通过三角形各顶点构成的行列式正负的几何意义来判断三角形中点、线、面之间的相对位置关系,
从而判断两三角形是否相交。其基本原理如下:给定空间四个点：a(ax, ay, az), b = (bx, by, bz), c = (cx, cy, cz), d = (dx, dy, dz), 定义行列式如下：

[a, b, c, d] 采用右手螺旋法则定义了四个空间点的位置关系。
[a, b, c, d] > 0 表示 d 在 a、b、c 按逆时针顺序所组成的三角形的正法线方向(即上方);
[a, b, c, d] < 0 表示 d 在 △abc的下方; [a, b, c, d] = 0 表示四点共面。

        设两个三角形T1和T2，顶点分别为：V10，V11，V12和V20，V21，V22，
    三角形所在的平面分别为π1和π2，其法向量分别为N1和N2.算法先判别三角形和另一个三角形所在的平面的相互位置关系, 提前排除不相交的情况。
    通过计算[V20, V21, V22, V1i].(i = 0, 1, 2)来判断T1和π2的关系：如果所有的行列式的值都不为零且同号，则T1和T2不相交；否则T1和π2相交。
    相交又分为如下几种情况：
        a)如果所有的行列式的值为零，则T1和T2共面，转化为共面的线段相交问题。
        b)如果其中一个行列式的值为零，而其他两个行列式同号，则只有一个点在平面内，测试顶点是否则T2内部，是则相交，否则不相交；
        c)否则T1的顶点位于平面π2两侧(包含T1的一条边在平面π2中的情况)。

        再按照类似的方法对 T 2 和 π 1 作进一步的测试。如果通过测试, 则每个三角形必有确定的一点位于另一个三角形所在平面的一侧,
    而另外两点位于其另一侧。算法分别循环置换每个三角形的顶点, 以使V10(V20)位于π2(π1)的一侧，另两个点位于其另一侧；
    同时对顶点V21，V22(V11, V12)进行交换操作，以确保V10(V20)位于π2(π1)的上方，即正法线方向。
    经过以上的预排除和置换操作，V10的邻边V10V11，V10V12和V20的邻边V20V21和V20V22与两平面的交线L相交于固定形式的点上，
    分别记为i，j，k，l(i<j, k<l), 如图：(参看原博客)
    这些点在L上形成的封闭区间为i1 = [i, j], i2 = [k, l].至此，两个三角形的相交测试问题转换为封闭区间i1，i2的重叠问题。
    若重叠则相交，否则不相交。由于交点形式固定，只需满足条件k <= j且i <= l即表明区间重叠，条件还可进一步缩减为判别式
    (1)是否成立：
                [V10, V11, V20, V21] <= 0 && [V10, V12, V22, V20] <= 0        判别式(1)
*/

typedef float float3[];

enum TopologicalStructure
{
    INTERSECT, NONINTERSECT
};

struct Triangle
{
    //float3 Normal_0;
    //float3 Vertex_1, Vertex_2, Vertex_3;
};

/*******************************************************************************************************/
//Devillers算法主函数
TopologicalStructure judge_triangle_topologicalStructure(Triangle* tri1, Triangle* tri2);

//返回bool值
bool isTriangleTntersect(Triangle* tri1, Triangle* tri2)
{
    TopologicalStructure  intersectSt = judge_triangle_topologicalStructure(tri1, tri2);
    if (intersectSt == INTERSECT)
        return true;
    return false;
}