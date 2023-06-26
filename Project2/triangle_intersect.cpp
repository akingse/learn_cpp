#include "pch.h"
using namespace std;
using namespace para;
using namespace Eigen;
using namespace psykronix;
#undef max
#undef min
static constexpr double eps = 1e-14; //DBL_EPSILON
static constexpr double _eps = -eps;
class TriangleVec
{
public:
	Vec3 m_p0;
	Vec3 m_p1;
	Vec3 m_p2;
};
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

//struct Triangle
//{
//	//float3 Normal_0;
//	//float3 Vertex_1, Vertex_2, Vertex_3;
//};

/*******************************************************************************************************/
//Devillers算法主函数
TopologicalStructure judge_triangle_topologicalStructure(Triangle* tri1, Triangle* tri2)
{
	return {};
}

//返回bool值
bool isTriangleTntersect(Triangle* tri1, Triangle* tri2)
{
	TopologicalStructure  intersectSt = judge_triangle_topologicalStructure(tri1, tri2);
	if (intersectSt == INTERSECT)
		return true;
	return false;
}

typedef std::tuple<char, int, double> TC;
namespace std
{
	//template <class T,class U,class V>
	//class tuple<T, U, V>
	//{
	//public:
	//	T operator[](int i)
	//	{

	//	}
	//};
}

/*
一般来说，可以用两个方法来检测三角形是否相交:
投影算法:
利用投影理论，可以将三维空间中的三角形，投影到任意一个平
面上，观察其投影之后，是否出现重叠，从而推断是否有相交;
三角剖分算法:
将两个三角形进行三角剖分，比较两个三角形的边、顶点是否有
交点，如有交点，则两个三角形可以认定为相交;

*/

enum class COLLISION :int
{
	T_SEPA,
	T_INTER,
	T_TANG,

};

COLLISION is_two_triangle_intersec(const TriangleVec& triA, const TriangleVec& triB)
{
	return {};
}


static void _test()
{
	std::array<std::array<double, 3>, 3> triangle = { { {0,0,0},{1,1,1}, {2,2,2}} };
	//int PQP_BV_TYPE = RSS_TYPE | OBB_TYPE;
	//int macro1 = PQP_BV_TYPE & RSS_TYPE;
	//int macro2 = PQP_BV_TYPE & OBB_TYPE;
	std::tuple<char, int, double> mytp;
	mytp = { 'a',1,2.0 };

	double nums[10] = { rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand() };
	//for (int i = 0; i < sizeof(nums)/sizeof(double); i++)
	//	cout << nums[i] << " | ";

		//bool isTI = TriangleIntersectionTest({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	//double td = getTrianglesDistance(P, Q, { triA0,triA1,triA2 }, { triB0,triB1,triB2 }); //43.905043670162655
	double a = 3;
	float b = 3;
	double c = a - b;
	double d = max(a, c);
	//psykronix::Vertex vec(1, 1, 1);
	//psykronix::Vertex vec2 = vec;
}


static void _test1()
{
	Vector3d triA_0 = Vector3d(4924494.8122771187, -385870.18283433426, 5749.9999999999054);
	Vector3d triA_1 = Vector3d(4924599.8122771177, -385945.18283433421, 5749.9999999999054);
	Vector3d triA_2 = Vector3d(4924586.8713248633, -385946.88654301979, 5749.9999999999054);
	Vector3d triB_0 = Vector3d(4924601.8102601077, -385940.89359764993, 5750.0000000000000);
	Vector3d triB_1 = Vector3d(4924595.2577039087, -385951.32193110074, 5750.0000000000000);
	Vector3d triB_2 = Vector3d(4924589.8109916430, -385975.18553675216, 5750.0000000000000);
	bool isTI = isTwoTrianglesIntersection({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	bool isTIT = TriangularIntersectionTest({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	cout << "return 0" << endl;
}


static int enrol = []()->int
{
	_test1();
	return 0;
}();

//离轴定理，包围盒与三角形求交

/*
找到与三角形平面平行的两个AABB顶点
测试三角形的三条边与AABB的三条轴的九种组合
测试三角形的顶点是否在AABB内
这个算法速度快且准确。使用分离轴定理，我们只需要测试13个潜在的分离轴是否存在。
如果有任何一个分离轴存在，那么三角形和AABB不相交。如果所有潜在的分离轴都不存在，则三角形和AABB相交。
*/

//2D triangles
bool psykronix::isTrianglesIntersectSAT(const std::array<Eigen::Vector2d, 3>& triA, const std::array<Eigen::Vector2d, 3>& triB)
{
	std::array< Vector2d, 6> axes;
	Vector2d axisA0 = triA[1] - triA[0];
	Vector2d axisA1 = triA[2] - triA[1];
	Vector2d axisA2 = triA[0] - triA[2];
	std::swap(axisA0.x(), axisA0.y()); //get the normal vector
	std::swap(axisA1.x(), axisA1.y());
	std::swap(axisA2.x(), axisA2.y());
	Vector2d axisB0 = triB[1] - triB[0];
	Vector2d axisB1 = triB[2] - triB[1];
	Vector2d axisB2 = triB[0] - triB[2];
	std::swap(axisB0.x(), axisB0.y()); //get the normal vector
	std::swap(axisB1.x(), axisB1.y());
	std::swap(axisB2.x(), axisB2.y());
	axes[0] = axisA0;
	axes[1] = axisA1;
	axes[2] = axisA2;
	axes[3] = axisB0;
	axes[4] = axisB1;
	axes[5] = axisB2;
	//projection
	//double maxA0 = std::max({ axisA0.dot(triA[0]), axisA0.dot(triA[1]), axisA0.dot(triA[2]) });
	//double minA0 = std::min({ axisA0.dot(triA[0]), axisA0.dot(triA[1]), axisA0.dot(triA[2]) });
	//double maxB0 = std::max({ axisA0.dot(triB[0]), axisA0.dot(triB[1]), axisA0.dot(triB[2]) });
	//double minB0 = std::min({ axisA0.dot(triB[0]), axisA0.dot(triB[1]), axisA0.dot(triB[2]) });
	//if (maxA0 - minB0 < _eps || maxB0 - minA0 < _eps)//(maxA0 < minB0 || maxB0 < minA0)
	//	return false;
	//double maxA1 = std::max({ axisA1.dot(triA[0]), axisA1.dot(triA[1]), axisA1.dot(triA[2]) });
	//double minA1 = std::min({ axisA1.dot(triA[0]), axisA1.dot(triA[1]), axisA1.dot(triA[2]) });
	//double maxB1 = std::max({ axisA1.dot(triB[0]), axisA1.dot(triB[1]), axisA1.dot(triB[2]) });
	//double minB1 = std::min({ axisA1.dot(triB[0]), axisA1.dot(triB[1]), axisA1.dot(triB[2]) });
	//if (maxA1 - minB1 < _eps || maxB1 - minA1 < _eps)//(maxA0 < minB0 || maxB0 < minA0)
	//	return false;
	for (const auto& axis : axes)
	{
		double maxA = std::max(std::max(axis.dot(triA[0]), axisA1.dot(triA[1])), axisA1.dot(triA[2]));
		double minA = std::min(std::max(axis.dot(triA[0]), axisA1.dot(triA[1])), axisA1.dot(triA[2]));
		double maxB = std::max(std::max(axis.dot(triB[0]), axisA1.dot(triB[1])), axisA1.dot(triB[2]));
		double minB = std::min(std::max(axis.dot(triB[0]), axisA1.dot(triB[1])), axisA1.dot(triB[2]));
		if (maxA - minB < _eps || maxB - minA < _eps)//(maxA0 < minB0 || maxB0 < minA0)
			return false;
	}
	return true;
}

bool test_axis(const Vector3d& axis, const const std::array<Eigen::Vector3d, 3>& tri, const Eigen::AlignedBox3d& aabb)
{
	double p0 = axis.x() * tri[0].x() + axis.y() * tri[0].y() + axis.z() * tri[0].z();
	double p1 = axis.x() * tri[1].x() + axis.y() * tri[1].y() + axis.z() * tri[1].z();
	double p2 = axis.x() * tri[2].x() + axis.y() * tri[2].y() + axis.z() * tri[2].z();
	double t_min = std::min({ p0, p1, p2 });
	double t_max = std::max({ p0, p1, p2 });
	double aabb_min = axis.x() * aabb.min().x() + axis.y() * aabb.min().y() + axis.z() * aabb.min().z();
	double aabb_max = axis.x() * aabb.max().x() + axis.y() * aabb.max().y() + axis.z() * aabb.max().z();

	return !(t_max < aabb_min || t_min > aabb_max);
}


//wrote by chatgpt4
bool intersect(const const std::array<Eigen::Vector3d, 3>& tri, const Eigen::AlignedBox3d& aabb)
{
	// Triangle edges
	Vector3d e0 = tri[1] - tri[0];
	Vector3d e1 = tri[2] - tri[1];
	Vector3d e2 = tri[0] - tri[2];
	// AABB axes
	Vector3d aabb_axis_x(1, 0, 0);
	Vector3d aabb_axis_y(0, 1, 0);
	Vector3d aabb_axis_z(0, 0, 1);

	// Test triangle edges against AABB axes
	if (!test_axis(Vector3d(e0.y() * aabb_axis_x.z() - e0.z() * aabb_axis_x.y(), e0.z() * aabb_axis_x.x() - e0.x() * aabb_axis_x.z(), e0.x() * aabb_axis_x.y() - e0.y() * aabb_axis_x.x()), tri, aabb)) return false;
	if (!test_axis(Vector3d(e0.y() * aabb_axis_y.z() - e0.z() * aabb_axis_y.y(), e0.z() * aabb_axis_y.x() - e0.x() * aabb_axis_y.z(), e0.x() * aabb_axis_y.y() - e0.y() * aabb_axis_y.x()), tri, aabb)) return false;
	if (!test_axis(Vector3d(e0.y() * aabb_axis_z.z() - e0.z() * aabb_axis_z.y(), e0.z() * aabb_axis_z.x() - e0.x() * aabb_axis_z.z(), e0.x() * aabb_axis_z.y() - e0.y() * aabb_axis_z.x()), tri, aabb)) return false;
	if (!test_axis(Vector3d(e1.y() * aabb_axis_x.z() - e1.z() * aabb_axis_x.y(), e1.z() * aabb_axis_x.x() - e1.x() * aabb_axis_x.z(), e1.x() * aabb_axis_x.y() - e1.y() * aabb_axis_x.x()), tri, aabb)) return false;
	if (!test_axis(Vector3d(e1.y() * aabb_axis_y.z() - e1.z() * aabb_axis_y.y(), e1.z() * aabb_axis_y.x() - e1.x() * aabb_axis_y.z(), e1.x() * aabb_axis_y.y() - e1.y() * aabb_axis_y.x()), tri, aabb)) return false;
	if (!test_axis(Vector3d(e1.y() * aabb_axis_z.z() - e1.z() * aabb_axis_z.y(), e1.z() * aabb_axis_z.x() - e1.x() * aabb_axis_z.z(), e1.x() * aabb_axis_z.y() - e1.y() * aabb_axis_z.x()), tri, aabb)) return false;
	if (!test_axis(Vector3d(e2.y() * aabb_axis_x.z() - e2.z() * aabb_axis_x.y(), e2.z() * aabb_axis_x.x() - e2.x() * aabb_axis_x.z(), e2.x() * aabb_axis_x.y() - e2.y() * aabb_axis_x.x()), tri, aabb)) return false;
	if (!test_axis(Vector3d(e2.y() * aabb_axis_y.z() - e2.z() * aabb_axis_y.y(), e2.z() * aabb_axis_y.x() - e2.x() * aabb_axis_y.z(), e2.x() * aabb_axis_y.y() - e2.y() * aabb_axis_y.x()), tri, aabb)) return false;
	if (!test_axis(Vector3d(e2.y() * aabb_axis_z.z() - e2.z() * aabb_axis_z.y(), e2.z() * aabb_axis_z.x() - e2.x() * aabb_axis_z.z(), e2.x() * aabb_axis_z.y() - e2.y() * aabb_axis_z.x()), tri, aabb)) return false;

	// Test triangle plane
	Vector3d normal = Vector3d(
		e0.y() * e1.z() - e0.z() * e1.y(),
		e0.z() * e1.x() - e0.x() * e1.z(),
		e0.x() * e1.y() - e0.y() * e1.x()
	);

	if (!test_axis(normal, tri, aabb))
		return false;
	return true;
}

//static int main0() {
//	Triangle tri = { Vector3d(0, 0, 0), Vector3d(1, 0, 0), Vector3d(0, 1, 0) };
//	AlignedBox3d aabb(Vector3d(-1, -1, -1), Vector3d(1, 1, 1));
//
//	if (intersect(tri, aabb)) {
//		std::cout << "Triangle and AABB intersect" << std::endl;
//	}
//	else {
//		std::cout << "Triangle and AABB do not intersect" << std::endl;
//	}
//
//	return 0;
//}

bool TriangleAABB(Triangle triangle, const Vector3d& center, const Vector3d& extents)
{
	// Get the triangle points as vectors
	Vector3d v0 = triangle[0] - center;
	Vector3d v1 = triangle[1] - center;
	Vector3d v2 = triangle[2] - center;

	// Compute the edge vectors of the triangle  (ABC)
	// That is, get the lines between the points as vectors
	Vector3d f0 = v1 - v0; // B - A
	Vector3d f1 = v2 - v1; // C - B
	Vector3d f2 = v0 - v2; // A - C

	// Compute the face normals of the AABB, because the AABB
	// is at center, and of course axis aligned, we know that 
	// it's normals are the X, Y and Z axis.
	Vector3d u0(1.0f, 0.0f, 0.0f);
	Vector3d u1(0.0f, 1.0f, 0.0f);
	Vector3d u2(0.0f, 0.0f, 1.0f);


	// There are a total of 13 axis to test!

	// We first test against 9 axis, these axis are given by
	// cross product combinations of the edges of the triangle
	// and the edges of the AABB. You need to get an axis testing
	// each of the 3 sides of the AABB against each of the 3 sides
	// of the triangle. The result is 9 axis of seperation
	// https://awwapp.com/b/umzoc8tiv/

	// Compute the 9 axis
	Vector3d axis[13];
	axis[0]=u0.cross(f0);	axis[0].normalize();
	axis[1]=u0.cross(f1);	axis[1].normalize();
	axis[2]=u0.cross(f2);	axis[2].normalize();
	axis[3]=u1.cross(f0);	axis[3].normalize();
	axis[4]=u1.cross(f1);	axis[4].normalize();
	axis[5]=u1.cross(f2);	axis[5].normalize();
	axis[6]=u2.cross(f0);	axis[6].normalize();
	axis[7]=u2.cross(f1);	axis[7].normalize();
	axis[8]=u2.cross(f2);	axis[8].normalize();

	// Testing axis: axis_u0_f0
	// Project all 3 vertices of the triangle onto the Seperating axis
	axis[9] = Vector3d(1, 0, 0);
	axis[10] = Vector3d(0, 1, 0);
	axis[11] = Vector3d(0, 0, 1);

	// Finally, we have one last axis to test, the face normal of the triangle
	// We can get the normal of the triangle by crossing the first two line segments
	axis[12]= f0.cross(f1); axis[12].normalize();

	for (int i = 0; i < 13; i++)
	{
		// Testing axis: axis_u0_f0
		// Project all 3 vertices of the triangle onto the Seperating axis
		double p0 = v0.dot(axis[i]);
		double p1 = v1.dot(axis[i]);
		double p2 = v2.dot(axis[i]);
		double r = extents.x() * fabs(u0.dot(axis[i])) +
					extents.y() * fabs(u1.dot(axis[i])) +
					extents.z() * fabs(u2.dot(axis[i]));
		// Now do the actual test, basically see if either of
		// the most extreme of the triangle points intersects r
		// You might need to write Min & Max functions that take 3 arguments
		if (std::max(-max(max(p0, p1), p2), min(min(p0, p1), p2)) > r) 
		{
			// This means BOTH of the points of the projected triangle
			// are outside the projected half-length of the AABB
			// Therefore the axis is seperating and we can exit
			return false;
		}
	}
	return true;
}

