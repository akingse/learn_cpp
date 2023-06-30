#include "pch.h"
#include <iomanip>
#include <calculateTriangle.h>
#include <my_class_fun.h>
using namespace std;
using namespace psykronix;
using namespace std;
using namespace para;
using namespace Eigen;
using namespace psykronix;
#undef max
#undef min
static constexpr double eps = 1e-14; //DBL_EPSILON
static constexpr double _eps = -eps;

static Eigen::Vector3d P(std::nan("0"), std::nan("0"), std::nan("0"));
static Eigen::Vector3d Q(std::nan("0"), std::nan("0"), std::nan("0"));


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

double getTrigonArea2(const Triangle& triA)
{
	double a = (triA[1] - triA[0]).squaredNorm();
	double b = (triA[2] - triA[1]).squaredNorm();
	double c = (triA[0] - triA[2]).squaredNorm();
	double p = 0.5 * (a + b + c);
	return p * (p - a) * (p - b) * (p - c);
}
bool isEqualTrigon(const Triangle& triA, const Triangle& triB)
{
	//if (isTwoTrianglesIntersectSAT(triA, triB))
	//{
	//	bool findFlag = false;
	//	for (const auto& iter : triInterList) //findTri
	//	{
	//		if (iter[0][0].x() == triA[0].x() && iter[0][0].y() == triA[0].y() && iter[0][0].z() == triA[0].z() &&
	//			iter[0][1].x() == triA[1].x() && iter[0][1].y() == triA[1].y() && iter[0][1].z() == triA[1].z() &&
	//			iter[0][2].x() == triA[2].x() && iter[0][2].y() == triA[2].y() && iter[0][2].z() == triA[2].z() &&
	//			iter[1][0].x() == triB[0].x() && iter[1][0].y() == triB[0].y() && iter[1][0].z() == triB[0].z() &&
	//			iter[1][1].x() == triB[1].x() && iter[1][1].y() == triB[1].y() && iter[1][1].z() == triB[1].z() &&
	//			iter[1][2].x() == triB[2].x() && iter[1][2].y() == triB[2].y() && iter[1][2].z() == triB[2].z())
	//		{
	//			findFlag = true;
	//			break;
	//		}
	//	}
	//	if (!findFlag) //record the trigons that detect by function, but exclude by pre-box
	//		triInterListExtra.push_back({ triA, triB }); //多余的相交三角形
	//	return true;
	//}

	double d = 1;
	Vector3d point0(1.1, 0, 0);
	Vector3d point1 = psykronix::translate(2.2, 0) * psykronix::roty(M_PI) *= point0;
	Vector3d point2 = psykronix::rotz(2*M_PI) *= point0;

	if (point0 == point1) //二进制判相等，无视精度
		d = 0;
	if (point0 == point2)
		d = 0;
	//size_t n = sizeof(Triangle); //72
	//return memcmp(&triA, &triB, sizeof(Triangle)) == 0;// triA[0] == triB[0] && triA[1] == triB[1] && triA[2] == triB[2];

	return 
		(triA[0] == triB[0] && triA[1] == triB[1] && triA[2] == triB[2]) ||
		(triA[0] == triB[0] && triA[1] == triB[2] && triA[2] == triB[1]) ||
		(triA[0] == triB[1] && triA[1] == triB[0] && triA[2] == triB[2]) || 
		(triA[0] == triB[1] && triA[1] == triB[2] && triA[2] == triB[0]) ||
		(triA[0] == triB[2] && triA[1] == triB[0] && triA[2] == triB[1]) ||
		(triA[0] == triB[2] && triA[1] == triB[1] && triA[2] == triB[0]) ;

	//area
	//return getTrigonArea2(triA) == getTrigonArea2(triB);
}

void printTrigon(const Triangle& trigon)
{
	cout << "" << endl;
}

static void _test0()
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

	double num = 1.123456789012345;

	std::stringstream ss;
	ss << setprecision(15) << num;
	string str = ss.str(); 
	cout << "keep:" << str << endl;
	ss.clear(15);
	ss << setprecision(15) << num;
	cout << "keep:" << str << endl;
	ss.clear(15);
	ss << setprecision(15) << M_E;
	cout << "keep:" << str << endl;

	std::stringstream ss1;
	ss1 << setprecision(15) << M_PI;
	cout << "keep:" << ss1.str() << endl;
	UINT64 max = (UINT64)~((UINT64)0);
	cout << max << endl;

}

static void _test1()
{

	double d=std::nan("0");
	Vector3d triA_0 = Vector3d(4924494.8122771187, -385870.18283433426, 5749.9999999999054);
	Vector3d triA_1 = Vector3d(4924599.8122771177, -385945.18283433421, 5749.9999999999054);
	Vector3d triA_2 = Vector3d(4924586.8713248633, -385946.88654301979, 5749.9999999999054);
	Vector3d triB_0 = Vector3d(4924601.8102601077, -385940.89359764993, 5750.0000000000000);
	Vector3d triB_1 = Vector3d(4924595.2577039087, -385951.32193110074, 5750.0000000000000);
	Vector3d triB_2 = Vector3d(4924589.8109916430, -385975.18553675216, 5750.0000000000000);
	bool isTI = isTwoTrianglesIntersection({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	bool isTIT = TriangularIntersectionTest({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });

	triA_0 = Vector3d(4948618.6464014640, -378059.39893364342, 39.982199933911403); //x
	triA_1 = Vector3d(4948618.6464014640, -378066.89893364336, 39.982199933911403);	//x-
	triA_2 = Vector3d(4948608.3857971635, -378068.70815501985, 39.982184091814524);
	triB_2 = Vector3d(4948618.6464014640, -378066.89893364359, 39.982199933801859); //x-
	triB_0 = Vector3d(4948648.6464014640, -378096.89893364365, 39.982246252978271);
	triB_1 = Vector3d(4948627.9169112947, -378068.36723815475, 39.982214247214543);
	d = getTrianglesDistance(P, Q, { triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	bool int1=isTwoTrianglesIntersectSAT({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });

	//triA_0 = Vector3d(4948618.646401, -378059.398934, 39.982200);
	//triA_1 = Vector3d(4948618.646401, -378066.898934, 39.982200);
	//triA_2 = Vector3d(4948608.385797, -378068.708155, 39.982184);
	//triB_0 = Vector3d(4948648.646401, -378096.898934, 39.982246);
	//triB_1 = Vector3d(4948618.646401, -378066.898934, 39.982200);
	//triB_2 = Vector3d(4948609.375892, -378068.367238, 39.982186);
	//d = getTrianglesDistance(P, Q, { triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	//bool int2 = isTwoTrianglesIntersectSAT({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });

	triA_0 = Vector3d(4948589.10216887,-378091.689488313,39.9821543184115);
	triA_1 = Vector3d(4948589.10216887,-378102.108378973,39.9821543184115);
	triA_2 = Vector3d(4948581.52809739,-378091.562127208,39.9821426242490);
	triB_0 = Vector3d(4948648.64640146,-378096.898933644,39.9822462529783);
	triB_1 = Vector3d(4948590.11470598,-378087.628423812,39.9821558816473);
	triB_2 = Vector3d(4948588.64640146,-378096.898933644,39.9821536146255);

	Triangle triA = { triA_0, triA_1, triA_2 };
	Triangle triB = { triB_0, triB_1, triB_2 };
	_wirteTrigonFile({ {triA, triB} }, "testTrisData.bin");
	auto res = _readTrigonFile("testTrisData.bin");

	double tolerance = 0.001;
	//bool inter = isTwoTrianglesBoundingBoxIntersect(triA, triB);
	//bool interR = isTwoTrianglesBoundingBoxIntersect(res[0][0], res[0][1]);
	//bool inter1 = isTwoTrianglesBoundingBoxIntersect(triA, triB, tolerance);
	bool isInt = isTwoTrianglesIntersection(triA, triB);
	bool isInt0 = isTwoTrianglesIntersectSAT(triA, triB);
	bool isInt1 = isTwoTrianglesIntersectSAT(res[0][0], res[0][1]);

	triA_0 = Vector3d(4934991.08492488,-380736.849323458,-266.330042529162); 
	triA_1 = Vector3d(4934984.36869635,-380736.849323732,-263.095677331456);	
	triA_2 = Vector3d(4934986.01043158,-380736.849323665,-271.249229247876);
	triB_0 = Vector3d(4934988.30653218,-380736.849323571,-265.705952052692);
	triB_1 = Vector3d(4934982.79133525,-380736.849323797,-262.020815280171);
	triB_2 = Vector3d(4935011.81215053,-380736.849322611,-250.000000000000); 

	bool res1 = isTwoTrianglesIntersectSAT({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	bool res2 = isTwoTrianglesIntersection({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	d = getTrianglesDistance(P, Q, { triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });

	//RW test
	std::vector<std::array<uint64_t, 2>> entityIdList = { {1,2},{3,4} };
	_wirteTrigonFile(entityIdList, "entityIdList.bin");
	std::vector<std::array<uint64_t, 2>> entity = _readEntityIDFile("entityIdList.bin");

	bool res3 = isEqualTrigon(triA, triA);
	bool res4 = isEqualTrigon(triA, triB);

	triA_0 = Vector3d(4935003.6138694724, -380736.84932294575, -221.24922924757556); 
	triA_1 = Vector3d(4934991.0849248841, -380736.84932345786, -216.33004252886147);	
	triA_2 = Vector3d(4935003.6138694724, -380736.84932294575, -221.24922924757556);
	triB_0 = Vector3d(4934988.3065321781, -380736.84932357143, -215.70595205269194);
	triB_1 = Vector3d(4934982.7913352484, -380736.84932379687, -212.02081528017138);
	triB_2 = Vector3d(4935011.8121505287, -380736.84932261088, -200.00000000000006); 

	bool err1= isTwoTrianglesIntersectSAT({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	bool err2= isTwoTrianglesBoundingBoxIntersect({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });

	cout << "return 0" << endl;
}

static void _test2()
{
	string path = "C:/Users/Aking/source/repos/bimbase/src/P3d2Stl/";
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> tris1 = _readTrigonFile(path+"triInterList_opt1.bin"); //4031
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> tris2 = _readTrigonFile(path+"triInterList_opt2.bin"); //4031
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> tris3 = _readTrigonFile(path+"triInterList_opt3.bin"); //4030
	//entity
	std::vector<std::array<uint64_t, 2>> entity1 = _readEntityIDFile(path + "entityIdList_off.bin");
	std::vector<std::array<uint64_t, 2>> entity2 = _readEntityIDFile(path + "entityIdList_on.bin");
	std::vector<std::array<uint64_t, 2>> entity3 = _readEntityIDFile(path + "entityIdList_opt.bin");

	bool isInt;
	double d;
	size_t count = 0;
	//for (auto& iter : tris)
	//{
	//	isInt = isTwoTrianglesIntersectSAT(iter[0], iter[1]);
	//	d = getTrianglesDistance(P,Q,iter[0], iter[1]);
	//	cout << d << endl;
	//}
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> tris1_extra;
	vector<size_t> numList;
	for (int i=0;i<tris1.size();++i)
	{
		if (!isTwoTrianglesIntersectSAT(tris1[i][0], tris1[i][1]))
			cout << "not intersect1" << endl;
		if (!isTwoTrianglesIntersectSAT(tris2[i][0], tris2[i][1]))
			cout << "not intersect2" << endl;
		if (!(isEqualTrigon(tris1[i][0], tris2[i][0]) && isEqualTrigon(tris1[i][1], tris2[i][1])) &&
			!(isEqualTrigon(tris1[i][0], tris2[i][1]) && isEqualTrigon(tris1[i][1], tris2[i][0])))
			numList.push_back(i);
	}
	cout << "diff-index:" << numList.size() << endl;
	cout << "intersect finish" << endl;

#ifdef DOUBLE_FOR_LOOP
	for (auto& iterA : tris1)
	{
		bool findFlag = false;
		for (auto& iterB : tris3) //findTri
		{
			if ((isEqualTrigon(iterA[0], iterB[0]) && isEqualTrigon(iterA[1], iterB[1])) ||
				(isEqualTrigon(iterA[0], iterB[1]) && isEqualTrigon(iterA[1], iterB[0])))
			{
				findFlag = true;
				break;
			}
		}
		if (!findFlag)
		{
			count++;
			tris1_extra.push_back(iterA);
			//cout << "no findFlag" << endl;
			//print_triangle(iter1[0], iter1[1]);
		}
	}
#endif

	for (auto& iterA : entity1)
	{
		bool findFlag = false;
		for (auto& iterB : entity3) //findTri
		{
			if (iterA[0] == iterB[0] && iterA[1] == iterB[1])
			{
				findFlag = true;
				break;
			}
		}
		if (!findFlag)
		{
			cout << "no findFlag" << endl;
			cout << iterA[0]<< "-"<< iterA[1] << endl; // 6509 - 6134
			//print_triangle(iter1[0], iter1[1]);
		}
	}

	cout << count << endl; //420

	cout << "return 0" << endl;
}

static int enrol = []()->int
{
	//_test0();
	_test1();
	_test2();
	return 0;
}();


//------------------------------------------------------------------------------------------------------------------------------
// 
//------------------------------------------------------------------------------------------------------------------------------
// 
//离轴定理，包围盒与三角形求交

/*
找到与三角形平面平行的两个AABB顶点
测试三角形的三条边与AABB的三条轴的九种组合
测试三角形的顶点是否在AABB内
这个算法速度快且准确。使用分离轴定理，我们只需要测试13个潜在的分离轴是否存在。
如果有任何一个分离轴存在，那么三角形和AABB不相交。如果所有潜在的分离轴都不存在，则三角形和AABB相交。
*/

