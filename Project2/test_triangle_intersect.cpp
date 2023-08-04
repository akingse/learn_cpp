#include "pch.h"
#include <iomanip>
using namespace std;
using namespace para;
using namespace Eigen;
using namespace psykronix;
#undef max
#undef min
static constexpr double eps = 1e-6; //DBL_EPSILON
static constexpr double _eps = -eps;

static Eigen::Vector3d P(std::nan("0"), std::nan("0"), std::nan("0"));
static Eigen::Vector3d Q(std::nan("0"), std::nan("0"), std::nan("0"));

double getTrigonArea2(const Triangle& triA)
{
	double a = (triA[1] - triA[0]).squaredNorm();
	double b = (triA[2] - triA[1]).squaredNorm();
	double c = (triA[0] - triA[2]).squaredNorm();
	double p = 0.5 * (a + b + c);
	return p * (p - a) * (p - b) * (p - c);
}

void printTrigon(const Triangle& trigon)
{
	cout << "" << endl;
}

typedef array<int, 3> Face;
bool is_convex(const vector<Vector3d>& points, const vector<array<int, 3>>& faces)
{
	for (const auto& iter : faces)
	{
		Vector3d normal = (points[iter[1]] - points[iter[0]]).cross(points[iter[2]] - points[iter[0]]).normalized();
		bool isFirst = true, isLeft = false, temp = false;
		for (size_t i = 0; i < points.size(); ++i)
		{
			if (i == iter[0] || i == iter[1] || i == iter[2] || fabs(normal.dot(points[i] - points[iter[0]])) < FLT_EPSILON) // self and coplanar
				continue;
			temp = normal.dot(points[i] - points[iter[0]]) < 0.0;
			if (isFirst)
			{
				isLeft = temp;
				isFirst = false;
			}
			else
			{
				if (temp != isLeft)
					return false;
			}
		}
	}
	return true;
}

bool isConvex(const vector<Vector3d>& points, const vector<array<int, 3>>& faces)
{
	//for (int i = 0; i < m; ++i) 
	//{
	//	for (int j = 0; j < n; ++j) 
	//	{
	//		if (j == faces[i][0] || j == faces[i][1] || j == faces[i][2]) //
	//			continue;
	//		if ((points[faces[i][1]] - points[faces[i][0]]).cross(points[faces[i][2]] - points[faces[i][0]]).
	//			dot(points[j] - points[faces[i][0]]) < 0.0)
	//		{
	//			//isIn = true; // 存在一个点在当前面的内部，说明多面体不是凸多面体
	//			//break;
	//			return false;
	//		}
	//	}
	//}
	size_t numFaces = faces.size();
	// 计算每个面的法向量
	vector<Vector3d> normals(numFaces);
	for (size_t i = 0; i < numFaces; ++i)
	{
		Vector3d v0 = points[faces[i][1]] - points[faces[i][0]];
		Vector3d v1 = points[faces[i][2]] - points[faces[i][0]];
		normals[i] = v0.cross(v1).normalized();
	}
	// 计算相邻面的法向量的点积
	for (size_t i = 0; i < numFaces; ++i) {
		for (size_t j = i + 1; j < numFaces; ++j)
		{
			if (normals[i].dot(normals[j]) < 0)
				return false;  // 凹多面体
		}
	}
	return true;  // 凸多面体
}

static bool _isPointInMesh(const Vector3d& point, const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo)
{
	bool isFirst = true, isLeft /*= false*/, temp /*= false*/;
	//for (size_t i = 0; i < vbo.size(); ++i)
	for (const auto& iter : ibo)
	{
		//顺时针绕行法
		//Triangle trigon = { vbo[iter[0]], vbo[iter[1]], vbo[iter[2]] };
		double croPro = (vbo[iter[1]] - vbo[iter[0]]).cross(vbo[iter[2]] - vbo[iter[1]]).dot(point - vbo[iter[0]]);
		if (fabs(croPro) < eps)
			return true; //point on the face
		temp = croPro < 0;
		if (isFirst)
		{
			isLeft = temp;
			isFirst = false;
		}
		else
		{
			if (temp != isLeft)
				return false;
		}
	}
	return true;
}

//测试平行
static void _test1()
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

	//测试旋转对平行的影响
	array<Vector3d, 3> points0 = { Vector3d(100000.1,0,0),  Vector3d(200000.2,0,0),  Vector3d(300000.3,0,0) };
	Vector3d croPro = (points0[1] - points0[0]).cross((points0[1] - points0[0]));
	array<Vector3d, 3> points1 = psykronix::rotz(M_PI / 2) * points0;
	Vector3d croPro1 = (points1[1] - points1[0]).cross((points1[1] - points1[0]));

	points1 = psykronix::rotz(M_PI) * points0;
	croPro1 = (points1[1] - points1[0]).cross((points1[1] - points1[0]));
	points1 = psykronix::rotz(M_PI / 3) * points0;
	croPro1 = (points1[1] - points1[0]).cross((points1[1] - points1[0]));
	points1 = psykronix::rotz(M_PI / 4) * points0;
	croPro1 = (points1[1] - points1[0]).cross((points1[1] - points1[0]));

	//射线法，测试点是否在mesh内部
	Vector3d p(0, 0, 0);
	//Triangle tri = { Vector3d(-1, -1, 0),Vector3d(1, 0, M_PI/3),Vector3d(0, 1, 1+ M_PI / 3) };
	Triangle tri = { Vector3d(-1, -1, 0),Vector3d(0, 1, 1 + M_PI / 3),Vector3d(1, 0, M_PI / 3) };
	isPointRayAcrossTriangleSAT(p, tri);

	cout << "return 0" << endl;
}

//using serialization 
static void _test7()
{
	// 测试SAT计算三角形距离 37s
	//std::array<Vector3d, 3>* randData3 = new std::array<Vector3d, 3>[totalNum];
	//std::array<Vector3d, 3>* randData3_ = new std::array<Vector3d, 3>[totalNum];
	//本地临时文件读写
	size_t countDiff = 0;
	size_t countSepa = 0;
	size_t countInter = 0;
	size_t countPair = 0;
	std::string randNumName = "bin_file/random_1e8.bin";
	std::string randNumNameSepa = "bin_file/random_1e8_sepa.bin";
	// stack size=1M=1048576byte, 1double=8byte, 131072
	const size_t totalNum = (size_t)1e4;
	vector<double> testE(18 * totalNum);
//#define STATE_WRITING
#ifdef STATE_WRITING
	double* arr = new double[18 * totalNum]; //countSepa
	double* readNum = _readNumberFile(totalNum, randNumName);
#else
	double* readNum = _readNumberFile(totalNum, randNumNameSepa);
#endif

#ifdef STATE_WRITING
	for (int i = 0; i < 10*totalNum; ++i)
	{
		Triangle triA = _get_rand3();
		Triangle triB = _get_rand3();
#else
	for (int i = 0; i < totalNum; ++i)
	{
		Triangle triA = { { {readNum[i*18 + 0],readNum[i *18  + 1],readNum[i*18 + 2]} ,
							{readNum[i*18 + 3],readNum[i *18  + 4],readNum[i*18 + 5]} ,
							{readNum[i*18 + 6],readNum[i *18  + 7],readNum[i*18 + 8]} } };
		Triangle triB = { { {readNum[i*18 + 9],readNum[i *18 + 10],readNum[i*18 + 11]} ,
							{readNum[i*18 + 12],readNum[i*18 + 13],readNum[i*18 + 14]} ,
							{readNum[i*18 + 15],readNum[i*18 + 16],readNum[i*18 + 17]} } };
		//for (int j = 0; j < 18; j++)
		//{
		//	testE[i * 18 + j] = readNum[i + 18 + j];
		//}
		//cout<< readNum[i*18 + 0]<<", "<< readNum[i*18 + 1] << ", " << readNum[i*18 + 2] << ", " <<
		//	readNum[i*18 + 3] << ", " << readNum[i*18 + 4] << ", " << readNum[i*18 + 5] << ", " <<
		//	readNum[i*18 + 6] << ", " << readNum[i*18 + 7] << ", " << readNum[i*18 + 8] << ", " << endl;
		//cout << readNum[i + 9] << ", " << readNum[i + 10] << ", " << readNum[i*18 + 11] << ", " <<
		//	readNum[i*18 + 12] << ", " << readNum[i*18 + 13] << ", " << readNum[i*18 + 14] << ", " <<
		//	readNum[i*18 + 15] << ", " << readNum[i*18 + 16] << ", " << readNum[i*18 + 17] << ", " << endl;

#endif		   
		bool isInter = isTwoTrianglesIntersectSAT(triA, triB);
		double d0, d1;
		if (!isInter)
		{
#ifdef STATE_WRITING
			//for (int j = 0; j < 18; j++)
			//{
			//	arr[countSepa * 18 + j] = readNum[i + j];
			//}
			for (int j = 0; j < 3; j++)
			{
				//cout << triA[j][0] << ", ";
				//cout << triA[j][1] << ", ";
				//cout << triA[j][2] << ", ";
				//cout << triB[j][0] << ", ";
				//cout << triB[j][1] << ", ";
				//cout << triB[j][2] << ", " << endl;

				arr[countSepa * 18 + j * 3 + 0] = triA[j][0];
				arr[countSepa * 18 + j * 3 + 1] = triA[j][1];
				arr[countSepa * 18 + j * 3 + 2] = triA[j][2];
				arr[countSepa * 18 + 9 + j * 3 + 0] = triB[j][0];
				arr[countSepa * 18 + 9 + j * 3 + 1] = triB[j][1];
				arr[countSepa * 18 + 9 + j * 3 + 2] = triB[j][2];

				testE[countSepa * 18 + j * 3 + 0] = triA[j][0];
				testE[countSepa * 18 + j * 3 + 1] = triA[j][1];
				testE[countSepa * 18 + j * 3 + 2] = triA[j][2];
				testE[countSepa * 18 + 9 + j * 3 + 0] = triB[j][0];
				testE[countSepa * 18 + 9 + j * 3 + 1] = triB[j][1];
				testE[countSepa * 18 + 9 + j * 3 + 2] = triB[j][2];
			}
			countSepa++;
			if (countSepa == 1e4)
				break;

#endif
			d0 = getTrianglesDistance(P, Q, triA, triB);
			d1 = getTrianglesDistanceSAT(triA, triB);
			//cout << d0-d1 << endl;
			if (fabs(d0 - d1) > eps)
				countDiff++;
		}
		if (isInter)
			countInter++;

	}
	cout << "countInter=" << countInter << endl;
	cout << "countDiff=" << countDiff << endl;

#ifdef STATE_WRITING
	// wirte new
	_wirteNumberFile(totalNum, arr, randNumNameSepa);
#endif
}

static void _test8()
{
	// error triangle
	Vector3d triA_0 = Vector3d(0, 0, 0);
	Vector3d triA_1 = Vector3d(10, 5, 0);
	Vector3d triA_2 = Vector3d(0, 10, 0);
	Vector3d triB_0 = Vector3d(3, 0, 0);
	Vector3d triB_1 = Vector3d(3, 10, 0);
	Vector3d triB_2 = Vector3d(-2, 5, 0);
	Triangle triA = { triA_0, triA_1, triA_2 };
	Triangle triB = { triB_0, triB_1, triB_2 };

	array<Vector3d, 2> pointsNear = getTwoTrianglesNearestPoints(triA, triB);
	array<Vector3d, 2> pointsInter = getTwoTrianglesIntersectPoints(triA, triB);

	std::array<Vector3d, 2> res;
	for (int i = 0; i < 2; ++i)
		res[i] = Vector3d(i, i, 0);

	//
	size_t countDiff1 = 0;
	size_t countDiff2 = 0;
	size_t countInter = 0;
	const size_t totalNum = (size_t)1e4;
	std::string randNumName = "bin_file/random_1e8.bin";
	double* readNum = _readNumberFile(totalNum, randNumName);
	for (int i = 0; i < totalNum; ++i)
	{
		Triangle triA = { { {readNum[i * 18 + 0],readNum[i * 18 + 1],readNum[i * 18 + 2]} ,
							{readNum[i * 18 + 3],readNum[i * 18 + 4],readNum[i * 18 + 5]} ,
							{readNum[i * 18 + 6],readNum[i * 18 + 7],readNum[i * 18 + 8]} } };
		Triangle triB = { { {readNum[i * 18 + 9],readNum[i * 18 + 10],readNum[i * 18 + 11]} ,
							{readNum[i * 18 + 12],readNum[i * 18 + 13],readNum[i * 18 + 14]} ,
							{readNum[i * 18 + 15],readNum[i * 18 + 16],readNum[i * 18 + 17]} } };
		double d0, d1, d2;
		bool isInter = isTwoTrianglesIntersectSAT(triA, triB);
		//if (isInter)
		{
			countInter++;
			//array<Vector3d, 2> pn2 = getTwoTrianglesNearestPoints(triA, triB);
			array<Vector3d, 2> pn2 = getTwoTrianglesIntersectPoints(triA, triB);
			Vector3d p0 = pn2[0];
			Vector3d p1 = pn2[1];
			//cout << p0 << endl << p1 << endl << endl;
			d1 = (pn2[1] - pn2[0]).norm();
		}
		if (!isInter)
		{
			d0 = getTrianglesDistance(P, Q, triA, triB);
			d1 = getTrianglesDistanceSAT(triA, triB);
			array<Vector3d, 2> pn2= getTwoTrianglesNearestPoints(triA, triB);
			d2 = (pn2[1] - pn2[0]).norm();
			//cout << d0-d1 << endl;
			if (fabs(d0 - d1) > eps)
				countDiff1++;
			if (fabs(d0 - d2) > eps)
				countDiff2++;
		}
	}
	cout << "countInter=" << countInter << endl;
	cout << "countDiff1=" << countDiff1 << endl;
	cout << "countDiff2=" << countDiff2 << endl;
}

static int enrol = []()->int
{
	//_test7();
	_test8();
	return 0;
}();



/*
//离轴定理，包围盒与三角形求交
找到与三角形平面平行的两个AABB顶点
测试三角形的三条边与AABB的三条轴的九种组合
测试三角形的顶点是否在AABB内
这个算法速度快且准确。使用分离轴定理，我们只需要测试13个潜在的分离轴是否存在。
如果有任何一个分离轴存在，那么三角形和AABB不相交。如果所有潜在的分离轴都不存在，则三角形和AABB相交。

//GYDevillersTriangle.h

快速检测空间三角形相交算法的代码实现(Devillers & Guigue算法)
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

//------------------------------------------------------------------------------------
// Voxel 
//------------------------------------------------------------------------------------

#undef max
#undef min
#include <map>
#include <set>
#include <array>
#include <vector>
#include <memory>
#include <ranges>
#include <format>
#include <cassert>
#include <numeric>
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <functional>
#include <type_traits>
#include "Eigen/Geometry"    
#include <iostream>

bool TriangularIntersectionTest(const std::array<Eigen::Vector3d, 3>& T1, const std::array<Eigen::Vector3d, 3>& T2)
{
	//#ifdef STATISTIC_DATA_COUNT
	//	isTwoTrianglesInter++;
	//#endif
	const Eigen::Vector3d& A1 = *(const Eigen::Vector3d*)(&T1.at(0));
	const Eigen::Vector3d& B1 = *(const Eigen::Vector3d*)(&T1.at(1));
	const Eigen::Vector3d& C1 = *(const Eigen::Vector3d*)(&T1.at(2));
	const Eigen::Vector3d& A2 = *(const Eigen::Vector3d*)(&T2.at(0));
	const Eigen::Vector3d& B2 = *(const Eigen::Vector3d*)(&T2.at(1));
	const Eigen::Vector3d& C2 = *(const Eigen::Vector3d*)(&T2.at(2));
	// T1
	Eigen::Vector3d A1B1 = B1 - A1;
	Eigen::Vector3d B1C1 = C1 - B1;
	Eigen::Vector3d C1A1 = A1 - C1;
	// A1 T2
	Eigen::Vector3d A1A2 = A2 - A1;
	Eigen::Vector3d A1B2 = B2 - A1;
	Eigen::Vector3d A1C2 = C2 - A1;
	// B1 T2
	Eigen::Vector3d B1A2 = A2 - B1;
	Eigen::Vector3d B1B2 = B2 - B1;
	Eigen::Vector3d B1C2 = C2 - B1;
	// C1 T2
	Eigen::Vector3d C1A2 = A2 - C1;
	Eigen::Vector3d C1B2 = B2 - C1;
	Eigen::Vector3d C1C2 = C2 - C1;

	// T2
	Eigen::Vector3d A2B2 = B2 - A2;
	Eigen::Vector3d B2C2 = C2 - A2;
	Eigen::Vector3d C2A2 = A2 - C2;
	// A2 T1
	Eigen::Vector3d A2A1 = A1 - A2;
	Eigen::Vector3d A2B1 = B1 - A2;
	Eigen::Vector3d A2C1 = C1 - A2;
	// B2 T1
	Eigen::Vector3d B2A1 = A1 - B2;
	Eigen::Vector3d B2B1 = B1 - B2;
	Eigen::Vector3d B2C1 = C1 - B2;
	// C2 T1
	Eigen::Vector3d C2A1 = A1 - C2;
	Eigen::Vector3d C2B1 = B1 - C2;
	Eigen::Vector3d C2C1 = C1 - C2;

	// n1
	Eigen::Vector3d n1 = A1B1.cross(B1C1);
	double n1_t1 = A1A2.dot(n1);
	double n1_t2 = A1B2.dot(n1);
	double n1_t3 = A1C2.dot(n1);

	// n2
	Eigen::Vector3d n2 = A2B2.cross(B2C2);
	double n2_t1 = A2A1.dot(n2);
	double n2_t2 = A2B1.dot(n2);
	double n2_t3 = A2C1.dot(n2);
	unsigned int n1_ts = (n1_t1 > 0.0 ? 18 : (n1_t1 < 0.0 ? 0 : 9)) + (n1_t2 > 0.0 ? 6 : (n1_t2 < 0.0 ? 0 : 3)) + (n1_t3 > 0.0 ? 2 : (n1_t3 < 0.0 ? 0 : 1));
	unsigned int n2_ts = (n2_t1 > 0.0 ? 18 : (n2_t1 < 0.0 ? 0 : 9)) + (n2_t2 > 0.0 ? 6 : (n2_t2 < 0.0 ? 0 : 3)) + (n2_t3 > 0.0 ? 2 : (n2_t3 < 0.0 ? 0 : 1));
	if (n1_ts == 0 || n2_ts == 0 || n1_ts == 26 || n2_ts == 26)
		return false;   // +*/<  72,48,0,18
	// 共面
	if (n1_ts == 13 || n2_ts == 13)
	{
		Eigen::Vector3d A1B1_outboard = A1B1.cross(n1);
		double A2_k1 = A1A2.dot(A1B1_outboard);
		double B2_k1 = A1B2.dot(A1B1_outboard);
		double C2_k1 = A1C2.dot(A1B1_outboard);
		if (A2_k1 > 0 && B2_k1 > 0 && C2_k1 > 0)
			return false; // +*/<  84,72,0,21
		Eigen::Vector3d B1C1_outboard = B1C1.cross(n1);
		double A2_k2 = B1A2.dot(B1C1_outboard);
		double B2_k2 = B1B2.dot(B1C1_outboard);
		double C2_k2 = B1C2.dot(B1C1_outboard);
		if (A2_k2 > 0 && B2_k2 > 0 && C2_k2 > 0)
			return false; // +*/<  96,96,0,24
		Eigen::Vector3d C1A1_outboard = C1A1.cross(n1);
		double A2_k3 = C1A2.dot(C1A1_outboard);
		double B2_k3 = C1B2.dot(C1A1_outboard);
		double C2_k3 = C1C2.dot(C1A1_outboard);
		if (A2_k3 > 0 && B2_k3 > 0 && C2_k3 > 0)
			return false; // +*/<  108,120,0,27 
		Eigen::Vector3d A2B2_outboard = A2B2.cross(n2);
		double A1_k1 = A2A1.dot(A2B2_outboard);
		double B1_k1 = A2B1.dot(A2B2_outboard);
		double C1_k1 = A2C1.dot(A2B2_outboard);
		if (A1_k1 > 0 && B1_k1 > 0 && C1_k1 > 0)
			return false; // +*/<  120,144,0,30
		Eigen::Vector3d B2C2_outboard = B2C2.cross(n2);
		double A1_k2 = B2A1.dot(B2C2_outboard);
		double B1_k2 = B2B1.dot(B2C2_outboard);
		double C1_k2 = B2C1.dot(B2C2_outboard);
		if (A1_k1 > 0 && B1_k1 > 0 && C1_k1 > 0)
			return false; // +*/<  132,168,0,33
		Eigen::Vector3d C2A2_outboard = C2A2.cross(n2);
		double A1_k3 = C2A1.dot(C2A2_outboard);
		double B1_k3 = C2B1.dot(C2A2_outboard);
		double C1_k3 = C2C1.dot(C2A2_outboard);
		if (A1_k1 > 0 && B1_k1 > 0 && C1_k1 > 0)
			return false; // +*/<  144,192,0,36
		return true; // +*/<  144,192,0,18
	}
	Eigen::Vector3d O1, P1, Q1, O2, P2, Q2;
	switch (n1_ts)
	{
	case 9:
	case 17:
	{
		// A2;
		Eigen::Vector3d A1B1_outboard = A1B1.cross(n1);
		Eigen::Vector3d B1C1_outboard = B1C1.cross(n1);
		Eigen::Vector3d C1A1_outboard = C1A1.cross(n1);
		double k1 = A1A2.dot(A1B1_outboard);
		double k2 = B1A2.dot(B1C1_outboard);
		double k3 = C1A2.dot(C1A1_outboard);
		return (k1 <= 0 && k2 <= 0 && k3 <= 0); // +*/<  90,84,0,21
	}
	case 3:
	case 23:
	{
		// B2;
		Eigen::Vector3d A1B1_outboard = A1B1.cross(n1);
		Eigen::Vector3d B1C1_outboard = B1C1.cross(n1);
		Eigen::Vector3d C1A1_outboard = C1A1.cross(n1);
		double k1 = A1B2.dot(A1B1_outboard);
		double k2 = B1B2.dot(B1C1_outboard);
		double k3 = C1B2.dot(C1A1_outboard);
		return (k1 <= 0 && k2 <= 0 && k3 <= 0); // +*/<  90,84,0,21
	}
	case 1:
	case 25:
	{
		// C2;
		Eigen::Vector3d A1B1_outboard = A1B1.cross(n1);
		Eigen::Vector3d B1C1_outboard = B1C1.cross(n1);
		Eigen::Vector3d C1A1_outboard = C1A1.cross(n1);
		double k1 = A1C2.dot(A1B1_outboard);
		double k2 = B1C2.dot(B1C1_outboard);
		double k3 = C1C2.dot(C1A1_outboard);
		return (k1 <= 0 && k2 <= 0 && k3 <= 0); // +*/<  90,84,0,21
	}
	case 12:
	case 14:
	{
		// A2B2;
		Eigen::Vector3d A2B2_outboard = A2B2.cross(n1);
		double kA = A2A1.dot(A2B2_outboard);
		double kB = A2B1.dot(A2B2_outboard);
		double kC = A2C1.dot(A2B2_outboard);
		if ((kA > 0 && kB > 0 && kC > 0) || (kA < 0 && kB < 0 && kC < 0)) // +*/<  84,72,0,24
			return false;
		break;
	}
	case 4:
	case 22:
	{
		// B2C2;
		Eigen::Vector3d B2C2_outboard = B2C2.cross(n1);
		double kA = B2A1.dot(B2C2_outboard);
		double kB = B2B1.dot(B2C2_outboard);
		double kC = B2C1.dot(B2C2_outboard);
		if ((kA > 0 && kB > 0 && kC > 0) || (kA < 0 && kB < 0 && kC < 0)) // +*/<  84,72,0,24
			return false;
		break;
	}
	case 10:
	case 16:
	{
		// C2A2;
		Eigen::Vector3d C2A2_outboard = C2A2.cross(n1);
		double kA = C2A1.dot(C2A2_outboard);
		double kB = C2B1.dot(C2A2_outboard);
		double kC = C2C1.dot(C2A2_outboard);
		if ((kA > 0 && kB > 0 && kC > 0) || (kA < 0 && kB < 0 && kC < 0)) // +*/<  84,72,0,24
			return false;
		break;
	}
	case 7:     // C2;      A2B2;
	case 19:    // C2;      A2B2;
	case 8:     // C2A2;    A2B2;
	case 18:    // C2A2;    A2B2;
		O2 = A2;
		P2 = B2;
		Q2 = C2;
		break;
	case 11:    // A2;      B2C2;
	case 15:    // A2;      B2C2;
	case 6:     // A2B2;    B2C2;
	case 20:    // A2B2;    B2C2;
		O2 = B2;
		P2 = A2;
		Q2 = C2;
		break;
	case 5:     // B2;      C2A2;
	case 21:    // B2;      C2A2;
	case 2:     // B2C2;    C2A2;
	case 24:    // B2C2;    C2A2;
		O2 = C2;
		P2 = B2;
		Q2 = A2;
		break;
	}
	switch (n2_ts)
	{
	case 9:
	case 17:
	{
		// A1;
		Eigen::Vector3d A2B2_outboard = A2B2.cross(n2);
		Eigen::Vector3d B2C2_outboard = B2C2.cross(n2);
		Eigen::Vector3d C2A2_outboard = C2A2.cross(n2);
		double k1 = A2A1.dot(A2B2_outboard);
		double k2 = B2A1.dot(B2C2_outboard);
		double k3 = C2A1.dot(C2A2_outboard);
		return (k1 <= 0 && k2 <= 0 && k3 <= 0); // +*/<  90,84,0,21
	}
	case 3:
	case 23:
	{
		// B1;
		Eigen::Vector3d A2B2_outboard = A2B2.cross(n2);
		Eigen::Vector3d B2C2_outboard = B2C2.cross(n2);
		Eigen::Vector3d C2A2_outboard = C2A2.cross(n2);
		double k1 = A2B1.dot(A2B2_outboard);
		double k2 = B2B1.dot(B2C2_outboard);
		double k3 = C2B1.dot(C2A2_outboard);
		return (k1 <= 0 && k2 <= 0 && k3 <= 0); // +*/<  90,84,0,21
	}
	case 1:
	case 25:
	{
		// C1;
		Eigen::Vector3d A2B2_outboard = A1B2.cross(n2);
		Eigen::Vector3d B2C2_outboard = B1C2.cross(n2);
		Eigen::Vector3d C2A2_outboard = C1A2.cross(n2);
		double k1 = A2C1.dot(A2B2_outboard);
		double k2 = B2C1.dot(B2C2_outboard);
		double k3 = C2C1.dot(C2A2_outboard);
		return (k1 <= 0 && k2 <= 0 && k3 <= 0); // +*/<  90,84,0,21
	}
	case 12:
	case 14:
	{
		// A1B1;
		Eigen::Vector3d A2B2_outboard = A2B2.cross(n1);
		double kA = A2A1.dot(A2B2_outboard);
		double kB = A2B1.dot(A2B2_outboard);
		double kC = A2C1.dot(A2B2_outboard);
		if ((kA > 0 && kB > 0 && kC > 0) || (kA < 0 && kB < 0 && kC < 0)) // +*/<  84,72,0,24
			return false;
		break;
	}
	case 4:
	case 22:
	{
		// B1C1;
		Eigen::Vector3d B2C2_outboard = B2C2.cross(n1);
		double kA = B2A1.dot(B2C2_outboard);
		double kB = B2B1.dot(B2C2_outboard);
		double kC = B2C1.dot(B2C2_outboard);
		if ((kA > 0 && kB > 0 && kC > 0) || (kA < 0 && kB < 0 && kC < 0)) // +*/<  84,72,0,24
			return false;
		break;
	}
	case 10:
	case 16:
	{
		// C1A1;
		Eigen::Vector3d C2A2_outboard = C2A2.cross(n1);
		double kA = C2A1.dot(C2A2_outboard);
		double kB = C2B1.dot(C2A2_outboard);
		double kC = C2C1.dot(C2A2_outboard);
		if ((kA > 0 && kB > 0 && kC > 0) || (kA < 0 && kB < 0 && kC < 0)) // +*/<  84,72,0,24
			return false;
		break;
	}
	case 7:     // C1;      A1B1;
	case 19:    // C1;      A1B1;
	case 8:     // C1A1;    A1B1;
	case 18:    // C1A1;    A1B1;
		O1 = A1;
		P1 = B1;
		Q1 = C1;
		break;
	case 11:    // A1;      B1C1;
	case 15:    // A1;      B1C1;
	case 6:     // A1B1;    B1C1;
	case 20:    // A1B1;    B1C1;
		O1 = B1;
		P1 = A1;
		Q1 = C1;
		break;
	case 5:     // B1;      C1A1;
	case 21:    // B1;      C1A1;
	case 2:     // B1C1;    C1A2;
	case 24:    // B1C1;    C1A2;
		O1 = C1;
		P1 = B1;
		Q1 = A1;
		break;
	}
	Eigen::Vector3d O1O2 = O2 - O1;
	Eigen::Vector3d O1P2 = P2 - O1;
	Eigen::Vector3d O1Q2 = Q2 - O1;
	Eigen::Vector3d O1P1 = P1 - O1;
	Eigen::Vector3d O1Q1 = Q1 - O1;

	Eigen::Vector3d NP = O1P1.cross(O1O2);
	double kpk = NP.dot(O1Q1);
	double kpp = NP.dot(O1P2);
	double kpq = NP.dot(O1Q2);
	if ((kpk > 0 && kpp < 0 && kpq < 0) || (kpk < 0 && kpp > 0 && kpq > 0))
		return false; // +*/<  89,74,0,24

	Eigen::Vector3d NQ = O1Q1.cross(O1O2);
	double kqk = NQ.dot(O1P1);
	double kqp = NQ.dot(O1P2);
	double kqq = NQ.dot(O1Q2);
	if ((kqk > 0 && kqp < 0 && kqq < 0) || (kqk < 0 && kqp > 0 && kqq > 0))
		return false; // +*/<  101,98,0,30
	return true;
}

void getSegmentsPoints(Eigen::Vector3d& VEC, Eigen::Vector3d& X, Eigen::Vector3d& Y,
	const Eigen::Vector3d& P, const Eigen::Vector3d& A, const Eigen::Vector3d& Q, const Eigen::Vector3d& B)
{
	Eigen::Vector3d TMP;
	double A_dot_A, B_dot_B, A_dot_B, A_dot_T, B_dot_T;
	Eigen::Vector3d T = Q - P;
	A_dot_A = A.dot(A);
	B_dot_B = B.dot(B);
	A_dot_B = A.dot(B);
	A_dot_T = A.dot(T);
	B_dot_T = B.dot(T);
	double denom = A_dot_A * B_dot_B - A_dot_B * A_dot_B;
	double t = (A_dot_T * B_dot_B - B_dot_T * A_dot_B) / denom;
	if ((t < 0) || isnan(t))
		t = 0;
	else if (t > 1)
		t = 1;
	double u = (t * A_dot_B - B_dot_T) / B_dot_B;
	if ((u <= 0) || isnan(u))
	{
		Y = Q;
		t = A_dot_T / A_dot_A;
		if ((t <= 0) || isnan(t))
		{
			X = P;
			VEC = Q - P;
		}
		else if (t >= 1)
		{
			X = P + A;
			VEC = Q - X;
		}
		else
		{
			X = P + t * A;
			TMP = T.cross(A);
			VEC = A.cross(TMP);
		}
	}
	else if (u >= 1)
	{
		Y = Q + B;
		t = (A_dot_B + A_dot_T) / A_dot_A;
		if ((t <= 0) || isnan(t))
		{
			X = P;
			VEC = Y - P;
		}
		else if (t >= 1)
		{
			X = P + A;
			VEC = Y - X;
		}
		else
		{
			X = P + t * A;
			T = Y - P;
			TMP = T.cross(A);
			VEC = A.cross(TMP);
		}
	}
	else
	{
		Y = Q + u * B;
		if ((t <= 0) || isnan(t))
		{
			X = P;
			TMP = T.cross(B);
			VEC = B.cross(TMP);
		}
		else if (t >= 1)
		{
			X = P + A;
			T = Q - X;
			TMP = T.cross(B);
			VEC = B.cross(TMP);
		}
		else
		{
			X = P + t * A;
			VEC = A.cross(B);
			if (VEC.dot(T) < 0)
				VEC = -1.0 * VEC;
		}
	}
}

double getTrianglesDistance(Eigen::Vector3d& P, Eigen::Vector3d& Q, const std::array<Eigen::Vector3d, 3>& S, const std::array<Eigen::Vector3d, 3>& T)
{
#ifdef STATISTIC_DATA_COUNT
	getTriDistC++;
#endif
	//#define USING_INNER_PRE_JUDGE
#ifdef USING_INNER_PRE_JUDGE
	if (isTwoTrianglesIntersectSAT(S, T)) // pre-judge intersect
	{
#ifdef STATISTIC_DATA_COUNT
		count_err_inter_dist++;
#endif
		return 0.0;
	}
#endif
	// Compute vectors along the 6 sides
	std::array<Eigen::Vector3d, 3> Sv, Tv;
	Vector3d VEC;
	Sv[0] = S[1] - S[0];
	Sv[1] = S[2] - S[1];
	Sv[2] = S[0] - S[2];
	Tv[0] = T[1] - T[0];
	Tv[1] = T[2] - T[1];
	Tv[2] = T[0] - T[2];
	Vector3d V, Z, minP, minQ;
	bool shown_disjoint = false;
	double mindd = (S[0] - T[0]).squaredNorm() + 1;  // Set first minimum safely high
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			// Find closest points on edges i & j, plus the vector (and distance squared) between these points
			getSegmentsPoints(VEC, P, Q, S[i], Sv[i], T[j], Tv[j]);
			V = Q - P;
			double dd = V.dot(V);
			// Verify this closest point pair only if the distance squared is less than the minimum found thus far.
			if (dd <= mindd)
			{
				minP = P;
				minQ = Q;
				mindd = dd;
				Z = S[(i + 2) % 3] - P;
				double a = Z.dot(VEC);
				Z = T[(j + 2) % 3] - Q;
				double b = Z.dot(VEC);
				if ((a <= 0) && (b >= 0))
					return sqrt(dd);
				double p = V.dot(VEC);
				if (a < 0)
					a = 0;
				if (b > 0)
					b = 0;
				if ((p - a + b) > 0)
					shown_disjoint = true;
			}
		}
	}
	Vector3d Sn = Sv[0].cross(Sv[1]); // Compute normal to S triangle
	double Snl = Sn.dot(Sn);      // Compute square of length of normal
	// If cross product is long enough,
	if (Snl > 1e-15)
	{
		// Get projection lengths of T points
		Vector3d Tp; //double Tp[3]
		V = S[0] - T[0];
		Tp[0] = V.dot(Sn);
		V = S[0] - T[1];
		Tp[1] = V.dot(Sn);
		V = S[0] - T[2];
		Tp[2] = V.dot(Sn);
		int point = -1;
		if ((Tp[0] > 0) && (Tp[1] > 0) && (Tp[2] > 0))
		{
			point = (Tp[0] < Tp[1]) ? 0 : 1;
			if (Tp[2] < Tp[point])
				point = 2;
		}
		else if ((Tp[0] < 0) && (Tp[1] < 0) && (Tp[2] < 0))
		{
			point = (Tp[0] > Tp[1]) ? 0 : 1;
			if (Tp[2] > Tp[point])
				point = 2;
		}
		// If Sn is a separating direction, 
		if (point >= 0)
		{
			shown_disjoint = true;
			// Test whether the point found, when projected onto the 
			// other triangle, lies within the face.
			V = T[point] - S[0];
			Z = Sn.cross(Sv[0]);
			if (V.dot(Z) > 0)
			{
				V = T[point] - S[1];
				Z = Sn.cross(Sv[1]);
				if (V.dot(Z) > 0)
				{
					V = T[point] - S[2];
					Z = Sn.cross(Sv[2]);
					if (V.dot(Z) > 0)
					{
						// T[point] passed the test - it's a closest point for 
						// the T triangle; the other point is on the face of S
						P = T[point] + (Tp[point] / Snl) * Sn;
						Q = T[point];
						return (P - Q).norm();
					}
				}
			}
		}
	}
	Vector3d Tn = Tv[0].cross(Tv[1]);
	double Tnl = Tn.dot(Tn);
	if (Tnl > 1e-15)
	{
		Vector3d Sp; //double Sp[3]
		V = T[0] - S[0];
		Sp[0] = V.dot(Tn);
		V = T[0] - S[1];
		Sp[1] = V.dot(Tn);
		V = T[0] - S[2];
		Sp[2] = V.dot(Tn);
		int point = -1;
		if ((Sp[0] > 0) && (Sp[1] > 0) && (Sp[2] > 0))
		{
			point = (Sp[0] < Sp[1]) ? 0 : 1;
			if (Sp[2] < Sp[point])
				point = 2;
		}
		else if ((Sp[0] < 0) && (Sp[1] < 0) && (Sp[2] < 0))
		{
			point = (Sp[0] > Sp[1]) ? 0 : 1;
			if (Sp[2] > Sp[point])
				point = 2;
		}
		if (point >= 0)
		{
			shown_disjoint = true;
			V = S[point] - T[0];
			Z = Tn.cross(Tv[0]);
			if (V.dot(Z) > 0)
			{
				V = S[point] - T[1];
				Z = Tn.cross(Tv[1]);
				if (V.dot(Z) > 0)
				{
					V = S[point] - T[2];
					Z = Tn.cross(Tv[2]);
					if (V.dot(Z) > 0)
					{
						P = S[point];
						Q = S[point] + (Sp[point] / Tnl) * Tn;
						return (P - Q).norm();
					}
				}
			}
		}
	}
	if (shown_disjoint)
	{
		P = minP;
		Q = minQ;
		return sqrt(mindd);
	}
	else
		return 0;
}

class Grid
{
public:
	Grid(const Eigen::AlignedBox3d& bounding, std::array<unsigned int, 3> num_size) :
		bounding_(bounding),
		num_size_(num_size),
		fill_cell_(num_size.at(0)* num_size.at(1)* num_size.at(2))
	{
	}
	void push(const std::array<Eigen::Vector3d, 3>& triangle, unsigned char mark)
	{
		for (size_t i = 0; i < 3; i++)
			if (std::isnan(triangle.at(i).x()) || std::isnan(triangle.at(i).y()) || std::isnan(triangle.at(i).z()))
				return;
		/***********************************************************************
		* 对三个点沿着三个方向进行排序，沿边扫描时走三条路径: 0->2; 0->1; 1->2;
		*             x_1
		* y_2       __--_
		*       __--     -_
		* y_0 -------______-_ y_1
		*   x_0             x_2
		**********************************************************************/
		std::array<unsigned int, 3> order_x({ 0, 1, 2 });
		std::array<unsigned int, 3> order_y({ 0, 1, 2 });
		std::array<unsigned int, 3> order_z({ 0, 1, 2 });
		std::sort(order_x.begin(), order_x.end(), [&](int i, int j) {return triangle.at(i).x() < triangle.at(j).x(); });
		std::sort(order_y.begin(), order_y.end(), [&](int i, int j) {return triangle.at(i).y() < triangle.at(j).y(); });
		std::sort(order_z.begin(), order_z.end(), [&](int i, int j) {return triangle.at(i).z() < triangle.at(j).z(); });
		// 排除完全不相交的情况
		if (triangle.at(order_x[0]).x() > bounding_.max().x())
			return;
		if (triangle.at(order_x[0]).y() > bounding_.max().y())
			return;
		if (triangle.at(order_x[0]).z() > bounding_.max().z())
			return;
		if (triangle.at(order_x[2]).x() < bounding_.min().x())
			return;
		if (triangle.at(order_x[2]).y() < bounding_.min().y())
			return;
		if (triangle.at(order_x[2]).z() < bounding_.min().z())
			return;
		// 三个方向格子长度
		double cell_size_x = (bounding_.max().x() - bounding_.min().x()) / num_size_.at(0);
		double cell_size_y = (bounding_.max().y() - bounding_.min().y()) / num_size_.at(1);
		double cell_size_z = (bounding_.max().z() - bounding_.min().z()) / num_size_.at(2);
		double cell_size_x_fraction = 1.0 / cell_size_x;
		double cell_size_y_fraction = 1.0 / cell_size_y;
		double cell_size_z_fraction = 1.0 / cell_size_z;

		/**********************************************************************
		* 例如体素5个格子，遍历的隔板范围为1:4；故采取ceil
		*          __--_
		*      __--     -_
		*    -------______-_
		* |   |   |   |   |   |
		* 0   1   2   3   4   5
		**********************************************************************/
		int partition_x_min_index = triangle.at(order_x[0]).x() > bounding_.min().x() ? ceil((triangle.at(order_x[0]).x() - bounding_.min().x()) * cell_size_x_fraction) : 1;
		int partition_y_min_index = triangle.at(order_y[0]).y() > bounding_.min().y() ? ceil((triangle.at(order_y[0]).y() - bounding_.min().y()) * cell_size_y_fraction) : 1;
		int partition_z_min_index = triangle.at(order_z[0]).z() > bounding_.min().z() ? ceil((triangle.at(order_z[0]).z() - bounding_.min().z()) * cell_size_z_fraction) : 1;
		int partition_x_max_index = triangle.at(order_x[2]).x() > bounding_.max().x() ? num_size_.at(0) : ceil((triangle.at(order_x[2]).x() - bounding_.min().x()) * cell_size_x_fraction);
		int partition_y_max_index = triangle.at(order_y[2]).y() > bounding_.max().y() ? num_size_.at(1) : ceil((triangle.at(order_y[2]).y() - bounding_.min().y()) * cell_size_y_fraction);
		int partition_z_max_index = triangle.at(order_z[2]).z() > bounding_.max().z() ? num_size_.at(2) : ceil((triangle.at(order_z[2]).z() - bounding_.min().z()) * cell_size_z_fraction);
		Eigen::Vector3d vec_x_01 = triangle.at(order_x[1]) - triangle.at(order_x[0]);
		Eigen::Vector3d vec_x_12 = triangle.at(order_x[2]) - triangle.at(order_x[1]);
		Eigen::Vector3d vec_x_20 = triangle.at(order_x[0]) - triangle.at(order_x[2]);
		Eigen::Vector3d vec_y_01 = triangle.at(order_y[1]) - triangle.at(order_y[0]);
		Eigen::Vector3d vec_y_12 = triangle.at(order_y[2]) - triangle.at(order_y[1]);
		Eigen::Vector3d vec_y_20 = triangle.at(order_y[0]) - triangle.at(order_y[2]);
		Eigen::Vector3d vec_z_01 = triangle.at(order_z[1]) - triangle.at(order_z[0]);
		Eigen::Vector3d vec_z_12 = triangle.at(order_z[2]) - triangle.at(order_z[1]);
		Eigen::Vector3d vec_z_20 = triangle.at(order_z[0]) - triangle.at(order_z[2]);
		double y_partial_x_01 = vec_x_01.x() == 0.0 ? 0.0 : vec_x_01.y() / vec_x_01.x();
		double z_partial_x_01 = vec_x_01.x() == 0.0 ? 0.0 : vec_x_01.z() / vec_x_01.x();
		double y_partial_x_12 = vec_x_12.x() == 0.0 ? 0.0 : vec_x_12.y() / vec_x_12.x();
		double z_partial_x_12 = vec_x_12.x() == 0.0 ? 0.0 : vec_x_12.z() / vec_x_12.x();
		double y_partial_x_20 = vec_x_20.x() == 0.0 ? 0.0 : vec_x_20.y() / vec_x_20.x();
		double z_partial_x_20 = vec_x_20.x() == 0.0 ? 0.0 : vec_x_20.z() / vec_x_20.x();
		double z_partial_y_01 = vec_y_01.y() == 0.0 ? 0.0 : vec_y_01.z() / vec_y_01.y();
		double x_partial_y_01 = vec_y_01.y() == 0.0 ? 0.0 : vec_y_01.x() / vec_y_01.y();
		double z_partial_y_12 = vec_y_12.y() == 0.0 ? 0.0 : vec_y_12.z() / vec_y_12.y();
		double x_partial_y_12 = vec_y_12.y() == 0.0 ? 0.0 : vec_y_12.x() / vec_y_12.y();
		double z_partial_y_20 = vec_y_20.y() == 0.0 ? 0.0 : vec_y_20.z() / vec_y_20.y();
		double x_partial_y_20 = vec_y_20.y() == 0.0 ? 0.0 : vec_y_20.x() / vec_y_20.y();
		double x_partial_z_01 = vec_z_01.z() == 0.0 ? 0.0 : vec_z_01.x() / vec_z_01.z();
		double y_partial_z_01 = vec_z_01.z() == 0.0 ? 0.0 : vec_z_01.y() / vec_z_01.z();
		double x_partial_z_12 = vec_z_12.z() == 0.0 ? 0.0 : vec_z_12.x() / vec_z_12.z();
		double y_partial_z_12 = vec_z_12.z() == 0.0 ? 0.0 : vec_z_12.y() / vec_z_12.z();
		double x_partial_z_20 = vec_z_20.z() == 0.0 ? 0.0 : vec_z_20.x() / vec_z_20.z();
		double y_partial_z_20 = vec_z_20.z() == 0.0 ? 0.0 : vec_z_20.y() / vec_z_20.z();
		int kbc = num_size_.at(1) * num_size_.at(2);
		// x
		for (int i_x = partition_x_min_index; i_x < partition_x_max_index; i_x++)
		{
			double partition_x = bounding_.min().x() + i_x * cell_size_x; // 隔板x的坐标
			// 隔板会切割三角形，并交边界于两个交点 point_twist=(partition_x, twist_y, twist_z); point_straight=(partition_x, straight_y, straight_z)
			double twist_y, twist_z, straight_y, straight_z;
			double dx_p0 = partition_x - triangle.at(order_x[0]).x();
			straight_y = y_partial_x_20 * dx_p0 + triangle.at(order_x[0]).y();
			straight_z = z_partial_x_20 * dx_p0 + triangle.at(order_x[0]).z();
			if (partition_x < triangle.at(order_x[1]).x())
			{
				twist_y = y_partial_x_01 * dx_p0 + triangle.at(order_x[0]).y();
				twist_z = z_partial_x_01 * dx_p0 + triangle.at(order_x[0]).z();
			}
			else
			{
				double dx_p1 = partition_x - triangle.at(order_x[1]).x();
				twist_y = y_partial_x_12 * dx_p1 + triangle.at(order_x[1]).y();
				twist_z = z_partial_x_12 * dx_p1 + triangle.at(order_x[1]).z();
			}
			double z_partial_y = (straight_z - twist_z) / (straight_y - twist_y);
			double y_partial_z = (straight_y - twist_y) / (straight_z - twist_z);
			// 计算y,z隔板的范围
			double y_min = std::min(twist_y, straight_y);
			double y_max = std::max(twist_y, straight_y);
			double z_min = std::min(twist_z, straight_z);
			double z_max = std::max(twist_z, straight_z);
			int y_min_index = y_min > bounding_.min().y() ? ceil((y_min - bounding_.min().y()) * cell_size_y_fraction) : 1;
			int y_max_index = y_max > bounding_.max().y() ? num_size_.at(1) : y_max == bounding_.min().y() ? 1 : ceil((y_max - bounding_.min().y()) * cell_size_y_fraction);
			int z_min_index = z_min > bounding_.min().z() ? ceil((z_min - bounding_.min().z()) * cell_size_z_fraction) : 1;
			int z_max_index = z_max > bounding_.max().z() ? num_size_.at(2) : z_max == bounding_.min().z() ? 1 : ceil((z_max - bounding_.min().z()) * cell_size_z_fraction);
			// 对于在x方向投影非常小的三角形（尺寸小于格子）并且没有与格子相交
			if (y_min_index == y_max_index && z_min_index == z_max_index)
			{
				int offset_base = i_x * kbc + (y_min_index - 1) * num_size_.at(2) + z_min_index - 1;
				fill_cell_.at(offset_base) |= mark;
				fill_cell_.at(offset_base - kbc) |= mark;
			}
			else
			{
				// point_twist与point_straight的连线会切割y隔板
				for (int i_y = y_min_index; i_y < y_max_index; i_y++)
				{
					// 交点位置为(partition_x,partition_y,partition_z)
					double partition_y = bounding_.min().y() + i_y * cell_size_y;
					double partition_z = z_partial_y * (partition_y - twist_y) + twist_z;
					if (partition_z < bounding_.min().z())
						continue;
					if (partition_z > bounding_.max().z())
						continue;
					// 索引位置(i_x, i_y, [i_z,i_z+1));
					int i_z = floor((partition_z - bounding_.min().z()) * cell_size_z_fraction);
					i_z = i_z == num_size_.at(2) ? i_z - 1 : i_z;
					// 交点位置的几何意义为：位于i_x,i_y的一根柱与三角面相交，则该柱的四周的四个区域均与三角面相交
					int offset_base = i_x * kbc + i_y * num_size_.at(2) + i_z;
					fill_cell_.at(offset_base) |= mark;
					fill_cell_.at(offset_base - num_size_.at(2)) |= mark;
					fill_cell_.at(offset_base - kbc) |= mark;
					fill_cell_.at(offset_base - kbc - num_size_.at(2)) |= mark;
				}
				// point_twist与point_straight的连线会切割z隔板
				for (int i_z = z_min_index; i_z < z_max_index; i_z++)
				{
					double partition_z = bounding_.min().z() + i_z * cell_size_z;
					double partition_y = y_partial_z * (partition_z - twist_z) + twist_y;
					if (partition_y < bounding_.min().y())
						continue;
					if (partition_y > bounding_.max().y())
						continue;
					int i_y = floor((partition_y - bounding_.min().y()) * cell_size_y_fraction);
					i_y = i_y == num_size_.at(1) ? i_y - 1 : i_y;
					int offset_base = i_x * kbc + i_y * num_size_.at(2) + i_z;
					fill_cell_.at(offset_base) |= mark;
					fill_cell_.at(offset_base - 1) |= mark;
					fill_cell_.at(offset_base - kbc) |= mark;
					fill_cell_.at(offset_base - kbc - 1) |= mark;
				}
			}
		}
		// y
		for (int i_y = partition_y_min_index; i_y < partition_y_max_index; i_y++)
		{
			double partition_y = bounding_.min().y() + i_y * cell_size_y;
			double twist_z, twist_x, straight_z, straight_x;
			double dy_p0 = partition_y - triangle.at(order_y[0]).y();
			straight_z = z_partial_y_20 * dy_p0 + triangle.at(order_y[0]).z();
			straight_x = x_partial_y_20 * dy_p0 + triangle.at(order_y[0]).x();
			if (partition_y < triangle.at(order_y[1]).y())
			{
				twist_z = z_partial_y_01 * dy_p0 + triangle.at(order_y[0]).z();
				twist_x = x_partial_y_01 * dy_p0 + triangle.at(order_y[0]).x();
			}
			else
			{
				double dy_p1 = partition_y - triangle.at(order_y[1]).y();
				twist_z = z_partial_y_12 * dy_p1 + triangle.at(order_y[1]).z();
				twist_x = x_partial_y_12 * dy_p1 + triangle.at(order_y[1]).x();
			}
			double x_partial_z = (straight_x - twist_x) / (straight_z - twist_z);
			double z_partial_x = (straight_z - twist_z) / (straight_x - twist_x);
			double z_min = std::min(twist_z, straight_z);
			double z_max = std::max(twist_z, straight_z);
			double x_min = std::min(twist_x, straight_x);
			double x_max = std::max(twist_x, straight_x);
			int z_min_index = z_min > bounding_.min().z() ? ceil((z_min - bounding_.min().z()) * cell_size_z_fraction) : 1;
			int z_max_index = z_max > bounding_.max().z() ? num_size_.at(2) : z_max == bounding_.min().z() ? 1 : ceil((z_max - bounding_.min().z()) * cell_size_z_fraction);
			int x_min_index = x_min > bounding_.min().x() ? ceil((x_min - bounding_.min().x()) * cell_size_x_fraction) : 1;
			int x_max_index = x_max > bounding_.max().x() ? num_size_.at(0) : x_max == bounding_.min().x() ? 1 : ceil((x_max - bounding_.min().x()) * cell_size_x_fraction);
			if (z_min_index == z_max_index && x_min_index == x_max_index)
			{
				int offset_base = (x_min_index - 1) * kbc + i_y * num_size_.at(2) + z_min_index - 1;
				fill_cell_.at(offset_base) |= mark;
				fill_cell_.at(offset_base - num_size_.at(2)) |= mark;
			}
			else
			{
				for (int i_z = z_min_index; i_z < z_max_index; i_z++)
				{
					double partition_z = bounding_.min().z() + i_z * cell_size_z;
					double partition_x = x_partial_z * (partition_z - twist_z) + twist_x;
					if (partition_x < bounding_.min().x())
						continue;
					if (partition_x > bounding_.max().x())
						continue;
					int i_x = floor((partition_x - bounding_.min().x()) * cell_size_x_fraction);
					i_x = i_x == num_size_.at(1) ? i_x - 1 : i_x;
					int offset_base = i_x * kbc + i_y * num_size_.at(2) + i_z;
					fill_cell_.at(offset_base) |= mark;
					fill_cell_.at(offset_base - 1) |= mark;
					fill_cell_.at(offset_base - num_size_.at(2)) |= mark;
					fill_cell_.at(offset_base - num_size_.at(2) - 1) |= mark;
				}
				for (int i_x = x_min_index; i_x < x_max_index; i_x++)
				{
					double partition_x = bounding_.min().x() + i_x * cell_size_x;
					double partition_z = z_partial_x * (partition_x - twist_x) + twist_z;
					if (partition_z < bounding_.min().z())
						continue;
					if (partition_z > bounding_.max().z())
						continue;
					int i_z = floor((partition_z - bounding_.min().z()) * cell_size_z_fraction);
					i_z = i_z == num_size_.at(1) ? i_z - 1 : i_z;
					int offset_base = i_x * kbc + i_y * num_size_.at(2) + i_z;
					fill_cell_.at(offset_base) |= mark;
					fill_cell_.at(offset_base - kbc) |= mark;
					fill_cell_.at(offset_base - num_size_.at(2)) |= mark;
					fill_cell_.at(offset_base - kbc - num_size_.at(2)) |= mark;
				}
			}
		}
		// z
		for (int i_z = partition_z_min_index; i_z < partition_z_max_index; i_z++)
		{
			double partition_z = bounding_.min().z() + i_z * cell_size_z;
			double twist_x, twist_y, straight_x, straight_y;
			double dz_p0 = partition_z - triangle.at(order_z[0]).z();
			straight_x = x_partial_z_20 * dz_p0 + triangle.at(order_z[0]).x();
			straight_y = y_partial_z_20 * dz_p0 + triangle.at(order_z[0]).y();
			if (partition_z < triangle.at(order_z[1]).z())
			{
				twist_x = x_partial_z_01 * dz_p0 + triangle.at(order_z[0]).x();
				twist_y = y_partial_z_01 * dz_p0 + triangle.at(order_z[0]).y();
			}
			else
			{
				double dz_p1 = partition_z - triangle.at(order_z[1]).z();
				twist_x = x_partial_z_12 * dz_p1 + triangle.at(order_z[1]).x();
				twist_y = y_partial_z_12 * dz_p1 + triangle.at(order_z[1]).y();
			}
			double y_partial_x = (straight_y - twist_y) / (straight_x - twist_x);
			double x_partial_y = (straight_x - twist_x) / (straight_y - twist_y);
			double x_min = std::min(twist_x, straight_x);
			double x_max = std::max(twist_x, straight_x);
			double y_min = std::min(twist_y, straight_y);
			double y_max = std::max(twist_y, straight_y);
			int x_min_index = x_min > bounding_.min().x() ? ceil((x_min - bounding_.min().x()) * cell_size_x_fraction) : 1;
			int x_max_index = x_max > bounding_.max().x() ? num_size_.at(0) : x_max == bounding_.min().x() ? 1 : ceil((x_max - bounding_.min().x()) * cell_size_x_fraction);
			int y_min_index = y_min > bounding_.min().y() ? ceil((y_min - bounding_.min().y()) * cell_size_y_fraction) : 1;
			int y_max_index = y_max > bounding_.max().y() ? num_size_.at(1) : y_max == bounding_.min().y() ? 1 : ceil((y_max - bounding_.min().y()) * cell_size_y_fraction);
			if (x_min_index == x_max_index && y_min_index == y_max_index)
			{
				int offset_base = (x_min_index - 1) * kbc + (y_min_index - 1) * num_size_.at(2) + i_z;
				fill_cell_.at(offset_base) |= mark;
				fill_cell_.at(offset_base - 1) |= mark;
			}
			else
			{
				for (int i_x = x_min_index; i_x < x_max_index; i_x++)
				{
					double partition_x = bounding_.min().x() + i_x * cell_size_x;
					double partition_y = y_partial_x * (partition_x - twist_x) + twist_y;
					if (partition_y < bounding_.min().y())
						continue;
					if (partition_y > bounding_.max().y())
						continue;
					int i_y = floor((partition_y - bounding_.min().y()) * cell_size_y_fraction);
					i_y = i_y == num_size_.at(1) ? i_y - 1 : i_y;
					int offset_base = i_x * kbc + i_y * num_size_.at(2) + i_z;
					fill_cell_.at(offset_base) |= mark;
					fill_cell_.at(offset_base - kbc) |= mark;
					fill_cell_.at(offset_base - 1) |= mark;
					fill_cell_.at(offset_base - kbc - 1) |= mark;
				}
				for (int i_y = y_min_index; i_y < y_max_index; i_y++)
				{
					double partition_y = bounding_.min().y() + i_y * cell_size_y;
					double partition_x = x_partial_y * (partition_y - twist_y) + twist_x;
					if (partition_x < bounding_.min().x())
						continue;
					if (partition_x > bounding_.max().x())
						continue;
					int i_x = floor((partition_x - bounding_.min().x()) * cell_size_x_fraction);
					i_x = i_x == num_size_.at(1) ? i_x - 1 : i_x;
					int offset_base = i_x * kbc + i_y * num_size_.at(2) + i_z;
					fill_cell_.at(offset_base) |= mark;
					fill_cell_.at(offset_base - num_size_.at(2)) |= mark;
					fill_cell_.at(offset_base - 1) |= mark;
					fill_cell_.at(offset_base - num_size_.at(2) - 1) |= mark;
				}
			}
		}
	}
	std::vector<std::tuple<Eigen::Vector3d, unsigned char>> list() const
	{
		std::vector<std::tuple<Eigen::Vector3d, unsigned char>> res;
		double cell_size_x = (bounding_.max().x() - bounding_.min().x()) / num_size_.at(0);
		double cell_size_y = (bounding_.max().y() - bounding_.min().y()) / num_size_.at(1);
		double cell_size_z = (bounding_.max().z() - bounding_.min().z()) / num_size_.at(2);
		int kbc = num_size_.at(1) * num_size_.at(2);
		for (int i = 0; i < fill_cell_.size(); i++)
		{
			if (fill_cell_.at(i) == 0)
				continue;
			int ix = i / kbc;
			int t = i % kbc;
			int iy = t / num_size_.at(2);
			int iz = t % num_size_.at(2);
			res.push_back({ {
				  bounding_.min().x() + (ix + 0.5) * cell_size_x,
				  bounding_.min().y() + (iy + 0.5) * cell_size_y,
				  bounding_.min().z() + (iz + 0.5) * cell_size_z},
				  fill_cell_.at(i) });
		}
		return res;
	}
private:
	std::vector<unsigned char> fill_cell_;
	Eigen::AlignedBox3d bounding_;
	std::array<unsigned int, 3> num_size_;
};

static void test()
{
	Grid grid(Eigen::AlignedBox3d(Eigen::Vector3d{ 0, 0, 0 }, Eigen::Vector3d{ 10, 10, 10 }), { 100,100,100 });
	double k = 10.0 / RAND_MAX;
	const int num = (int)1e5; //stack overflow
	std::array<std::array<Eigen::Vector3d, 3>, num> coor;
	for (size_t i = 0; i < coor.size(); i++)
		coor.at(i) = { Eigen::Vector3d{k* rand(), k* rand(), k* rand()}, Eigen::Vector3d{k* rand(), k* rand(), k* rand()}, Eigen::Vector3d{k* rand(), k* rand(), k* rand()} };
	auto start = std::chrono::steady_clock::now(); // 开始计时
	for (size_t i = 0; i < num; i++)
	{
		grid.push(coor.at(i), 0xff);
		if (i % 100 == 0)
			std::cout << "\r" << i / double(num) * 100.0 << "%\t\t\t";
	}
	auto end = std::chrono::steady_clock::now();   // 结束计时
	std::cout << "\r" << 100.0 << "%\t\t\t" << std::endl;
	auto res = grid.list();
	std::cout << res.size() << std::endl;
	std::chrono::duration<double> elapsed = end - start;
	std::cout << elapsed.count() * 1000 << " ms." << std::endl;

}

