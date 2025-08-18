#include "pch.h"
#include <iomanip>
using namespace std;
using namespace para;
using namespace Eigen;
using namespace eigen;
using namespace clash;
//using namespace sat;

static Eigen::Vector3d P(std::nan("0"), std::nan("0"), std::nan("0"));
static Eigen::Vector3d Q(std::nan("0"), std::nan("0"), std::nan("0"));

static clash::TriMesh createTriMesh_UnitCube()
{
	clash::TriMesh mesh;
	mesh.vbo_ = {
		Eigen::Vector3d(1.0,  1.0,  1.0), // 0
		Eigen::Vector3d(-1.0,  1.0,  1.0), // 1
		Eigen::Vector3d(-1.0, -1.0,  1.0), // 2
		Eigen::Vector3d(1.0, -1.0,  1.0), // 3
		Eigen::Vector3d(1.0,  1.0, -1.0), // 4
		Eigen::Vector3d(-1.0,  1.0, -1.0), // 5
		Eigen::Vector3d(-1.0, -1.0, -1.0), // 6
		Eigen::Vector3d(1.0, -1.0, -1.0)  // 7
	};
	mesh.ibo_ = {
		// 前脸 (z = 1)
		{0, 1, 2}, {0, 2, 3},
		// 后脸 (z = -1)
		{4, 5, 6}, {4, 6, 7},
		// 顶脸 (y = 1)
		{0, 4, 5}, {0, 5, 1},
		// 底脸 (y = -1)
		{2, 6, 7}, {2, 7, 3},
		// 右脸 (x = 1)
		{0, 3, 7}, {0, 7, 4},
		// 左脸 (x = -1)
		{1, 5, 6}, {1, 6, 2}
	};
	mesh.fno_ = {
		// 前脸法线 (0,0,1)
		{0, 0, 1}, {0, 0, 1},
		// 后脸法线 (0,0,-1)
		{0, 0, -1}, {0, 0, -1},
		// 顶脸法线 (0,1,0)
		{0, 1, 0}, {0, 1, 0},
		// 底脸法线 (0,-1,0)
		{0, -1, 0}, {0, -1, 0},
		// 右脸法线 (1,0,0)
		{1, 0, 0}, {1, 0, 0},
		// 左脸法线 (-1,0,0)
		{-1, 0, 0}, {-1, 0, 0}
	};
	mesh.bounding_ = Eigen::AlignedBox3d(Eigen::Vector3d(-1.0, -1.0, -1.0), Eigen::Vector3d(1.0, 1.0, 1.0));
	return mesh;
}

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
		if (fabs(croPro) < epsF)
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
	//clash::Vertex vec(1, 1, 1);
	//clash::Vertex vec2 = vec;

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
	array<Vector3d, 3> points1 = eigen::rotz(M_PI / 2) * points0;
	Vector3d croPro1 = (points1[1] - points1[0]).cross((points1[1] - points1[0]));

	points1 = eigen::rotz(M_PI) * points0;
	croPro1 = (points1[1] - points1[0]).cross((points1[1] - points1[0]));
	points1 = eigen::rotz(M_PI / 3) * points0;
	croPro1 = (points1[1] - points1[0]).cross((points1[1] - points1[0]));
	points1 = eigen::rotz(M_PI / 4) * points0;
	croPro1 = (points1[1] - points1[0]).cross((points1[1] - points1[0]));

	//射线法，测试点是否在mesh内部
	Vector3d p(0, 0, 0);
	//Triangle tri = { Vector3d(-1, -1, 0),Vector3d(1, 0, M_PI/3),Vector3d(0, 1, 1+ M_PI / 3) };
	Triangle tri = { Vector3d(-1, -1, 0),Vector3d(0, 1, 1 + M_PI / 3),Vector3d(1, 0, M_PI / 3) };
	isPointRayAcrossTriangleSAT(p, tri);

	cout << "return 0" << endl;
}

static void _test2()
{
	//测试 isPointInTriangle
	Vector3d triA_0 = Vector3d(10, 10, 0.3);
	Vector3d triA_1 = Vector3d(10, -10, 0);
	Vector3d triA_2 = Vector3d(10, 0, 20.2);
	Vector3d triB_0 = Vector3d(1.8943685, 0.3846357, 0.8737654);
	Vector3d triB_1 = Vector3d(20.983265, 2.9384856, 10.2398456);
	Vector3d triB_2 = Vector3d();
	Triangle plane = { triA_0, triA_1, triA_2 };
	plane = eigen::scale(1e6, 1e6, 1e6) * plane;
	//Triangle triB = { triB_0, triB_1, triB_2 };
	Segment segment = { triB_0 ,triB_1 };
	segment = eigen::scale(1e6, 1e6, 1e6) * segment;

	//calculate
	Vector3d vecSeg = segment[1] - segment[0];
	Vector3d normal = (plane[1] - plane[0]).cross(plane[2] - plane[1]);// normalized();
	double k = (plane[0] - segment[0]).dot(normal) / vecSeg.dot(normal); //k
	Vector3d local = segment[0] + k * vecSeg; // intersect point

	cout << endl;
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
	for (int i = 0; i < 10 * totalNum; ++i)
	{
		Triangle triA = _get_rand3();
		Triangle triB = _get_rand3();
#else
	for (int i = 0; i < totalNum; ++i)
	{
		Triangle triA = { { {readNum[i * 18 + 0],readNum[i * 18 + 1],readNum[i * 18 + 2]} ,
							{readNum[i * 18 + 3],readNum[i * 18 + 4],readNum[i * 18 + 5]} ,
							{readNum[i * 18 + 6],readNum[i * 18 + 7],readNum[i * 18 + 8]} } };
		Triangle triB = { { {readNum[i * 18 + 9],readNum[i * 18 + 10],readNum[i * 18 + 11]} ,
							{readNum[i * 18 + 12],readNum[i * 18 + 13],readNum[i * 18 + 14]} ,
							{readNum[i * 18 + 15],readNum[i * 18 + 16],readNum[i * 18 + 17]} } };
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
			//d0 = getTrianglesDistance(P, Q, triA, triB);
			d1 = getTrianglesDistanceSAT(triA, triB);
			//cout << d0-d1 << endl;
			if (fabs(d0 - d1) > epsF)
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
			//d0 = getTrianglesDistance(P, Q, triA, triB);
			d1 = getTrianglesDistanceSAT(triA, triB);
			array<Vector3d, 2> pn2 = getTwoTrianglesNearestPoints(triA, triB);
			d2 = (pn2[1] - pn2[0]).norm();
			//cout << d0-d1 << endl;
			if (fabs(d0 - d1) > epsF)
				countDiff1++;
			if (fabs(d0 - d2) > epsF)
				countDiff2++;
		}
	}
	cout << "countInter=" << countInter << endl;
	cout << "countDiff1=" << countDiff1 << endl;
	cout << "countDiff2=" << countDiff2 << endl;
}

std::atomic<size_t> count_gRTT = 0;
static void _test9()
{
	Vector3d triA_0 = Vector3d(0, 0, 0);
	Vector3d triA_1 = Vector3d(10, 5, 0);
	Vector3d triA_2 = Vector3d(0, 10, 0);
	//Vector3d triB_0 = Vector3d(5, 0, 0);
	Vector3d triB_0 = Vector3d(5, 0, -2);
	Vector3d triB_1 = Vector3d(5, 10, 0);
	Vector3d triB_2 = Vector3d(5, 5, 10);

	triA_0 = Vector3d(0, 0, 0);
	triA_1 = Vector3d(10, 0, 0);
	triA_2 = Vector3d(10, 10, 0);
	//triB_0 = Vector3d(8, 0, -1);
	//triB_1 = Vector3d(8, 10, -1);
	//triB_2 = Vector3d(8, 0, 10);
	//para
	triB_0 = Vector3d(100, 0, 20);
	triB_1 = Vector3d(110, 0, 20);
	triB_2 = Vector3d(110, 10, 20);
	Vector3d triA_00 = Vector3d(0, 0, 0).normalized();

	Triangle triA = { triA_0, triA_1, triA_2 };
	Triangle triB = { triB_0, triB_1, triB_2 };

	//double d0 = getTrianglesDistance(triA, triB); //判相交的17个分离轴，不一定含有最大的分离轴
	double d1 = getTrianglesDistanceSAT(triA, triB);

	std::array<Vector3d, 3> tri1 = { Vector3d(), Vector3d(), Vector3d()}; //both support
	std::array<Vector3d, 3> tri2 = { { Vector3d(), Vector3d(), Vector3d()} };
	//RelationOfTwoTriangles rela = getRelationOfTwoTrianglesSAT(triA, triB);
	if (RelationOfTwoTriangles::COPLANAR == getRelationOfTwoTrianglesSAT(triA, triB)) //O2优化不掉两次运行
		cout << "COPLANAR" << endl;
	else if (RelationOfTwoTriangles::CONTACT == getRelationOfTwoTrianglesSAT(triA, triB))
		cout << "CONTACT" << endl;
	else
		cout << "INTRUSIVE" << endl;
	cout << "count_gRTT=" << count_gRTT << endl;

	return;
}

//trigon相交测试
static void _test10()
{
	Vector3d triA_0 = Vector3d(0, 0, 0);
	Vector3d triA_1 = Vector3d(10, 0, 0);
	Vector3d triA_2 = Vector3d(0, 10, 0);
	//
	Vector3d triB_0 = Vector3d(10-0.1, 0, 0);
	Vector3d triB_1 = Vector3d(20, 0, 0);
	Vector3d triB_2 = Vector3d(20, 10, 0);
	Triangle triA = { triA_0, triA_1, triA_2 };
	Triangle triB = { triB_0, triB_1, triB_2 };
	bool isInter = false;
	isInter= isTwoTrianglesIntersectSAT(triA, triB);
	isInter= isTwoTrianglesIntrusionSAT(triA, triB);

	Vector2d tri2A_0 = Vector2d(0,1);
	Vector2d tri2A_1 = Vector2d(1,2);
	Vector2d tri2A_2 = Vector2d(0,2);
	Vector2d tri2B_0 = Vector2d(0,1);
	Vector2d tri2B_1 = Vector2d(1,2);
	Vector2d tri2B_2 = Vector2d(0,0);
	Triangle2d tri2A = { tri2A_0, tri2A_1, tri2A_2 };
	Triangle2d tri2B = { tri2B_0, tri2B_1, tri2B_2 };
	isInter = isTwoTrianglesIntrusionSAT(tri2A, tri2B);

	Triangle2d triA2 = { *(Vector2d*)triA[0].data(),  *(Vector2d*)triA[1].data(),  *(Vector2d*)triA[2].data() };
	Triangle2d triB2 = { *(Vector2d*)triB[0].data(),  *(Vector2d*)triB[1].data(),  *(Vector2d*)triB[2].data() };
	isInter = isTwoTrianglesIntrusionSAT(triA2, triB2);
	//std::vector<Eigen::Vector2d> polyA(triA2.data(), 3);

	Vector2d poly_0 = Vector2d(0, 0);
	Vector2d poly_1 = Vector2d(10, 0);
	Vector2d poly_2 = Vector2d(0, 10);
	std::vector<Eigen::Vector2d> polygonA = { poly_0 ,poly_1 ,poly_2 };
	isInter = isTwoPolygonsIntersectSAT(polygonA, polygonA);

	//isTwoTrianglesIntrusionSAT
	{
		Triangle3d triA = {
		Vector3d(0, 0, 0) ,
		Vector3d(10, 0, 0) ,
		Vector3d(0, 10, 0) , };
		Triangle3d triB = {
			Vector3d(1, 1, -1) ,
			//Vector3d(0, 0, 0) ,
			Vector3d(0, -10, 0) ,
			Vector3d(0, 0, 10) , };
		double intrusion = getTrianglesIntrusionSAT(triA, triB); //0.9
		bool isIntr1 = sat::isTwoTrianglesIntrusionSAT(triA, triB, 1);
		bool isIntr2 = sat::isTwoTrianglesIntrusionSAT(triA, triB, -1);
	}
	return;
}

static void _test11()
{
	//Vector3d triA_0 = Vector3d(0, 0, 0);
	//Vector3d triA_1 = Vector3d(10, 0, 0);
	//Vector3d triA_2 = Vector3d(0, 10, 0);
	//Triangle triA = { triA_0, triA_1, triA_2 };
	Vector2d triA_0 = Vector2d(0, 0);
	Vector2d triA_1 = Vector2d(10, 0);
	Vector2d triA_2 = Vector2d(0, 10);
	Triangle2d triA = { triA_0, triA_1, triA_2 };
	double distance = getDistanceOfPointAndTriangle(Vector2d(10, 10), triA);
	//int res=maxSubArray(vector<int>{ -2,1,-3,4,-1,2,1,-5,4 });
	
	return;
}

//测试isTwoMeshsIntersectSAT
static void _test13()
{
	Eigen::AlignedBox3d box = {
	Vector3d(0, 0, 0),
	Vector3d(10, 10, 10), };

	bool isCon = box.contains(Vector3d(0, 0, 0));//include point on box edge

	//随机向量
	for (int i = 0; i < 10; i++)
	{
		Vector3d rayDir = Vector3d(0.0, 0.0, 1.0);
		Affine3d rotation = Affine3d::Identity();
		Vector3d axis = Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff).normalized();
		rotation.rotate(AngleAxisd(1.0, axis));
		rayDir = rotation * rayDir;
	}
	return;
}

//isTriangleAndBoundingBoxIntersectSAT
static void _test14()
{
	//3D
	std::array<Eigen::Vector3d, 3> trigon0 = {
		Vector3d(-10, 0, 0),
		Vector3d(0, 0, 0),
		Vector3d(0, 10, 0),
	};
	std::array<Eigen::Vector3d, 3> trigon1 = {
		Vector3d(0, 0, 0),
		Vector3d(10, 0, 0),
		Vector3d(10, 10, 0),
	};
	bool isinter0 = isTwoTrianglesBoxIntersect(trigon0, trigon1, 0);
	bool isinter1 = isTwoTrianglesBoxIntersect(trigon0, trigon1, 0.1);
	bool isinter2 = isTwoTrianglesBoxIntersect(trigon0, trigon1, -0.1);

	std::array<Eigen::Vector3d, 3> trigon2 = {
		Vector3d(-10, 0, 0),
		Vector3d(10, 0, 0),
		Vector3d(0, 10, 0),
	};
	std::array<Eigen::Vector3d, 3> trigon3 = {
		Vector3d(0, -10, -10),
		Vector3d(0, -10, 10),
		Vector3d(0, 1, 0),
	};

	bool isintert1 = isTwoTrianglesBoxIntersect(trigon2, trigon3, -1.1);


	std::array<Eigen::Vector3d, 3> trigon4 = {
		Vector3d(-5, -5, 0),
		Vector3d(4, -5, 0),
		Vector3d(-5, 5, 0),
	};
	Eigen::AlignedBox3d box = {
		Vector3d(0, 0, 0),
		Vector3d(10, 10, 10), };

	bool isinter = sat::isTriangleAndBoxIntersectSAT(trigon4, box);
	return;
}

static void _test15()
{
	using namespace sat;
	TriMesh meshIn = createTriMesh_UnitCube();
	TriMesh meshOut = eigen::scale(2) * meshIn;
	bool isin1 = isMeshInsideOtherMesh(meshIn, meshOut);
	bool isin2 = isMeshInsideOtherMesh(meshIn, meshIn);
    bool isin3 = isMeshInsideOtherMesh(eigen::translate(1, 1, 1) * meshIn, meshOut);

	TriMesh meshEmpty;
	bool isin4 = meshOut.bounding_.contains(meshEmpty.bounding_);
	bool isin5 = meshOut.bounding_.contains(meshIn.bounding_);

	bool isinter0 = isMeshInsideOtherMesh(meshIn, meshIn);
	bool isinter1 = isTwoMeshsIntersectSAT(meshIn, meshOut, 0);

    bool isConvex1 = isMeshConvexPolyhedron(toModelMesh(meshIn));


	return;
}

//随机三角形，顶点在法向方向的投影
static void _test16()
{
	//Vector3d p(0, 0, 0);
	//Vector3d p(1e-6, 0, 0);
	Vector3d p(-1e-6, 0, 0);
	bool is0 = p.isZero(1e-7);
	bool is1 = p.isZero(1e-6);

	double k = 1.2342;
	for (int i = 0; i < 100; ++i)
	{
		//double random_num = rand() / (RAND_MAX + 1.0); //a+b+c=1
		Triangle3d trigon = { 
			Vector3d(k*rand() ,k*rand() ,k*rand() ),
			Vector3d(k*rand() ,k*rand() ,k*rand()), 
			Vector3d(k*rand() ,k*rand() ,k*rand()), };
		Vector3d normal = (trigon[1] - trigon[0]).cross(trigon[2] - trigon[1]).normalized();

		double delta = 0;
		delta += fabs(normal.dot(trigon[1] - trigon[0]));
		delta += fabs(normal.dot(trigon[2] - trigon[0]));
		continue;
	}
	//结论：整形没有误差；
	//整形+normalized，有较小误差
	//浮点型+不normalized，误差很大
}

bool isRayAndTriangleIntersect(const Vector3d& pnt, const Vector3d& dir, const std::array<Vector3d, 3 >& tri)
{
	return
		(0.0 <= (tri[0] - pnt).cross(dir).dot(dir.cross(tri[1] - pnt)) && 0.0 <= (tri[0] - pnt).cross(dir).dot((tri[0] - pnt).cross(tri[1] - pnt))) ||
		(0.0 <= (tri[1] - pnt).cross(dir).dot(dir.cross(tri[2] - pnt)) && 0.0 <= (tri[1] - pnt).cross(dir).dot((tri[1] - pnt).cross(tri[2] - pnt))) ||
		(0.0 <= (tri[2] - pnt).cross(dir).dot(dir.cross(tri[0] - pnt)) && 0.0 <= (tri[2] - pnt).cross(dir).dot((tri[2] - pnt).cross(tri[0] - pnt)));
}

//射线法
static void _test17()
{
	auto _isRayAndTriangleIntersectParallel = [](const Vector3d& ori, const Vector3d& dir, const std::array<Vector3d, 3 >& trigon)->bool
		{
			// negetive direction ray cause cross product result opposite
            Vector3d cp0 = (trigon[0] - ori).cross(dir);
            Vector3d cp1 = dir.cross(trigon[1] - ori);
            return
				(0.0 <= (trigon[0] - ori).cross(dir).dot(dir.cross(trigon[1] - ori)) && 0.0 <= (trigon[0] - ori).cross(dir).dot((trigon[0] - ori).cross(trigon[1] - ori))) ||
				(0.0 <= (trigon[1] - ori).cross(dir).dot(dir.cross(trigon[2] - ori)) && 0.0 <= (trigon[1] - ori).cross(dir).dot((trigon[1] - ori).cross(trigon[2] - ori))) ||
				(0.0 <= (trigon[2] - ori).cross(dir).dot(dir.cross(trigon[0] - ori)) && 0.0 <= (trigon[2] - ori).cross(dir).dot((trigon[2] - ori).cross(trigon[0] - ori)));
		};

	const Vector3d& direction = Vector3d(-1, 0, 0);
	Vector3d face_center = Vector3d(-10, 5, 0);
	const std::array<Eigen::Vector3d, 3> trigon = {
		Vector3d(10,0, 0),
		Vector3d(10,10,0),
		Vector3d(2,2,0),
	};
	bool inter = _isRayAndTriangleIntersectParallel(face_center, direction, trigon);

	for (int j = 0; j < 2; ++j)
	{
		const std::array<Eigen::Vector3d, 3> trigon = {
			Vector3d(10,0, 0),
			Vector3d(10,10,0),
			Vector3d(2,2,0),
		};
		const Eigen::Vector3d& normal = Vector3d(0, 0, 1);
		double deno = direction.dot(normal); //ray.direction
		if (deno == 0.0)//ray direction is parallel, necessarily using tolarance, avoid overflow
		{
			if ((face_center - trigon[0]).dot(normal) != 0.0) // ray not coplanar
				continue;
			if (!_isRayAndTriangleIntersectParallel(face_center, direction, trigon)) //coplanar
				continue;
			face_center = Vector3d(0, 0, 0);
			//isNewRay = true;
			//countInter = 0;//reset
			//break;
		}
	}

}

static void _test18()
{
	auto _isPointOnTriangleSurface = [](const Vector3d& point, const std::array<Vector3d, 3>& trigon, const Vector3d& normal, double toleDist)->bool
		{
			//_isPointInTriangle
			if (point[0] + toleDist < std::min(std::min(trigon[0][0], trigon[1][0]), trigon[2][0]) ||
				point[0] - toleDist > std::max(std::max(trigon[0][0], trigon[1][0]), trigon[2][0]) ||
				point[1] + toleDist < std::min(std::min(trigon[0][1], trigon[1][1]), trigon[2][1]) ||
				point[1] - toleDist > std::max(std::max(trigon[0][1], trigon[1][1]), trigon[2][1]) ||
				point[2] + toleDist < std::min(std::min(trigon[0][2], trigon[1][2]), trigon[2][2]) ||
				point[2] - toleDist > std::max(std::max(trigon[0][2], trigon[1][2]), trigon[2][2]) ||
				toleDist < fabs(normal.dot(point - trigon[0])))
				return false;
			return
				-toleDist * (trigon[1] - trigon[0]).norm() < (trigon[1] - trigon[0]).cross(point - trigon[0]).dot(normal) && //bool isLeftA
				-toleDist * (trigon[2] - trigon[1]).norm() < (trigon[2] - trigon[1]).cross(point - trigon[1]).dot(normal) && //bool isLeftB
				-toleDist * (trigon[0] - trigon[2]).norm() < (trigon[0] - trigon[2]).cross(point - trigon[2]).dot(normal);   //bool isLeftC
		};
	//Vector3d point(0, 0, 0);
	//Vector3d point(0, 1, 0);
	//Vector3d point(2, 0, 0);
	Vector3d point(0, 0, 1e-7);

	std::array<Vector3d, 3> trigon =
	{
		Vector3d(0,0,0),
		Vector3d(1,0,0),
		Vector3d(0,1,0),
	};
	Vector3d normal(0, 0, 1);
	bool ison = _isPointOnTriangleSurface(point, trigon, normal, 1e-8);
	return;
}

//mesh 薄片问题
static void _test20()
{
	double toleDist = 1e-6;
	double toleAngle = 1e-8;
	//lineTar
	Triangle3d trigonA = {
		Vector3d(8127.927601097925, 1160.0165000554985, 4200),
		Vector3d(8127.927601097925, 1160.0165000554985, 5000),
		Vector3d(25302.92699713257, 1160.1218053095708, 5000),
	};
	Vector3d normalA = (trigonA[1] - trigonA[0]).cross(trigonA[1] - trigonA[2]).normalized();

	//lineTo1
	Triangle3d trigonB = {
		Vector3d(8502.927131848737, 1160.1218053090997, 0),
		Vector3d(7802.927131848737, 1160.1218053090997, 5100),
		Vector3d(8502.927131848737, 1160.1218053090997, 5100),
	};
	Vector3d normalB = (trigonB[1] - trigonB[0]).cross(trigonB[1] - trigonB[2]).normalized();

	Triangle3d trigonC = {
		Vector3d(16902.927131848737, 1160.1218053090997, 0),
		Vector3d(16202.927131848737, 1160.1218053090997, 5100),
		Vector3d(16902.927131848737, 1160.1218053090997, 5100),
	};
	Vector3d normalC = (trigonC[1] - trigonC[0]).cross(trigonC[1] - trigonC[2]).normalized();
	Triangle3d trigonD = {
		Vector3d(25302.927131848737, 1160.1218053090997, 0),
		Vector3d(24602.927131848737, 1160.1218053090997, 5100),
		Vector3d(25302.927131848737, 1160.1218053090997, 5100),
	};

	double dotPrp = fabs(fabs(normalA.dot(normalB) - 1.0));
	bool isinter = sat::isTwoTrianglesIntersectSAT(trigonA, trigonB);
	bool isinter1 = sat::isTwoTrianglesIntrusionSAT(trigonA, trigonB, toleDist);
	bool isinter2 = sat::isTwoTrianglesIntersectSAT(trigonA, trigonC);
	bool isinter3 = sat::isTwoTrianglesIntersectSAT(trigonA, trigonD);


	double distance1 = getTrianglesDistanceSAT(trigonA, trigonB);
	double distance2 = getTrianglesDistanceSAT(trigonA, trigonC);
	//
	double norm1 = getTrianglesDistanceSAT(trigonA, normalA, trigonB, normalB);
	double norm2 = getTrianglesDistanceSAT(trigonA, normalA, trigonC, normalC);
	return;
}

//退化三角形测试
static void _test21()
{
	Triangle3d trigonA = {
	Vector3d(0,0,0),
	Vector3d(10,0,0),
	Vector3d(0,10,0),
	};
	Vector3d normalA = (trigonA[1] - trigonA[0]).cross(trigonA[1] - trigonA[2]).normalized();

	//lineTo1
	Triangle3d trigonB = {
	Vector3d(10,0,10),
	Vector3d(10,0,10),
	Vector3d(0,10,10),
	};
	Vector3d normalB = (trigonB[1] - trigonB[0]).cross(trigonB[1] - trigonB[2]).normalized();

	//double distance1 = getTrianglesDistanceSAT(trigonA, trigonB);
	double distance1 = getTrianglesDistanceSAT(trigonA, eigen::scale(0.5)*trigonB);
	//
	double norm1 = getTrianglesDistanceSAT(trigonA, normalA, eigen::scale(0.5) * trigonB, normalB);
	cout << "distance=" << distance1 << endl;
	cout << "distance=" << norm1 << endl;
	return;
}

//随机三角形，测试distance误差
static void _test22()
{

}

//相交容差
static void _test23()
{
	Triangle3d trigonA = {
	Vector3d(0,0,0),
	Vector3d(10,0,0),
	Vector3d(0,10,0),
	};
	Vector3d normalA = (trigonA[1] - trigonA[0]).cross(trigonA[1] - trigonA[2]).normalized();

	//lineTo1
	Triangle3d trigonB = {
	Vector3d(0,0,0),
	Vector3d(10,0,0),
	Vector3d(0,10,0),
	};
	Vector3d normalB = (trigonB[1] - trigonB[0]).cross(trigonB[1] - trigonB[2]).normalized();

	bool isinter0 = isTwoTrianglesIntrusionSAT(trigonA, eigen::translate(10.1,0,0) * trigonB, 0.1);
	bool isinter1 = isTwoTrianglesIntrusionSAT(trigonA, eigen::translate(10.1,0,0) * trigonB, -0.1);
	return;
}

static Vector3d createRandVector()
{
	Vector3d vec(
		rand() - RAND_MAX / 2 + double(rand()) / RAND_MAX,
		rand() - RAND_MAX / 2 + double(rand()) / RAND_MAX,
		rand() - RAND_MAX / 2 + double(rand()) / RAND_MAX);
	return vec;
}

//旋转对法向的影响 //经过旋转之后，将不再严格平行，但isZero默认参数可以判住
static void _test24()
{
	int paracount = 0;
	for (int i = 0; i < 10; i++)
	{
		Triangle3d trigonA = {
			Vector3d(rand() - RAND_MAX / 2 + double(rand()) / RAND_MAX,rand() - RAND_MAX / 2 + double(rand()) / RAND_MAX,0),
			Vector3d(rand() - RAND_MAX / 2 + double(rand()) / RAND_MAX,rand() - RAND_MAX / 2 + double(rand()) / RAND_MAX,0),
			Vector3d(rand() - RAND_MAX / 2 + double(rand()) / RAND_MAX,rand() - RAND_MAX / 2 + double(rand()) / RAND_MAX,0),
		};
		Triangle3d trigonB = {
			Vector3d(rand() - RAND_MAX / 2 + double(rand()) / RAND_MAX,rand() - RAND_MAX / 2 + double(rand()) / RAND_MAX,0),
			Vector3d(rand() - RAND_MAX / 2 + double(rand()) / RAND_MAX,rand() - RAND_MAX / 2 + double(rand()) / RAND_MAX,0),
			Vector3d(rand() - RAND_MAX / 2 + double(rand()) / RAND_MAX,rand() - RAND_MAX / 2 + double(rand()) / RAND_MAX,0),
		};
		Eigen::Matrix4d mat = eigen::rotate(createRandVector(), double(rand()) / RAND_MAX); //误差在 1e-15~1e-20
		//trigonA = eigen::rotx(M_PI / 2) * trigonA; //误差在 1e-30~1e-50
		trigonA = mat * trigonA;
		Vector3d normalA = (trigonA[1] - trigonA[0]).cross(trigonA[1] - trigonA[2]).normalized();
		//trigonB = eigen::rotx(M_PI / 2) * trigonB;
		trigonB = mat * trigonB;
		Vector3d normalB = (trigonB[1] - trigonB[0]).cross(trigonB[1] - trigonB[2]).normalized();

		Vector3d cropro = normalA.cross(normalB);
		if (cropro.isZero()) //自带默认误差，dummy_precision() { return 1e-12; }
			paracount++;
	}
	return;
}

//对比
static void _test25()
{
	constexpr double epsilon = Eigen::NumTraits<double>::epsilon(); //DBL_EPSILON
	int intercount = 0;
	int intrucount = 0;
	for (int i = 0; i < int(1e4); i++)
	{
		Triangle3d trigonA = { createRandVector(),createRandVector(),createRandVector() };
		Triangle3d trigonB = { createRandVector(),createRandVector(),createRandVector() };
		
		Eigen::Vector3d normalA = (trigonA[1] - trigonA[0]).cross(trigonA[2] - trigonA[1]).normalized();
		Eigen::Vector3d normalB = (trigonB[1] - trigonB[0]).cross(trigonB[2] - trigonB[1]).normalized();

		bool isinter = isTwoTrianglesIntersectSAT(trigonA, trigonB);
		if (isinter)
			intercount++;
		bool isintru = isTwoTrianglesIntrusionSAT(trigonA, trigonB);
		if (isintru)
			intrucount++;
	}
	return;
}

static int enrol = []()->int
    {
        //_test1();
        //_test2();
        //_test7();
        //_test8();
        //_test9();
        //_test10();
        //_test11();
        //_test12();
        //_test13();
        //_test14();
        //_test15();
        //_test16();
        //_test17();
        //_test18();

        //lamina
        //_test20();
        //_test21();
        _test23();
        _test25();

        cout << "test_triangle_intersect finished.\n" << endl;
        return 0;
    }();

