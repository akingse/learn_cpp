#include "pch.h"
#include <iomanip>
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

	//double d = 1;
	//Vector3d point0(1.1, 0, 0);
	//Vector3d point1 = psykronix::translate(2.2, 0) * psykronix::roty(M_PI) *= point0;
	//Vector3d point2 = psykronix::rotz(2*M_PI) *= point0;

	//if (point0 == point1) //二进制判相等，无视精度
	//	d = 0;
	//if (point0 == point2)
	//	d = 0;
	//size_t n = sizeof(Triangle); //72
	return memcmp(&triA, &triB, sizeof(Triangle)) == 0;// triA[0] == triB[0] && triA[1] == triB[1] && triA[2] == triB[2];

	//return // include exchange
	//	(triA[0] == triB[0] && triA[1] == triB[1] && triA[2] == triB[2]) ||
	//	(triA[0] == triB[0] && triA[1] == triB[2] && triA[2] == triB[1]) ||
	//	(triA[0] == triB[1] && triA[1] == triB[0] && triA[2] == triB[2]) || 
	//	(triA[0] == triB[1] && triA[1] == triB[2] && triA[2] == triB[0]) ||
	//	(triA[0] == triB[2] && triA[1] == triB[0] && triA[2] == triB[1]) ||
	//	(triA[0] == triB[2] && triA[1] == triB[1] && triA[2] == triB[0]) ;

	//area
	//return getTrigonArea2(triA) == getTrigonArea2(triB);
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

	cout << "return 0" << endl;
}

static void _test1()
{

	double d = std::nan("0");
	Vector3d triA_0 = Vector3d(4924494.8122771187, -385870.18283433426, 5749.9999999999054);
	Vector3d triA_1 = Vector3d(4924599.8122771177, -385945.18283433421, 5749.9999999999054);
	Vector3d triA_2 = Vector3d(4924586.8713248633, -385946.88654301979, 5749.9999999999054);
	Vector3d triB_0 = Vector3d(4924601.8102601077, -385940.89359764993, 5750.0000000000000);
	Vector3d triB_1 = Vector3d(4924595.2577039087, -385951.32193110074, 5750.0000000000000);
	Vector3d triB_2 = Vector3d(4924589.8109916430, -385975.18553675216, 5750.0000000000000);
	bool isTI = isTwoTrianglesIntersection({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	//bool isTIT = TriangularIntersectionTest({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });

	triA_0 = Vector3d(4948618.6464014640, -378059.39893364342, 39.982199933911403); //x
	triA_1 = Vector3d(4948618.6464014640, -378066.89893364336, 39.982199933911403);	//x-
	triA_2 = Vector3d(4948608.3857971635, -378068.70815501985, 39.982184091814524);
	triB_2 = Vector3d(4948618.6464014640, -378066.89893364359, 39.982199933801859); //x-
	triB_0 = Vector3d(4948648.6464014640, -378096.89893364365, 39.982246252978271);
	triB_1 = Vector3d(4948627.9169112947, -378068.36723815475, 39.982214247214543);
	d = getTrianglesDistance(P, Q, { triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	bool int1 = isTwoTrianglesIntersectSAT({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });

	//triA_0 = Vector3d(4948618.646401, -378059.398934, 39.982200);
	//triA_1 = Vector3d(4948618.646401, -378066.898934, 39.982200);
	//triA_2 = Vector3d(4948608.385797, -378068.708155, 39.982184);
	//triB_0 = Vector3d(4948648.646401, -378096.898934, 39.982246);
	//triB_1 = Vector3d(4948618.646401, -378066.898934, 39.982200);
	//triB_2 = Vector3d(4948609.375892, -378068.367238, 39.982186);
	//d = getTrianglesDistance(P, Q, { triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	//bool int2 = isTwoTrianglesIntersectSAT({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });

	triA_0 = Vector3d(4948589.10216887, -378091.689488313, 39.9821543184115);
	triA_1 = Vector3d(4948589.10216887, -378102.108378973, 39.9821543184115);
	triA_2 = Vector3d(4948581.52809739, -378091.562127208, 39.9821426242490);
	triB_0 = Vector3d(4948648.64640146, -378096.898933644, 39.9822462529783);
	triB_1 = Vector3d(4948590.11470598, -378087.628423812, 39.9821558816473);
	triB_2 = Vector3d(4948588.64640146, -378096.898933644, 39.9821536146255);

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

	triA_0 = Vector3d(4934991.08492488, -380736.849323458, -266.330042529162);
	triA_1 = Vector3d(4934984.36869635, -380736.849323732, -263.095677331456);
	triA_2 = Vector3d(4934986.01043158, -380736.849323665, -271.249229247876);
	triB_0 = Vector3d(4934988.30653218, -380736.849323571, -265.705952052692);
	triB_1 = Vector3d(4934982.79133525, -380736.849323797, -262.020815280171);
	triB_2 = Vector3d(4935011.81215053, -380736.849322611, -250.000000000000);

	//bool res1 = isTwoTrianglesIntersectSAT({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	//bool res2 = isTwoTrianglesIntersection({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	//d = getTrianglesDistance(P, Q, { triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });

	//RW test
	std::vector<std::array<uint64_t, 2>> entityIdList = { {1,2},{3,4} };
	_wirteTrigonFile(entityIdList, "entityIdList.bin");
	std::vector<std::array<uint64_t, 2>> entity = _readEntityIDFile("entityIdList.bin");

	bool res3 = isEqualTrigon(triA, triA);
	bool res4 = isEqualTrigon(triA, triB);

	triA_0 = Vector3d(4935003.6138694724, -380736.84932294575, -221.24922924757556); // ==
	triA_2 = Vector3d(4935003.6138694724, -380736.84932294575, -221.24922924757556); // ==
	triA_1 = Vector3d(4934991.0849248841, -380736.84932345786, -216.33004252886147);
	triB_0 = Vector3d(4934988.3065321781, -380736.84932357143, -215.70595205269194);
	triB_1 = Vector3d(4934982.7913352484, -380736.84932379687, -212.02081528017138);
	triB_2 = Vector3d(4935011.8121505287, -380736.84932261088, -200.00000000000006);

	bool err1 = isTwoTrianglesIntersectSAT({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	bool err2 = isTwoTrianglesBoundingBoxIntersect({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });

	cout << "return 0" << endl;
}

static void _test2()
{
	string path = "C:/Users/Aking/source/repos/bimbase/src/P3d2Stl/bin_file/";
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> tris1 = _readTrigonFile(path + "triInterList_opt1.bin"); //4031
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> tris2 = _readTrigonFile(path + "triInterList_opt2.bin"); //4031
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> tris3 = _readTrigonFile(path + "triInterList_opt3.bin"); //4030
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> tris4 = _readTrigonFile(path + "triInterList_opt.bin"); //4030
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> tris5 = _readTrigonFile(path + "interTriInfo_4027.bin"); //4027

	//entity
	std::vector<std::array<uint64_t, 2>> entity1 = _readEntityIDFile(path + "entityIdList_off.bin"); //4031
	std::vector<std::array<uint64_t, 2>> entity2 = _readEntityIDFile(path + "entityIdList_on.bin"); //4031
	std::vector<std::array<uint64_t, 2>> entity3 = _readEntityIDFile(path + "entityIdList_soft_off.bin"); //1069
	std::vector<std::array<uint64_t, 2>> entity4 = _readEntityIDFile(path + "entityIdList_hard.bin"); //4030
	//(6405-5779)(6510-6123)(6509-6134)
	//std::vector<std::array<uint64_t, 2>> entity5 = _readEntityIDFile(path + "entityIdList_soft_opt_off.bin");
	//std::vector<std::array<uint64_t, 2>> entity6 = _readEntityIDFile(path + "entityIdList_soft_off.bin");
	//(1340-1339)(1347-1345)(1347-1346)

	size_t count = 0;
	//for (auto& iter : tris)
	//{
	//	isInt = isTwoTrianglesIntersectSAT(iter[0], iter[1]);
	//	d = getTrianglesDistance(P,Q,iter[0], iter[1]);
	//	cout << d << endl;
	//}
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> tris1_extra;
	vector<size_t> numList;
	//for (int i=0;i<tris1.size();++i) //entity的不同三角形
	//{
	//	if (!isTwoTrianglesIntersectSAT(tris1[i][0], tris1[i][1]))
	//		cout << "not intersect1" << endl;
	//	if (!isTwoTrianglesIntersectSAT(tris2[i][0], tris2[i][1]))
	//		cout << "not intersect2" << endl;
	//	if (!(isEqualTrigon(tris1[i][0], tris2[i][0]) && isEqualTrigon(tris1[i][1], tris2[i][1])) &&
	//		!(isEqualTrigon(tris1[i][0], tris2[i][1]) && isEqualTrigon(tris1[i][1], tris2[i][0])))
	//		numList.push_back(i);
	//}
	//cout << "diff-index:" << numList.size() << endl;
	//cout << "intersect finish" << endl;

#define DOUBLE_FOR_LOOP
#ifdef DOUBLE_FOR_LOOP
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> extra;
	for (auto& iterA : tris1) //more
	{
		bool findFlag = false;
		for (auto& iterB : tris5) //findTri
		{
			if ((isEqualTrigon(iterA[0], iterB[0]) && isEqualTrigon(iterA[1], iterB[1])) ||
				(isEqualTrigon(iterA[0], iterB[1]) && isEqualTrigon(iterA[1], iterB[0])))
			{
				findFlag = true;
				break;
			}
		}
		if (!findFlag)
			extra.push_back(iterA);
	}
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

	for (int i = 0; i < entity1.size(); ++i)
	{
		bool findFlag = false;
		for (int j = 0; j < entity4.size(); ++j) //big vector, findTri
		{
			if (entity1[i][0] == entity4[j][0] && entity1[i][1] == entity4[j][1])
			{
				findFlag = true;
				//cout << "index=" << j << endl; //823,950,954
				break;
			}
		}
		if (!findFlag)
		{
			count++;
			cout << "no findFlag" << endl;
			cout << entity1[i][0] << "-" << entity1[i][1] << endl; // 6509 - 6134
		}
	}
#endif

	cout << count << endl; //420
	cout << "return 0" << endl;
}

static void _test3()
{

	std::vector<double> diatanceLen;
	//std::set<double> diatanceList;
	//std::multiset<double> diatanceList;
	//std::map<double,vector<std::array<uint64_t, 2>>> diatanceList;
	std::map<double, set<std::array<uint64_t, 2>>> diatanceList;


	int count = 0;
	for (auto& iter : diatanceList)
	{
		//count += iter.second.size(); //1069
		if (iter.first > 1e-9)
		{
			count++;
		}
	}
	//check
	// 	//void _wirteTrigonFile(const std::string & fileName, const std::vector<std::tuple<
	//	std::array<Eigen::Vector3d, 3>, std::array<Eigen::Vector3d, 3>, unsigned long long, unsigned long long, double>>& triinfos)
	string path = "C:/Users/Aking/source/repos/bimbase/src/P3d2Stl/bin_file/";
	std::vector<std::array<uint64_t, 2>> entity1 = _readEntityIDFile(path + "entityIdList_hard.bin"); //4030
	std::vector<std::array<uint64_t, 2>> entity2 = _readEntityIDFile(path + "entityIdList_soft.bin"); //1069
	std::vector<std::array<uint64_t, 2>> entity3 = _readEntityIDFile(path + "entityIdList_4027.bin"); //4027
	for (int i = 0; i < entity1.size(); ++i)
	{
		bool findFlag = false;
		for (int j = 0; j < entity1.size(); ++j) //big vector, findTri
		{
			if (i == j)
				continue;
			if (entity1[i][0] == entity1[j][0] && entity1[i][1] == entity1[j][1])
			{
				findFlag = true;
				//cout << "index=" << j << endl; //823,950,954
				break;
			}
		}
		if (findFlag)
		{
			cout << "findFlag repeat" << endl; //no repeat
			cout << entity1[i][0] << "-" << entity1[i][1] << endl; // 6509 - 6134
		}
	}
	cout << "return 0" << endl;
}

static void _test4()
{
	//射线法，测试点是否在mesh内部
	Vector3d p(0, 0, 0);
	//Triangle tri = { Vector3d(-1, -1, 0),Vector3d(1, 0, M_PI/3),Vector3d(0, 1, 1+ M_PI / 3) };
	Triangle tri = { Vector3d(-1, -1, 0),Vector3d(0, 1, 1 + M_PI / 3),Vector3d(1, 0, M_PI / 3) };
	isPointRayAcrossTriangleSAT(p, tri);

	cout << "return 0" << endl;
}

//test serialization 


static void _test5()
{
#ifdef USING_FLATBUFFERS_SERIALIZATION
	//读取二进制mesh
	//string filename = "C:/Users/Aking/source/repos/bimbase/src/P3d2Stl/bin_file/cvtMeshVct_3.bin";
	string filename = "C:/Users/Aking/source/repos/bimbase/src/P3d2Stl/bin_file/cvtMeshVct_5233.bin";
	std::vector<ModelMesh> meshs = _read_ModelMesh(filename);
	size_t count = 0;
	clock_t startT, endT;
	startT = clock();

	for (auto& iter : meshs)
	{
		//bool b1 = isConvex(iter.vbo_, iter.ibo_);
		//bool b2 = is_convex(iter.vbo_, iter.ibo_);
		//cout << b1 << b2 << endl;
		//bool isConvex = is_convex(iter.vbo_, iter.ibo_); //
		bool isConvex = isMeshConvexPolyhedron(iter.vbo_, iter.ibo_); //
		if (!isConvex)
			count++;
	}
	endT = clock();
	cout << "mesh count=" << meshs.size() << endl;
	cout << "count=" << count << endl;
	cout << "time=" << double(endT - startT) / CLOCKS_PER_SEC << "s" << endl;
#endif // USING_FLATBUFFERS_SERIALIZATION
	cout << "return 0" << endl;
}


static int enrol = []()->int
{
	//_test0();
	//_test1();
	//_test2();
	_test3();
	//_test4();
	//_test5();
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


//------------------------------------------------------------------------------------
// Voxel 
//------------------------------------------------------------------------------------

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

