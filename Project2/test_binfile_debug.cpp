#include "pch.h"
#include <iomanip>
using namespace std;
using namespace para;
using namespace Eigen;
using namespace psykronix;

static constexpr unsigned long long ULL_MAX1 = 18446744073709551615; // 
static constexpr unsigned long long ULL_MAX2 = 2 ^ 64 - 1; //
static constexpr unsigned long long ULL_MAX3 = 0xFFFFFFFFFFFFFFFF; //

static Eigen::Vector3d P(std::nan("0"), std::nan("0"), std::nan("0"));
static Eigen::Vector3d Q(std::nan("0"), std::nan("0"), std::nan("0"));
static std::string binFilePath = "C:/Users/Aking/source/repos/bimbase/src/P3d2Stl/bin_file/";
#undef max
#undef min

enum class TwoMeshCon
{
	AOBO = 0,
	AOBU,
	AUBO,
	AUBU,
};

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

bool _opLessInfo(const InterTriInfo& lhs, const InterTriInfo& rhs)
{
	return lhs.distance < rhs.distance;
}

// 三角形相交测试
static void _test1()
{
	double d = std::nan("0");
	double d1;
	Vector3d triA_0 = Vector3d(4924494.8122771187, -385870.18283433426, 5749.9999999999054);
	Vector3d triA_1 = Vector3d(4924599.8122771177, -385945.18283433421, 5749.9999999999054);
	Vector3d triA_2 = Vector3d(4924586.8713248633, -385946.88654301979, 5749.9999999999054);
	Vector3d triB_0 = Vector3d(4924601.8102601077, -385940.89359764993, 5750.0000000000000);
	Vector3d triB_1 = Vector3d(4924595.2577039087, -385951.32193110074, 5750.0000000000000);
	Vector3d triB_2 = Vector3d(4924589.8109916430, -385975.18553675216, 5750.0000000000000);
	bool isTI = isTwoTrianglesIntersectPIT({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	//bool isTIT = TriangularIntersectionTest({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });

	triA_0 = Vector3d(4948618.6464014640, -378059.39893364342, 39.982199933911403); //x
	triA_1 = Vector3d(4948618.6464014640, -378066.89893364336, 39.982199933911403);	//x-
	triA_2 = Vector3d(4948608.3857971635, -378068.70815501985, 39.982184091814524);
	triB_2 = Vector3d(4948618.6464014640, -378066.89893364359, 39.982199933801859); //x-
	triB_0 = Vector3d(4948648.6464014640, -378096.89893364365, 39.982246252978271);
	triB_1 = Vector3d(4948627.9169112947, -378068.36723815475, 39.982214247214543);

	//triA_0 = Vector3d(0,0,0);
	//triA_1 = Vector3d(10,0,0);
	//triA_2 = Vector3d(10,20,30);
	//triB_2 = Vector3d(0, 1, 0);
	//triB_0 = Vector3d(10, 1, 0);
	//triB_1 = Vector3d(10, 21, 30);

	//triA_0 = Vector3d(0,0,0);
	//triA_1 = Vector3d(0,10,0);
	//triA_2 = Vector3d(5,5,6);
	//triB_2 = Vector3d(20, 0, 0);
	//triB_0 = Vector3d(20, 10, 0);
	//triB_1 = Vector3d(0, 5, 20);

	//triA_0 = Vector3d(0, 0, 0);
	//triA_1 = Vector3d(0, 10, 0);
	//triA_2 = Vector3d(-5, 5, 0);
	//triB_0 = Vector3d(5, 5, -5);
	//triB_1 = Vector3d(5, 5, 5);
	//triB_2 = Vector3d(15, 5, 0);

	//psykronix::roty(M_PI / 2) * std::array<Eigen::Vector3d, 3>
	Triangle triA = { triA_0, triA_1, triA_2 };
	Triangle triB = { triB_0, triB_1, triB_2 };
	bool int1 = isTwoTrianglesIntersectSAT(triA, triB);

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

	triA = { triA_0, triA_1, triA_2 };
	triB = { triB_0, triB_1, triB_2 };
	_wirteTrigonFile({ {triA, triB} }, "bin_file/testTrisData.bin");
	auto res = _readTrigonFile("bin_file/testTrisData.bin");

	double tolerance = 0.001;
	//bool inter = isTwoTrianglesBoundingBoxIntersect(triA, triB);
	//bool interR = isTwoTrianglesBoundingBoxIntersect(res[0][0], res[0][1]);
	//bool inter1 = isTwoTrianglesBoundingBoxIntersect(triA, triB, tolerance);
	bool isInt = isTwoTrianglesIntersectPIT(triA, triB);
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
	_wirteTrigonFile(entityIdList, "bin_file/entityIdList.bin");
	std::vector<std::array<uint64_t, 2>> entity = _readEntityIDFile("bin_file/entityIdList.bin");

	//bool res3 = isEqualTrigon(triA, triA);
	//bool res4 = isEqualTrigon(triA, triB);

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
	//string path = "C:/Users/Aking/source/repos/bimbase/src/P3d2Stl/bin_file/";
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> tris1 = _readTrigonFile(binFilePath + "triInterList_opt1.bin"); //4031
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> tris2 = _readTrigonFile(binFilePath + "triInterList_opt2.bin"); //4031
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> tris3 = _readTrigonFile(binFilePath + "triInterList_opt3.bin"); //4030
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> tris4 = _readTrigonFile(binFilePath + "triInterList_opt.bin"); //4030
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> tris5 = _readTrigonFile(binFilePath + "interTriInfo_4027.bin"); //4027

	//entity
	std::vector<std::array<uint64_t, 2>> entity1 = _readEntityIDFile(binFilePath + "entityIdList_off.bin"); //4031
	std::vector<std::array<uint64_t, 2>> entity2 = _readEntityIDFile(binFilePath + "entityIdList_on.bin"); //4031
	std::vector<std::array<uint64_t, 2>> entity3 = _readEntityIDFile(binFilePath + "entityIdList_soft_off.bin"); //1069
	std::vector<std::array<uint64_t, 2>> entity4 = _readEntityIDFile(binFilePath + "entityIdList_hard.bin"); //4030
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

//读取entityIdList
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
	std::vector<std::array<uint64_t, 2>> entity1 = _readEntityIDFile(binFilePath + "entityIdList_hard.bin"); //4030
	std::vector<std::array<uint64_t, 2>> entity2 = _readEntityIDFile(binFilePath + "entityIdList_soft.bin"); //1069
	std::vector<std::array<uint64_t, 2>> entity3 = _readEntityIDFile(binFilePath + "entityIdList_4031.bin"); //4031
	std::vector<std::array<uint64_t, 2>> entity4 = _readEntityIDFile(binFilePath + "entityIdList_4027.bin"); //4027
	std::vector<std::array<uint64_t, 2>> entity5;// = _readEntityIDFile(binFilePath + "entityIdList_4028.bin"); //4028
	std::vector<std::array<uint64_t, 2>> entity6;// = _readEntityIDFile(binFilePath + "entityIdList_4042.bin"); //4042

	//for (int i = 0; i < entity1.size(); ++i)
	//{
	//	bool findFlag = false;
	//	for (int j = 0; j < entity1.size(); ++j) //big vector, findTri
	//	{
	//		if (i == j)
	//			continue;
	//		if (entity1[i][0] == entity1[j][0] && entity1[i][1] == entity1[j][1])
	//		{
	//			findFlag = true;
	//			//cout << "index=" << j << endl; //823,950,954
	//			break;
	//		}
	//	}
	//	if (findFlag)
	//	{
	//		cout << "findFlag repeat" << endl; //no repeat
	//		cout << entity1[i][0] << "-" << entity1[i][1] << endl; // 6509 - 6134
	//	}
	//}

	cout << "findFlag extra entityid pair:" << endl;
	for (int i = 0; i < entity6.size(); ++i)
	{
		bool findFlag = false;
		for (int j = 0; j < entity5.size(); ++j) //big vector, findTri
		{
			if (entity6[i][0] == entity5[j][0] && entity6[i][1] == entity5[j][1])
			{
				findFlag = true;
				//cout << "index=" << j << endl; //823,950,954
				break;
			}
		}
		if (!findFlag)
		{
			cout << entity6[i][0] << ", " << entity6[i][1] << endl;
		}
	}
	cout << "return 0" << endl;
}

//数据测试
static void _test0()
{
	double rd = double(rand());// / RAND_MAX;
	rd = double(rand());
	rd = double(rand());
	// 不相交的三角形
	Vector3d triA_0 = Vector3d(4924494.8122771187, -385870.18283433426, 5749.9999999999054);
	Vector3d triA_1 = Vector3d(4924599.8122771177, -385945.18283433421, 5749.9999999999054);
	Vector3d triA_2 = Vector3d(4924586.8713248633, -385946.88654301979, 5749.9999999999054);
	Vector3d triB_0 = Vector3d(4924601.8102601077, -385940.89359764993, 5750.0000000000000);
	Vector3d triB_1 = Vector3d(4924595.2577039087, -385951.32193110074, 5750.0000000000000);
	Vector3d triB_2 = Vector3d(4924589.8109916430, -385975.18553675216, 5750.0000000000000);
	Triangle triA = { triA_0, triA_1, triA_2 };
	Triangle triB = { triB_0, triB_1, triB_2 };
	//double d = getTrianglesDistanceSAT(triA, triB);

	triA_0 = Vector3d(0, 0, 0);
	triA_1 = Vector3d(0, 10, 0);
	triA_2 = Vector3d(10, 5, 0);
	triB_0 = Vector3d(-10, 0, 0);
	triB_1 = Vector3d(-10, 10, 0);
	triB_2 = Vector3d(-5, 5, 5);
	triA = { triA_0, triA_1, triA_2 };
	triB = { triB_0, triB_1, triB_2 };

	double d0 = getTrianglesDistance(P, Q, triA, triB);
	double d1 = getTrianglesDistanceSAT(triA, triB);
	//double d2 = getDistanceOfPointAndPlaneINF(Vector3d(0, 0, 0), triB);
	double d = std::nan("0");
	if (0.0 > d) // nan not greater, not less
		d++;
	if (0.0 < d)
		d++;

	Vector3d v1(1, 0, 0);
	Vector3d v2(1, 0, 0);
	Vector3d v3 = v1.cross(v2).normalized(); //0的单位化还是0

	size_t m2 = ULL_MAX1 + 2; //归零重新开始计数
	double dm1 = DBL_MAX + 1;
	double dm2 = DBL_MAX + DBL_MAX;
	double dm3 = 2 * DBL_MAX;
	double dm4 = DBL_MAX + 10000;
	double dm5 = 1.1 * DBL_MAX;
	double dm6 = DBL_MAX + 1e300;

	double _inf = std::numeric_limits<double>::infinity();
	double _max = std::numeric_limits<double>::max();
	double _min = std::numeric_limits<double>::min();
	bool s1 = signbit(1.2);
	bool s2 = signbit(-1.2);
	bool isi = isinf(DBL_MAX + 1);
	bool isf = isfinite(gVecNaN[0]);// :ture is a finite value, false otherwise。可用来一起判断inf和nan。
	cout << "return 0;" << endl;
}

//距离划分等级
#ifdef USING_FLATBUFFERS_SERIALIZATION
static void _test4() //优化SAT的侵入距离计算，验证
{
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> triRec;

	std::vector<InterTriInfo> triInfo4027 = read_InterTriInfo(binFilePath + "interTriInfo_4027.bin");
	std::vector<InterTriInfo> triInfo4042 = read_InterTriInfo(binFilePath + "interTriInfo_4042.bin");
	// Category
	std::vector<InterTriInfo> triInfoSq;
	std::vector<InterTriInfo> triInfoZero;
	std::vector<InterTriInfo> triInfo1e_6;
	std::vector<InterTriInfo> triInfo1e_1;
	std::vector<InterTriInfo> triInfo1;
	std::vector<InterTriInfo> triInfo10;
	std::vector<InterTriInfo> triInfo100;
	std::vector<InterTriInfo> triInfo200;

	// find diff
	std::vector<InterTriInfo> triInfo4034 = read_InterTriInfo(binFilePath + "interTriInfo_4034.bin"); //more, latest
	//std::vector<InterTriInfo> triInfo4034_all = read_InterTriInfo(binFilePath + "interTriInfo_4034_all.bin"); // all-iter
	std::vector<InterTriInfo> triInfo4034_all = read_InterTriInfo(binFilePath + "interTriInfo_5108.bin"); 
	std::sort(triInfo4034_all.begin(), triInfo4034_all.end(), _opLessInfo);
	triInfoSq = triInfo4034;
	std::sort(triInfoSq.begin(), triInfoSq.end(), _opLessInfo); //-200
	//std::vector<InterTriInfo> triInfo10_122;
	//for (const auto& iter : interTriInfo_4034_5)
	//{
	//	if (-10.0 < iter.distance && iter.distance <= -1.0)
	//		triInfo10_122.push_back(iter);
	//}
	std::vector<ModelMesh> meshs_5233 = read_ModelMesh(binFilePath + "cvtMeshVct_con.bin");

	std::vector<InterTriInfo> triInfoDiff;
	std::vector<double> triInfoDiffd;
	std::vector<tuple<size_t, size_t>> triInfoDiffCon; // 全是convex的entityid
	double d_ins6, d_ins7; //-32.011189515236765
	bool findflag = false;
	bool isCon = false;
	for (int i = 0; i < triInfo4034.size(); ++i )
	{
		for (int j = 0; j < triInfo4034_all.size(); ++j )
		{
			if (i <= j)
				continue;
			if (triInfo4034[i].entityPair[0] == triInfo4034_all[j].entityPair[0] &&
				triInfo4034[i].entityPair[1] == triInfo4034_all[j].entityPair[1] &&
				fabs(triInfo4034[i].distance - triInfo4034_all[j].distance) > eps)
			{
				double d1 = triInfo4034[i].distance;
				double d2 = triInfo4034_all[j].distance;
				bool conA = false, conB = false;// = triInfo4034[i].entityPair
				for (auto& iter : meshs_5233)
				{
					if (iter.entityid_ == triInfo4034[i].entityPair[0] && iter.convex_)
						conA = true;
					if (iter.entityid_ == triInfo4034[i].entityPair[1] && iter.convex_)
						conB = true;
					if (iter.entityid_ == 1663 || iter.entityid_ == 1662)
						findflag = true;
					if (iter.entityid_ == 1663)
						isCon = isMeshConvexPolyhedron(iter);
				}
				if (conA && conB) //(1480, 1474)
					triInfoDiffCon.push_back({ triInfo4034[i].entityPair[0] ,triInfo4034[i].entityPair[1] });
				triInfoDiff.push_back(triInfo4034[i]);
				triInfoDiffd.push_back(d1 - d2);
			}
		}
	}

	//find difference 查找指定entityID
	for (int i = 0; i < triInfo4034.size(); ++i)
	{
		//
		bool findFlag = false;
		for (int j = 0; j < triInfo4027.size(); ++j) //big vector, findTri
		{
			if (triInfo4034[i].entityPair[0] == triInfo4027[j].entityPair[0] &&
				triInfo4034[i].entityPair[1] == triInfo4027[j].entityPair[1])
			{
				findFlag = true;
				//cout << "index=" << j << endl; //823,950,954
				break;
			}
		}
		if (!findFlag)
		{
			//cout << triInfo4034[i].entityPair[0] << ", " << triInfo4034[i].entityPair[1] << endl;
			triRec.push_back(triInfo4034[i].trianglePair);
		}
		// the inside
		size_t idA = 1418;
		size_t idB = 1417;
		if (triInfo4034[i].entityPair[0] == 6564 && triInfo4034[i].entityPair[1] == 4293)
			d_ins6 = triInfo4034[i].distance;
		if (triInfo4034[i].entityPair[0] == 6563 && triInfo4034[i].entityPair[1] == 4296)
			d_ins7 = triInfo4034[i].distance;
	}

	for (int i = 0; i < triInfoSq.size(); ++i)
	{
		if (triInfoSq[i].distance <= -100)
			triInfo200.push_back(triInfoSq[i]); // d=-196(6569,4279) d=-101(6572,4267)
		else if (triInfoSq[i].distance <= -10)
			triInfo100.push_back(triInfoSq[i]);
		else if (triInfoSq[i].distance <= -1)
			triInfo10.push_back(triInfoSq[i]);
		else if (triInfoSq[i].distance <= -1e-1) //d=-9.5(5268,4706)
			triInfo1.push_back(triInfoSq[i]);
		else if (triInfoSq[i].distance <= -1e-6)
			triInfo1e_1.push_back(triInfoSq[i]);
		else if (triInfoSq[i].distance == 0.0) //d=0(4224,4221)
			triInfoZero.push_back(triInfoSq[i]);
		else 
			triInfo1e_6.push_back(triInfoSq[i]); //d=1e-7(6125,6123)
		// d=-2.3283064365386963e-10
		//d<0 1521,1519
		//d<0 2978,2976
	}


	/*
	1443, 1411
	1662, 1642
	1935, 1903
	2153, 2133
	2255, 2243
	//---
	2418, 2386
	2636, 2616
	2901, 2869
	3119, 3099
	3384, 3352
	//---
	3599, 3579
	3864, 3832
	4080, 4060
	4478, 4446
	6509, 6134 //min
	//inside
	1418, 1417
	1910, 1909
	2393, 2392
	2876, 2875
	3359, 3358
	3839, 3838
	4453, 4452
	*/
	for (auto& iter : triRec)
	{
		bool isInter0 = isTwoTrianglesIntersectSAT(iter[0], iter[1]);
		bool isInter1 = isTwoTrianglesIntersectEIT(iter[0], iter[1]);
		bool isInter2 = isTwoTrianglesIntersectPIT(iter[0], iter[1]);
		//cout << endl;
	}
	bool isInter0 = isTwoTrianglesIntersectSAT(triRec.back()[0], triRec.back()[1]);
	Vector3d triA_0(4934991.0849248841, -380736.84932345786, -216.33004252886147);
	Vector3d triA_1(4934991.0849241894, -380719.84932345781, -216.33004252886150);
	Vector3d triA_2(4934984.3686963469, -380736.84932373225, -213.09567733115546);
	Vector3d triB_0(4934994.8122385293, -382890.18308486621, -217.00000000000003);
	Vector3d triB_1(4934994.8121505287, -380736.84932330565, -217.00000000000003);
	Vector3d triB_2(4934988.3065321781, -380736.84932357143, -215.70595205269194);
	double d0 = getTrianglesDistance(P, Q, { triA_0 ,triA_2 }, { triB_1 ,triB_2 });
	double er = (triA_2 - triA_0).cross(triB_1 - triA_0).dot(triB_2 - triA_0);

	Triangle triA = { triA_0, triA_1, triA_2 };
	Triangle triB = { triB_0, triB_1, triB_2 };
	double d1 = getTrianglesDistanceSAT(triA, triB);
	auto pnt = getTwoTrianglesNearestPoints(triA, triB);
	auto pnt2 = getTwoTrianglesIntersectPoints(triA, triB);

	cout << "triInfoSq:" << triInfoSq.size() << endl;
	cout << "triInfoZero:" << triInfoZero.size() << endl;
	cout << "triInfo1e_6:" << triInfo1e_6.size() << endl;
	cout << "triInfo1e_1:" << triInfo1e_1.size() << endl;
	cout << "triInfo1:" << triInfo1.size() << endl;
	cout << "triInfo10:" << triInfo10.size() << endl;
	cout << "triInfo100:" << triInfo100.size() << endl;
	cout << "triInfo200:" << triInfo200.size() << endl;

	cout << "return 0" << endl;
}

//读取二进制mesh
static void _test5()
{
	//std::vector<ModelMesh> meshs0;
	//std::vector<ModelMesh> meshs = _read_ModelMesh(filename + "cvtMeshVct_6509_6134.bin");
	//std::vector<ModelMesh> meshs_5233 = read_ModelMesh(binFilePath + "cvtMeshVct_jingujun.bin");
	std::vector<ModelMesh> meshs_jgj = read_ModelMesh(binFilePath + "cvtMeshVct_jgj.bin");
	std::vector<ModelMesh> meshs_5233 = read_ModelMesh(binFilePath + "cvtMeshVct_con.bin");
	std::vector<ModelMesh> meshs_con = read_ModelMesh(binFilePath + "cvtMeshVct_con.bin");
	//std::vector<ModelMesh> meshs0 = read_ModelMesh(binFilePath + "cvtMeshVct_1.bin"); //old fbs file
	//std::vector<ModelMesh> meshs2 = read_ModelMesh(binFilePath + "cvtMeshVct_2.bin");
	//std::vector<ModelMesh> meshs3 = read_ModelMesh(binFilePath + "cvtMeshVct_3.bin");
	//std::vector<ModelMesh> meshs4 = read_ModelMesh(binFilePath + "cvtMeshVct_4.bin");
	//std::vector<ModelMesh> meshs5 = read_ModelMesh(binFilePath + "cvtMeshVct_5.bin");

	std::vector<ModelMesh> meshs_conDiff;
	for (size_t i = 0; i < meshs_jgj.size(); i++)
	{
		if (meshs_jgj[i].convex_ != meshs_con[i].convex_)
			meshs_conDiff.push_back(meshs_con[i]);
		//1308, 1317 1794, 5216
	}

	size_t count = 0, countInter = 0, countPre = 0, countLoop = 0;
	clock_t startT, endT;

	//测试一个点遍历所有mesh的效率
	Vector3d point = Vector3d(rand(), rand(), rand());
	//Vector3d point = Vector3d(0, 0, 0);
	startT = clock();
	bool isIn;
	int nFace = 0, nVertex = 0;
	//for (const auto& iter: meshs_5233)
		//bool res = isPointInsidePolyhedronCEIL(point, iter);
		//bool res = isPointInsidePolyhedronAZ(point, iter);
		//RelationOfPointAndMesh res = isPointInsidePolyhedronRZ(point, iter);
	for (int i = 0; i < meshs_5233.size()/2; ++i)
	{
		nFace += meshs_5233[i].ibo_.size(); //1982041  //1031776
		nVertex += meshs_5233[i].vbo_.size(); //977267  //506709
	}

#define TEST_TIME_COST
#ifdef TEST_TIME_COST
#pragma omp parallel for // omp optimize
	for (int i = 0; i < meshs_5233.size()/20; ++i)
	{
		for (int j = 0; j < meshs_5233.size()/20; ++j)
		{
			if (i <= j)
				continue;
			for (const auto& iter : meshs_5233[i].vbo_)
			{
				/*isIn =*/ 
				isPointInsidePolyhedronRZ(iter, meshs_5233[j]);
				//isIn = isPointInsidePolyhedronAZ(iter, meshs_5233[j]);
				//isIn = isPointInsidePolyhedronCL(iter, meshs_5233[j]);
				//isIn = isPointInsidePolyhedronFL(iter, meshs_5233[j]);
				countLoop++;
			}
		}
	}
#endif
	endT = clock();
	cout << "mesh count=" << meshs_5233.size() << endl;
	cout << "count=" << count << endl;
	cout << "countLoop=" << countLoop << endl;
	cout << "time=" << double(endT - startT) / CLOCKS_PER_SEC << "s" << endl;
	cout << "return 0;" << endl;
}

//三角形距离
static void _test6()
{
	std::vector<ModelMesh> meshs = read_ModelMesh(binFilePath + "cvtMeshVct_cube2.bin");
	//std::vector<ModelMesh> meshs = _read_ModelMesh(binFilePath + "cvtMeshVct_intrusion2.bin");
	ModelMesh meshA = meshs[0];
	ModelMesh meshB = meshs[1];

	RelationOfPointAndMesh b0 = isPointInsidePolyhedronRZ(Vector3d(150, 50, 50), meshA.vbo_, meshA.ibo_);

	//Vector3d df = getPenetrationDepthOfTwoMeshs(meshA, meshB);
	//Vector3d di = getPenetrationDepthOfTwoMeshs(meshB, meshA);
	auto df = getTwoMeshsIntersectRelation(meshA, meshB);
	auto di = getTwoMeshsIntersectRelation(meshB, meshA);

	std::array<Vector3d, 2> segmA = { Vector3d(0,0,0), Vector3d(100,100,0) };
	std::array<Vector3d, 2> segmB = { Vector3d(100,0,100), Vector3d(0,100,100) };
	bool b1 = isTwoSegmentsIntersect(segmA, segmB);

	cout << "return 0" << endl;
}
#endif // USING_FLATBUFFERS_SERIALIZATION

//优化测试
static void _test7()
{

	Vector3d triA_0 = Vector3d(4936998.2332053846, -383787.17924958991, 6015.6846000000369);
	Vector3d triA_1 = Vector3d(4937024.8494758252, -383779.28694613208, 6070.1700860465535);
	Vector3d triA_2 = Vector3d(4937029.5225078566, -383777.90129043174, 6060.6040417688109);
	Vector3d triB_0 = Vector3d(4937021.0285847718, -383780.41992348642, 6057.7254248593736);
	Vector3d triB_1 = Vector3d(4937022.2016856968, -383780.07207353070, 6050.0000000000000);
	Vector3d triB_2 = Vector3d(4936824.6681833472, -383113.90342609736, 6050.0000000000000);

	Triangle triA = { triA_0, triA_1, triA_2 };
	Triangle triB = { triB_0, triB_1, triB_2 };
	bool issat = isTwoTrianglesIntersectSAT(triA, triB);


	std::vector<ModelMesh> cvtMeshVct_con = read_ModelMesh(binFilePath + "cvtMeshVct_con.bin"); //all mesh
	std::vector<InterTriInfo> triInfo4034 = read_InterTriInfo(binFilePath + "interTriInfo_4034.bin");
	//size_t idA = 1521, idB = 1519; //same model
	//size_t idA = 2978, idB = 2976;
	//size_t idA = 1480, idB = 1474; //intru diff
	//size_t idA = 6563, idB = 4296; // dp不同
	//size_t idA = 6564, idB = 4293;
	//size_t idA = 3871, idB = 3391; //双杆
	size_t idA = 1521, idB = 1519; //inter but d<0

	ModelMesh meshA, meshB;
	double dtest;
	for (const auto& iter : cvtMeshVct_con)
	{
		if (iter.entityid_ == idA)
			meshA = iter;
		if (iter.entityid_ == idB)
			meshB = iter;
		//bool b1 = isConvex(iter.vbo_, iter.ibo_);
		//bool b2 = is_convex(iter.vbo_, iter.ibo_);
		//cout << b1 << b2 << endl;
		//bool isConvex = is_convex(iter.vbo_, iter.ibo_); //
	}

	//手动计算PenetrationDepth
	std::tuple<Eigen::Vector3d, std::array<size_t, 2>> res1 = getPenetrationDepthOfTwoConvexALL(meshA, meshB);
	double d1 = std::get<0>(res1).norm();
	bool isInter2=isTwoMeshsIntersectSAT(meshA, meshB);
	std::tuple<RelationOfTwoMesh, Eigen::Vector3d> res2 = getTwoMeshsIntersectRelation(meshA, meshB);
	double d2 = std::get<1>(res2).norm();
	double dinfo;
	for (int i = 0; i < triInfo4034.size(); ++i)
	{
		if (triInfo4034[i].entityPair[0] == idA &&
			triInfo4034[i].entityPair[1] == idB )
		{
			dinfo = triInfo4034[i].distance;
		}
	}
	// without d = -75.000000000040018
	 //with vertex d = -131.80984782427549

	//相交测试
	bool isInter = false;
	double distance = nan("0");
	for (size_t i = 0; i < meshA.ibo_.size(); i++)
	{
		for (size_t j = 0; j < meshB.ibo_.size(); j++)
		{
			std::array<Eigen::Vector3d, 3> triA = {
				meshA.vbo_[meshA.ibo_[i][0]],
				meshA.vbo_[meshA.ibo_[i][1]],
				meshA.vbo_[meshA.ibo_[i][2]] };
			std::array<Eigen::Vector3d, 3> triB = {
				meshB.vbo_[meshB.ibo_[j][0]],
				meshB.vbo_[meshB.ibo_[j][1]],
				meshB.vbo_[meshB.ibo_[j][2]] };
			if (isTwoTrianglesBoundingBoxIntersect(triA, triB))
			{
				if (isTwoTrianglesIntersectSAT(triA, triB))
				{
					isInter = true;
					//distance = getTrianglesDistanceSAT(triA, triB);
				}
			}
		}
	}
	bool isInter1= isTwoMeshsIntersectSAT(meshA, meshB);


	cout << "return 0;" << endl;
}

static int enrol = []()->int
{
#ifdef USING_FLATBUFFERS_SERIALIZATION
	//_test0();
	_test4();
	//_test5();
	//_test6();
	_test7();
#endif
	return 0;
}();
