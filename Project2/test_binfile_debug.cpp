#include "pch.h"
#include <iomanip>
using namespace std;
using namespace para;
using namespace Eigen;
using namespace psykronix;

static Eigen::Vector3d P(std::nan("0"), std::nan("0"), std::nan("0"));
static Eigen::Vector3d Q(std::nan("0"), std::nan("0"), std::nan("0"));
static std::string binFilePath = "C:/Users/Aking/source/repos/bimbase/src/P3d2Stl/bin_file/";

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
	std::vector<std::array<uint64_t, 2>> entity5;// = _readEntityIDFile(path + "entityIdList_48677.bin"); //48677
	std::vector<std::array<uint64_t, 2>> entity6;// = _readEntityIDFile(path + "entityIdList_48678.bin"); //48678
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

#ifdef USING_FLATBUFFERS_SERIALIZATION
//测试修复SAT相关
static void _test4()
{
	std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> triRec;

	std::vector<InterTriInfo> triInfo1=read_InterTriInfo(binFilePath + "interTriInfo_4027.bin");
	std::vector<InterTriInfo> triInfo2=read_InterTriInfo(binFilePath + "interTriInfo_4028.bin"); //more
	//find difference
	for (int i = 0; i < triInfo2.size(); ++i)
	{
		bool findFlag = false;
		for (int j = 0; j < triInfo1.size(); ++j) //big vector, findTri
		{
			if (triInfo2[i].entityPair[0] == triInfo1[j].entityPair[0] && 
				triInfo2[i].entityPair[1] == triInfo1[j].entityPair[1])
			{
				findFlag = true;
				//cout << "index=" << j << endl; //823,950,954
				break;
			}
		}
		if (!findFlag)
		{
			cout << triInfo2[i].entityPair[0] << ", " << triInfo2[i].entityPair[1] << endl;
			triRec.push_back(triInfo2[i].trianglePair);
		}
	}
	/*
	1443, 1411
	1662, 1642
	1935, 1903
	2153, 2133
	2255, 2243
	2418, 2386
	2636, 2616
	2901, 2869
	3119, 3099
	3384, 3352
	3599, 3579
	3864, 3832
	4080, 4060
	4478, 4446
	6509, 6134
	*/
	for (auto& iter : triRec)
	{
		bool isInter = isTwoTrianglesIntersectSAT(iter[0], iter[1]);
		bool isInter1 = isTwoTrianglesIntersectEIT(iter[0], iter[1]);
		bool isInter2 = isTwoTrianglesIntersectPIT(iter[0], iter[1]);
		cout << endl;
	}

	cout << "return 0" << endl;
}

static void _test5()
{
	//读取二进制mesh
	//std::vector<ModelMesh> meshs = _read_ModelMesh(filename + "cvtMeshVct_5233.bin");
	//std::vector<ModelMesh> meshs = _read_ModelMesh(filename + "cvtMeshVct_6509_6134.bin");
	std::vector<ModelMesh> meshs0 = read_ModelMesh(binFilePath + "cvtMeshVct_1.bin");
	std::vector<ModelMesh> meshs2 = read_ModelMesh(binFilePath + "cvtMeshVct_2.bin");
	std::vector<ModelMesh> meshs3 = read_ModelMesh(binFilePath + "cvtMeshVct_3.bin");
	std::vector<ModelMesh> meshs4 = read_ModelMesh(binFilePath + "cvtMeshVct_4.bin");
	std::vector<ModelMesh> meshs5 = read_ModelMesh(binFilePath + "cvtMeshVct_5.bin");
	size_t count = 0, countInter = 0;
	clock_t startT, endT;
	startT = clock();

	for (const auto& iter : meshs0)
	{
		//bool b1 = isConvex(iter.vbo_, iter.ibo_);
		//bool b2 = is_convex(iter.vbo_, iter.ibo_);
		//cout << b1 << b2 << endl;
		//bool isConvex = is_convex(iter.vbo_, iter.ibo_); //
		bool isConvex = isMeshConvexPolyhedron(iter.vbo_, iter.ibo_); //
		if (!isConvex)
			count++;
	}
	//相交测试
	bool isInter = false;
	ModelMesh mesh_a = meshs0[0];
	ModelMesh mesh_b = meshs0[1];
	for (size_t i = 0; i < mesh_a.ibo_.size(); i++)
	{
		for (size_t j = 0; j < meshs0[1].ibo_.size(); j++)
		{
			std::array<Eigen::Vector3d, 3> triA = {
				mesh_a.vbo_[mesh_a.ibo_[i][0]],
				mesh_a.vbo_[mesh_a.ibo_[i][1]],
				mesh_a.vbo_[mesh_a.ibo_[i][2]] };
			std::array<Eigen::Vector3d, 3> triB = {
				mesh_b.vbo_[mesh_b.ibo_[j][0]],
				mesh_b.vbo_[mesh_b.ibo_[j][1]],
				mesh_b.vbo_[mesh_b.ibo_[j][2]] };
			if (isTwoTrianglesBoundingBoxIntersect(triA, triB))
			{
				countInter++;
				if (isTwoTrianglesIntersectSAT(triA, triB))
					isInter = true;
			}
		}
	}

	endT = clock();
	cout << "mesh count=" << meshs0.size() << endl;
	cout << "count=" << count << endl;
	cout << "time=" << double(endT - startT) / CLOCKS_PER_SEC << "s" << endl;
	cout << "return 0" << endl;
}

//三角形距离
static void _test6()
{
	std::vector<ModelMesh> meshs = read_ModelMesh(binFilePath + "cvtMeshVct_cube2.bin");
	//std::vector<ModelMesh> meshs = _read_ModelMesh(binFilePath + "cvtMeshVct_intrusion2.bin");
	ModelMesh meshA = meshs[0];
	ModelMesh meshB = meshs[1];

	bool b0 = isPointInsidePolyhedron(Vector3d(150, 50, 50), meshA.vbo_, meshA.ibo_);

	Vector3d df = getPenetrationDepthOfTwoMeshs(meshA, meshB);
	Vector3d di = getPenetrationDepthOfTwoMeshs(meshB, meshA);

	std::array<Vector3d, 2> segmA = { Vector3d(0,0,0), Vector3d(100,100,0) };
	std::array<Vector3d, 2> segmB = { Vector3d(100,0,100), Vector3d(0,100,100) };
	bool b1 = isTwoSegmentsIntersect(segmA, segmB);

	cout << "d_min=" << df << endl;
	cout << "return 0" << endl;
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
	double d2 = getDistanceOfPointAndPlaneINF(Vector3d(0, 0, 0), triB);
	d2 = sqrt(d2);

}
#endif // USING_FLATBUFFERS_SERIALIZATION


static int enrol = []()->int
{
#ifdef USING_FLATBUFFERS_SERIALIZATION
	_test4();
	//_test5();
	//_test6();
#endif
	return 0;
}();
