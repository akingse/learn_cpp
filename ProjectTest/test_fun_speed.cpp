#include "pch.h"
#include "commonFileReadWrite.h"

#include <synchapi.h>
using namespace std;
using namespace para;
using namespace Eigen;
using namespace eigen;
using namespace clash;
#undef max //AlignedBox3d mem-fun
#undef min
//精度
static constexpr float eps = 1e-6;
static constexpr float epsF = FLT_EPSILON;
static constexpr double epsD = DBL_EPSILON;
extern std::atomic<size_t>  count_pointInTri, count_edgeCrossTri, TriangularIntersectC, count_segCrossTri, count_across,
TriangularIntersectC_in, TriangularIntersectC_all;

auto eps1 = std::numeric_limits<float>::epsilon();
auto eps2 = std::numeric_limits<double>::epsilon();
Eigen::Vector3d P(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
Eigen::Vector3d Q(std::nan("0"), std::nan("0"), std::nan("0"));

//#define RSS_TYPE     1
//#define OBB_TYPE     2
constexpr int RSS_TYPE = 1;
constexpr int OBB_TYPE = 2;
// abcdefghijklmnopqrstuvwxyz
// ABCDEFGHIJKLMNOPQRSTUVWXYZ

const size_t totalNum = (size_t)1e8;

// //std::array 数组的总大小不能超过0x7fffffff个字节 2G  //with max size 2e7
//std::array<std::array<Vector3d, 3>, totalNum>* randData3 = new std::array<std::array<Vector3d, 3>, totalNum>;
//std::array<std::array<Vector3d, 3>, totalNum>* randData3_ = new std::array<std::array<Vector3d, 3>, totalNum>;
//std::array<Vector3d, 3>* randData3 = new std::array<Vector3d, 3>;
//std::array<Vector3d, 3>* randData3_ = new std::array<Vector3d, 3>;

#define TEST_SPEED_OF_FUNCTION
//#define TEST_TRIGON3
//#define TEST_TRIGON2_TRIGON2
//#define TEST_TRIGON2D_TRIGON2D
//#define TEST_TRIGON1_TRIGON3
//#define TEST_TRIGON2_TRIGON3
#define TEST_TRIGON3_TRIGON3
//#define TEST_TRIGON3XY_TRIGON3XY //on XOY plane
static std::string randNumName = "bin_file/random_1e8.bin";
static std::string randNumNameSepa = "bin_file/random_1e8_sepa.bin";

//vector<> 版本非常慢，50s+
static bool isTwoTrianglesIntrusionSAT_vector(const std::array<Vector3d, 3>& triA, const std::array<Vector3d, 3>& triB,
	const Eigen::Vector3d& normalA, const Eigen::Vector3d& normalB, double tolerance = 0.0)
{
	std::array<Eigen::Vector3d, 3> edgesA = {
		triA[1] - triA[0],
		triA[2] - triA[1],
		triA[0] - triA[2] };
	std::array<Eigen::Vector3d, 3> edgesB = {
		triB[1] - triB[0],
		triB[2] - triB[1],
		triB[0] - triB[2] };
	std::vector<Eigen::Vector3d> axes(17);
    axes[0] = normalA;
	axes[1] = normalB;
    axes[2] = edgesA[0].cross(edgesB[0]).normalized();
    axes[3] = edgesA[0].cross(edgesB[1]).normalized();
    axes[4] = edgesA[0].cross(edgesB[2]).normalized();
    axes[5] = edgesA[1].cross(edgesB[0]).normalized();
    axes[6] = edgesA[1].cross(edgesB[1]).normalized();
    axes[7] = edgesA[1].cross(edgesB[2]).normalized();
    axes[8] = edgesA[2].cross(edgesB[0]).normalized();
    axes[9] = edgesA[2].cross(edgesB[1]).normalized();
    axes[10] = edgesA[2].cross(edgesB[2]).normalized();
    axes[11] = normalA.cross(edgesA[0]).normalized();
    axes[12] = normalA.cross(edgesA[1]).normalized();
    axes[13] = normalA.cross(edgesA[2]).normalized();
    axes[14] = normalB.cross(edgesB[0]).normalized();
    axes[15] = normalB.cross(edgesB[1]).normalized();
    axes[16] = normalB.cross(edgesB[2]).normalized();

	//if (normalA.cross(normalB).isZero())//is parallel
	//{
	//	if (tolerance < fabs(normalA.dot(triB[0] - triA[0]))) //not coplanar
	//		return false;
	//	axes.resize(6);
	//	axes[0] = normalA.cross(edgesA[0]).normalized();
	//	axes[1] = normalA.cross(edgesA[1]).normalized();
	//	axes[2] = normalA.cross(edgesA[2]).normalized();
	//	axes[3] = normalB.cross(edgesB[0]).normalized();
	//	axes[4] = normalB.cross(edgesB[1]).normalized();
	//	axes[5] = normalB.cross(edgesB[2]).normalized();
	//}
	//else
	//{
	//	//axes.resize(11);
	//	axes[0] = normalA;
	//	axes[1] = normalB;
	//	axes[2] = edgesA[0].cross(edgesB[0]).normalized();
	//	axes[3] = edgesA[0].cross(edgesB[1]).normalized();
	//	axes[4] = edgesA[0].cross(edgesB[2]).normalized();
	//	axes[5] = edgesA[1].cross(edgesB[0]).normalized();
	//	axes[6] = edgesA[1].cross(edgesB[1]).normalized();
	//	axes[7] = edgesA[1].cross(edgesB[2]).normalized();
	//	axes[8] = edgesA[2].cross(edgesB[0]).normalized();
	//	axes[9] = edgesA[2].cross(edgesB[1]).normalized();
	//	axes[10] = edgesA[2].cross(edgesB[2]).normalized();
	//}
	double minA, maxA, minB, maxB, projection;
	for (const Vector3d& axis : axes)
	{
		if (axis.isZero())
			continue;
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const Vector3d& vertex : triA)
		{
			projection = axis.dot(vertex);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const Vector3d& vertex : triB)
		{
			projection = axis.dot(vertex);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (maxA < minB + tolerance || maxB < minA + tolerance)
			return false;
	}
	return true;
}

static bool isTwoTrianglesIntrusionSAT_c_array(const std::array<Vector3d, 3>& triA, const std::array<Vector3d, 3>& triB,
	const Eigen::Vector3d& normalA, const Eigen::Vector3d& normalB, double tolerance = 0.0)
{
	std::array<Eigen::Vector3d, 3> edgesA = {
		triA[1] - triA[0],
		triA[2] - triA[1],
		triA[0] - triA[2] };
	std::array<Eigen::Vector3d, 3> edgesB = {
		triB[1] - triB[0],
		triB[2] - triB[1],
		triB[0] - triB[2] };
	//Eigen::Vector3d axes[17]; //49s
	std::array<Eigen::Vector3d, 17> axes;//55s
	axes[0] = normalA;
	axes[1] = normalB;
	axes[2] = edgesA[0].cross(edgesB[0]).normalized();
	axes[3] = edgesA[0].cross(edgesB[1]).normalized();
	axes[4] = edgesA[0].cross(edgesB[2]).normalized();
	axes[5] = edgesA[1].cross(edgesB[0]).normalized();
	axes[6] = edgesA[1].cross(edgesB[1]).normalized();
	axes[7] = edgesA[1].cross(edgesB[2]).normalized();
	axes[8] = edgesA[2].cross(edgesB[0]).normalized();
	axes[9] = edgesA[2].cross(edgesB[1]).normalized();
	axes[10] = edgesA[2].cross(edgesB[2]).normalized();
	axes[11] = normalA.cross(edgesA[0]).normalized();
	axes[12] = normalA.cross(edgesA[1]).normalized();
	axes[13] = normalA.cross(edgesA[2]).normalized();
	axes[14] = normalB.cross(edgesB[0]).normalized();
	axes[15] = normalB.cross(edgesB[1]).normalized();
	axes[16] = normalB.cross(edgesB[2]).normalized();
	double minA, maxA, minB, maxB, projection;
	//for (const Vector3d& axis : axes)
    for (int i = 0; i < 17; i++)
	{
		const Vector3d& axis = axes[i];
		if (axis.isZero())
			continue;
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const Vector3d& vertex : triA)
		{
			projection = axis.dot(vertex);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const Vector3d& vertex : triB)
		{
			projection = axis.dot(vertex);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (maxA < minB + tolerance || maxB < minA + tolerance)
			return false;
	}
	return true;
}

static bool isTwoTrianglesIntrusionSAT_array(const std::array<Vector3d, 3>& triA, const std::array<Vector3d, 3>& triB,
	const Eigen::Vector3d& normalA, const Eigen::Vector3d& normalB, double tolerance = 0.0)
{
	std::array<Eigen::Vector3d, 3> edgesA = {
		triA[1] - triA[0],
		triA[2] - triA[1],
		triA[0] - triA[2] };
	std::array<Eigen::Vector3d, 3> edgesB = {
		triB[1] - triB[0],
		triB[2] - triB[1],
		triB[0] - triB[2] };
	std::array<Eigen::Vector3d, 17> axes = { {
		edgesA[0].cross(edgesB[0]).normalized(),
		edgesA[0].cross(edgesB[1]).normalized(),
		edgesA[0].cross(edgesB[2]).normalized(),
		edgesA[1].cross(edgesB[0]).normalized(),
		edgesA[1].cross(edgesB[1]).normalized(),
		edgesA[1].cross(edgesB[2]).normalized(),
		edgesA[2].cross(edgesB[0]).normalized(),
		edgesA[2].cross(edgesB[1]).normalized(),
		edgesA[2].cross(edgesB[2]).normalized(),
		normalA, //normal direction projection
		normalB,
		normalA.cross(edgesA[0]).normalized(), //perpendi to edge when coplanar
		normalA.cross(edgesA[1]).normalized(),
		normalA.cross(edgesA[2]).normalized(),
		normalB.cross(edgesB[0]).normalized(),
		normalB.cross(edgesB[1]).normalized(),
		normalB.cross(edgesB[2]).normalized()
		} };
	double minA, maxA, minB, maxB, projection;
	for (const Vector3d& axis : axes) //25s
	//for (int i = 0; i < 17; i++) //30s
	{
		//const Vector3d& axis = axes[i];
		if (axis.isZero())
			continue;
		minA = DBL_MAX;
		maxA = -DBL_MAX;
		minB = DBL_MAX;
		maxB = -DBL_MAX;
		for (const Vector3d& vertex : triA)
		{
			projection = axis.dot(vertex);
			minA = std::min(minA, projection);
			maxA = std::max(maxA, projection);
		}
		for (const Vector3d& vertex : triB)
		{
			projection = axis.dot(vertex);
			minB = std::min(minB, projection);
			maxB = std::max(maxB, projection);
		}
		if (maxA < minB + tolerance || maxB < minA + tolerance)
			return false;
	}
	return true;
}

//测速
static void _test0()
{
	clock_t start, end;
	string exepath = "C:/Users/Aking/source/repos/learn_cpp/ProjectTest/";//"getExePath();
	cout << "exepath=" << exepath << endl;

#ifdef TEST_SPEED_OF_FUNCTION
	start = clock();
	cout << "data number:" << totalNum << ", load start..." << endl;
	// the data
    double* readNum = _readNumberFile(size_t(sqrt(totalNum)), exepath + randNumNameSepa);

#ifdef TEST_TRIGON3
	std::array<Vector3d, 3>* randData3 = new std::array<Vector3d, 3>[totalNum];
#endif

#ifdef TEST_TRIGON2_TRIGON2
	std::array<Vector3d, 2>* randData2 = new std::array<Vector3d, 2>[totalNum];
	std::array<Vector3d, 2>* randData2_ = new std::array<Vector3d, 2>[totalNum];
#endif
#ifdef TEST_TRIGON2D_TRIGON2D
	std::array<Vector2d, 3>* randData2D = new std::array<Vector2d, 3>[totalNum];
	std::array<Vector2d, 3>* randData2D_ = new std::array<Vector2d, 3>[totalNum];
#endif
#ifdef TEST_TRIGON1_TRIGON3
	std::array<Vector3d, 1>* randData1 = new std::array<Vector3d, 1>[totalNum];
	std::array<Vector3d, 3>* randData3 = new std::array<Vector3d, 3>[totalNum];
#endif

#ifdef TEST_TRIGON3_TRIGON3
	std::array<Vector3d, 3>* randData3 = new std::array<Vector3d, 3>[totalNum];
	std::array<Vector3d, 3>* randData3_ = new std::array<Vector3d, 3>[totalNum];
	Vector3d* randData3N = new Vector3d[totalNum];
	Vector3d* randData3N_ = new Vector3d[totalNum];
#endif

#ifdef TEST_TRIGON2_TRIGON3
	std::array<Vector3d, 3>* randData2 = new std::array<Vector3d, 2>[totalNum];
	std::array<Vector3d, 3>* randData3 = new std::array<Vector3d, 3>[totalNum];
#endif

#ifdef TEST_TRIGON3XY_TRIGON3XY
	std::array<Vector3d, 3>* randData3xy = new std::array<Vector3d, 3>[totalNum];
	std::array<Vector3d, 3>* randData3_xy = new std::array<Vector3d, 3>[totalNum];
#endif
	//for (size_t i = 0; i < totalNum; ++i)
	//{
	//	randData3[i] = _get_rand3();
	//	randData3_[i] = _get_rand3();
	//}
	size_t length = size_t(sqrt(totalNum));// (1e4);
	for (int i = 0; i < length; ++i)
	{
		for (int j = 0; j < length; ++j)
		{
#ifdef TEST_TRIGON3
			randData3[i * length + j] = { { {readNum[j + 0],readNum[j + 2],readNum[j + 4]} ,
											{readNum[j + 6], readNum[j + 8], readNum[j + 10]},
											{ readNum[j + 12],readNum[j + 14],readNum[j + 16] } } };
#endif

#ifdef TEST_TRIGON1_TRIGON3
			randData1[i * length + j] = { { {readNum[j + 0],readNum[j + 2],readNum[j + 4]} } };
			randData3[i * length + j] = { { {readNum[j + 1],readNum[j + 3],readNum[j + 5]} ,
											{readNum[j + 7],readNum[j + 9],readNum[j + 11]} ,
											{readNum[j + 13],readNum[j + 15],readNum[j + 17]} } };
#endif
#ifdef TEST_TRIGON3_TRIGON3
            randData3[i * length + j] = { { {readNum[j + 0] / 10,readNum[j + 2] / 10,readNum[j + 4] / 10} ,
											{readNum[j + 6] / 10,readNum[j + 8] / 10,readNum[j + 10] / 10} ,
											{readNum[j + 12] / 10,readNum[j + 14] / 10,readNum[j + 16] / 10} } };
			randData3_[i * length + j] = { { {readNum[j + 1] / 10,readNum[j + 3] / 10,readNum[j + 5] / 10} ,
											{readNum[j + 7] / 10,readNum[j + 9] / 10,readNum[j + 11] / 10} ,
											{readNum[j + 13] / 10,readNum[j + 15] / 10,readNum[j + 17] / 10} } };
			Vector3d normalA = (randData3[i * length + j][1] - randData3[i * length + j][0]).cross(randData3[i * length + j][2] - randData3[i * length + j][1]).normalized();
			Vector3d normalB = (randData3_[i * length + j][1] - randData3_[i * length + j][0]).cross(randData3_[i * length + j][2] - randData3_[i * length + j][1]).normalized();
			randData3N[i * length + j] = normalA;
			randData3N_[i * length + j] = normalB;
#endif
#ifdef TEST_TRIGON2D_TRIGON2D
			randData2D[i * length + j] = { { {readNum[j + 0],readNum[j + 2]} ,
											{readNum[j + 6],readNum[j + 8]},
											{readNum[j + 12],readNum[j + 14]} } };
			randData2D_[i * length + j] = { { {readNum[j + 1],readNum[j + 3]} ,
											{readNum[j + 7],readNum[j + 9]} ,
											{readNum[j + 13],readNum[j + 15]} } };
#endif
#ifdef TEST_TRIGON2_TRIGON3
			randData2[i * length + j] = { { {readNum[j + 0],readNum[j + 2],readNum[j + 4]} ,
											{readNum[j + 6],readNum[j + 8],readNum[j + 10]} } };
			randData3[i * length + j] = { { {readNum[j + 1],readNum[j + 3],readNum[j + 5]} ,
											{readNum[j + 7],readNum[j + 9],readNum[j + 11]} ,
											{readNum[j + 13],readNum[j + 15],readNum[j + 17]} } };
#endif
#ifdef TEST_TRIGON2_TRIGON2

			randData2[i * length + j] = { { {readNum[j + 0],readNum[j + 2],readNum[j + 4]} ,
											{readNum[j + 6],readNum[j + 8],readNum[j + 10]} } };
			randData2_[i * length + j] = { { {readNum[j + 1],readNum[j + 3],readNum[j + 5]} ,
											{readNum[j + 7],readNum[j + 9],readNum[j + 11]} } };
#endif
#ifdef TEST_TRIGON3XY_TRIGON3XY
			randData3xy[i * length + j] = { { {readNum[j + 0],readNum[j + 2],0.0} ,
											{readNum[j + 6],readNum[j + 8],0.0} ,
											{readNum[j + 12],readNum[j + 14],0.0} } };
			randData3_xy[i * length + j] = { { {readNum[j + 1],readNum[j + 3],0.0} ,
											{readNum[j + 7],readNum[j + 9],0.0} ,
											{readNum[j + 13],readNum[j + 15],0.0} } };
#endif
		}
	}
	end = clock(); //completed
	cout << "data load finished, cost time = " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
	int countDiff = 0;
	for (int j = 0; j < 3; j++)
	{
		start = clock();
		//totalNum = 1;
		int inter_count = 0;
//#pragma omp parallel for //开启omp优化
		for (int i = 0; i < totalNum; i++)
		{
			//auto res = _get_circumcircle_center({ _get_rand() ,_get_rand() ,_get_rand() });
			//double res = _test_custom_calculate(nums);
			// 点与三角形
			//bool res = isPointInTriangle(randData2D[i][0], randData2D_[i]); //2D
			//bool res = isPointInTriangle(randData3[i][0], randData3_[i]); //3D
			//bool res = isPointOnTriangleSurface(Vector3d(1,2,3), randData3[i]);
			// 
			// 线段
			//bool res = isTwoSegmentsIntersect(randData2[i], randData2_[i]);
			// 
			// 三角形相交测试
			//bool res = isTwoTrianglesIntersectSAT(randData3[i], randData3_[i]); //9.435s
			//bool res1 = isTwoTrianglesIntrusionSAT(randData3[i], randData3_[i]);//14.859s
			//if (res)
			//	continue;
			//bool res = isTwoTrianglesIntrusionSAT_vector(randData3[i], randData3_[i], randData3N[i], randData3N_[i], 0.0);//
			bool res = isTwoTrianglesIntrusionSAT_array(randData3[i], randData3_[i], randData3N[i], randData3N_[i], 0.0);//
			//bool res1 = isTwoTrianglesIntersectSAT_17(randData3[i], randData3_[i], randData3N[i], randData3N_[i]);//

#pragma omp critical
			{
				//if (res)
				//	inter_count++;
     //           if (res != res1)
					//countDiff++;
			}
			//bool res = isSegmentCrossTriangleSurface(randData2[i], randData3_[i]);
			//bool res = TriangularIntersectionTest(randData3[i], randData3_[i]);
			//包围盒
			//bool res = isTriangleAndBoundingBoxIntersect(randData3_[i], { randData2[i][0] ,randData2[i][1]});
			//bool res = isTriangleAndBoundingBoxIntersectSAT(randData3_[i], { randData2[i][0] ,randData2[i][1]});
			//射线法
			//bool res = isPointRayAcrossTriangle(randData1[i][0], randData3[i]);

			//包围圆
			//auto crA = getTriangleBoundingCircle(randData3[i]);
			//auto crB = getTriangleBoundingCircle(randData3_[i]);
			//bool res = (std::get<0>(crA) - std::get<0>(crB)).norm() > std::get<1>(crA) + std::get<1>(crB);
			//bool res = isTwoTrianglesBoundingBoxIntersect(randData3[i], randData3_[i]);
			//
			// 软碰撞
			//double d = getTrianglesDistance(P, Q, randData3[i], randData3_[i]);
			//double d = getTrianglesDistanceSAT(randData3[i], randData3_[i]); //60s about, 7s(omp)
			//double dn = getTrianglesDistanceSAT(randData3[i], randData3_[i], randData3N[i], randData3N_[i]); //47s, 5.5s(omp)

			//array<Vector3d, 2> near2 = getTwoTrianglesNearestPoints(randData3[i], randData3_[i]);
			//array<Vector3d, 2> res = getTwoTrianglesIntersectPoints(randData3[i], randData3_[i]);
			//array<Vector3d, 2> res = getTwoTrianglesIntersectPoints(randData3xy[i], randData3_xy[i]);
			//double d = (res[1] - res[0]).norm(); << endl
			//double dnorm = (near2[1] - near2[0]).norm();
			//测试包围盒 
			//Eigen::AlignedBox3d res = Eigen::AlignedBox3d(randData2[i][0], randData2[i][1]).intersection(Eigen::AlignedBox3d(randData2_[i][0], randData2_[i][1]));
#pragma omp critical
			{
				//if (1e-8 < fabs(d - dn) || 1e-8 < fabs(dnorm - dn))
				//	countDiff++;
			}
			//calculate
			//double dotPro = (randData3[i][1].dot(randData3[i][2] - randData3_[i][0])) * (randData3[i][1].dot(randData3[i][0] - randData3_[i][0])) < eps;
			//Vector3d point = randData3[i][0] + (randData3[i][1].dot(randData3[i][0] - randData3_[i][0]) / dotPro) * (randData3[i][1] - randData3[i][0]);
			//bool res = isPointInTriangle(point, randData3[i]);

		}
		end = clock();
		cout << "time = " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
		cout << "countDiff=" << countDiff << endl;
		cout << "inter_count=" << inter_count << endl;
        cout << "not inter_count=" << totalNum - inter_count << endl;
		Sleep(100); //ms

	}

#endif

	// 输出当前时间
	time_t nowtime;
	time(&nowtime); //获取1970年1月1日0点0分0秒到现在经过的秒数
	tm p;
	localtime_s(&p, &nowtime); //将秒数转换为本地时间,年从1900算起,需要+1900,月为0-11,所以要+1
	printf("%04d/%02d/%02d-%02d:%02d:%02d \n", p.tm_year + 1900, p.tm_mon + 1, p.tm_mday, p.tm_hour, p.tm_min, p.tm_sec);
	cout << "main over." << endl;
}

//测试 运算符 速度
static void _test1()
{
	//create
	double* num = new double[totalNum];
	double* deno = new double[totalNum];
	clock_t start, end;
	start = clock();
	for (int i = 0; i < totalNum; i++)
	{
		num[i] = rand() + double(rand()) / RAND_MAX;
		deno[i] = rand() + double(rand()) / RAND_MAX;
	}
	end = clock();
	cout << "create_time = " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;

	//test
	double* res = new double[totalNum];
	double* resR = new double[totalNum];
	start = clock();
	for (int j = 0; j < 3; j++)
	{
		for (int i = 0; i < totalNum; i++)
		{
			//res[i] = num[i] / deno[i]; //0.5s
   //         resR[i] = deno[i] / num[i];
            res[i] = 1.0 / deno[i] * num[i]; //用时几乎一样的
            resR[i] = 1.0 / num[i] * deno[i];
		}
	}
	end = clock();
	cout << "calculate_time = " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
	Sleep(100); //ms
}

static void _test2()
{
	Triangle3d triA = { createRandVector(),createRandVector(),createRandVector() };
	Triangle3d triB = { createRandVector(),createRandVector(),createRandVector() };
	Vector3d normalA = (triA[1] - triA[0]).cross(triA[2] - triA[0]).normalized();
	Vector3d normalB = (triB[1] - triB[0]).cross(triB[2] - triB[0]).normalized();
	//bool res = isTwoTrianglesIntrusionSAT_c(triA, triB, normalA, normalB);
	return;
}

static int enrol = []()->int
{
	_test0();
	//_test1();
	//_test2();
	cout << "test_fun_speed finished.\n" << endl;
	return 0;
}();

