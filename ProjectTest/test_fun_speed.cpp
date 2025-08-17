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

static void _test0()
{
	clock_t start, end;
	string dir = "C:/Users/Aking/source/repos/learn_cpp/ProjectTest/";//"getExePath();
	cout << "exepath=" << dir << endl;

#ifdef TEST_SPEED_OF_FUNCTION
	start = clock();
	cout << "data number:" << totalNum << ", load start..." << endl;
	// the data
    double* readNum = _readNumberFile(size_t(sqrt(totalNum)), dir + randNumName);

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
	for (int i = 0; i < 3; i++)
	{
		start = clock();
		//totalNum = 1;
#pragma omp parallel for //开启omp优化
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
			//bool res = isTwoTrianglesIntersectSAT(randData3[i], randData3_[i]);
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
			double d = getTrianglesDistanceSAT(randData3[i], randData3_[i]);
			Vector3d normalA = (randData3[i][1] - randData3[i][0]).cross(randData3[i][2] - randData3[i][1]).normalized();
			Vector3d normalB = (randData3_[i][1] - randData3_[i][0]).cross(randData3_[i][2] - randData3_[i][1]).normalized();
			double dn = getTrianglesDistanceSAT(randData3[i], normalA, randData3_[i], normalB);
			//array<Vector3d, 2> res = getTwoTrianglesNearestPoints(randData3[i], randData3_[i]);
			//array<Vector3d, 2> res = getTwoTrianglesIntersectPoints(randData3[i], randData3_[i]);
			//array<Vector3d, 2> res = getTwoTrianglesIntersectPoints(randData3xy[i], randData3_xy[i]);
			//double d = (res[1] - res[0]).norm(); << endl
			//测试包围盒 
			//Eigen::AlignedBox3d res = Eigen::AlignedBox3d(randData2[i][0], randData2[i][1]).intersection(Eigen::AlignedBox3d(randData2_[i][0], randData2_[i][1]));

			//calculate
			//double dotPro = (randData3[i][1].dot(randData3[i][2] - randData3_[i][0])) * (randData3[i][1].dot(randData3[i][0] - randData3_[i][0])) < eps;
			//Vector3d point = randData3[i][0] + (randData3[i][1].dot(randData3[i][0] - randData3_[i][0]) / dotPro) * (randData3[i][1] - randData3[i][0]);
			//bool res = isPointInTriangle(point, randData3[i]);

		}
		end = clock();
		cout << "time = " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
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

static int enrol = []()->int
{
	_test0();
	return 0;
}();

