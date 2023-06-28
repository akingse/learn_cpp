#include "pch.h"
using namespace psykronix;
using namespace std;
using namespace para;
using namespace Eigen;
using namespace psykronix;
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
int main()
{
	auto dir = getExePath();
	cout << "exepath=" << dir << endl;
	clock_t start, end;

	Eigen::Vector3d P(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
	Eigen::Vector3d Q(std::nan("0"), std::nan("0"), std::nan("0"));
	double pi = M_PI;

	start = clock();
	cout << "data number:"<< totalNum <<", load start..." << endl;
	double* readNum = _readNumberFile(size_t(sqrt(totalNum)));
	// the data
	static std::array<Vector3d, 2>* randData2 = new std::array<Vector3d, 2>[totalNum];
	//static std::array<Vector3d, 2>* randData2_ = new std::array<Vector3d, 2>[totalNum];
	//std::array<Vector3d, 3>* randData3 = new std::array<Vector3d, 3>[totalNum];
	std::array<Vector3d, 3>* randData3_ = new std::array<Vector3d, 3>[totalNum];
	//static std::array<Vector3d, 3>* randData30 = new std::array<Vector3d, 3>[totalNum];
	//static std::array<Vector3d, 3>* randData30_ = new std::array<Vector3d, 3>[totalNum];
	//for (size_t i = 0; i < totalNum; ++i)
	//{
	//	randData3[i] = _get_rand3();
	//	randData3_[i] = _get_rand3();
	//}



#ifdef TEST_SPEED_OF_FUNCTION
	size_t length = size_t(sqrt(totalNum));// (1e4);
	for (int i = 0; i < length; ++i)
	{
		for (int j = 0; j < length; ++j)
		{
			//randData3[i * length + j] = { { {readNum[j + 0],readNum[j + 2],readNum[j + 4]} ,
			//								{readNum[j + 6],readNum[j + 8],readNum[j + 10]} ,
			//								{readNum[j + 12],readNum[j + 14],readNum[j + 16]} } };
			randData3_[i * length + j] = { { {readNum[j + 1],readNum[j + 3],readNum[j + 5]} ,
											{readNum[j + 7],readNum[j + 9],readNum[j + 11]} ,
											{readNum[j + 13],readNum[j + 15],readNum[j + 17]} } };

			randData2[i * length + j] = { { {readNum[j + 0],readNum[j + 2],readNum[j + 4]} ,
											{readNum[j + 6],readNum[j + 8],readNum[j + 10]} } };
			//randData2_[i * length + j] = { { {readNum[j + 1],readNum[j + 3],readNum[j + 5]} ,
			//								{readNum[j + 7],readNum[j + 9],readNum[j + 11]} } };
			
			//randData30[i * length + j] = { { {readNum[j + 0],readNum[j + 2],0.0} ,
			//								{readNum[j + 6],readNum[j + 8],0.0} ,
			//								{readNum[j + 12],readNum[j + 14],0.0} } };
			//randData30_[i * length + j] = { { {readNum[j + 1],readNum[j + 3],0.0} ,
			//								{readNum[j + 7],readNum[j + 9],0.0} ,
			//								{readNum[j + 13],readNum[j + 15],0.0} } };
		}
	}
	end = clock();
	cout << "data load finish, cost time = " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;

//#pragma omp parallel for //开启omp优化
	for (int i = 0; i < 3; i++)
	{
		start = clock();
		//totalNum = 1;
		for (int i = 0; i < totalNum; i++)
		{
			//auto res = _get_circumcircle_center({ _get_rand() ,_get_rand() ,_get_rand() });
			//double res = _test_custom_calculate(nums);
			// 三角形相交测试
			//bool res = isTwoTrianglesIntersection(randData3[i], randData3_[i]);
			//bool res = isTwoTrianglesIntersection2(randData3[i], randData3_[i]);
			//bool res = isTwoTrianglesIntersectionSAT(randData3[i], randData3_[i]);
			//bool res = isTwoTrianglesIntersection2(randData3[i], randData3_[i]);
			//bool r1 = isSegmentCrossTriangleSurface(_get_rand2(), _get_rand3());
			//bool r2 = isSegmentCrossTriangleSurface(_get_rand2(), _get_rand3());
			//bool res = isPointInTriangle(randData3[i][0], randData3_[i]);
			//bool res = isSegmentCrossTriangleSurface(randData2[i], randData3_[i]);
			//bool res = TriangularIntersectionTest(randData3[i], randData3_[i]);
			//包围盒
			//bool res = isTriangleAndBoundingBoxIntersect(randData3_[i], { randData2[i][0] ,randData2[i][1]});
			bool res = isTriangleAndBoundingBoxIntersectSAT(randData3_[i], { randData2[i][0] ,randData2[i][1]});

			//包围圆
			//auto crA = getTriangleBoundingCircle(randData3[i]);
			//auto crB = getTriangleBoundingCircle(randData3_[i]);
			//bool res = (std::get<0>(crA) - std::get<0>(crB)).norm() > std::get<1>(crA) + std::get<1>(crB);
			//bool res = isTwoTrianglesBoundingBoxIntersect(randData3[i], randData3_[i]);
			//
			// 软碰撞
			//double d = getTrianglesDistance(P, Q, randData3[i], randData3_[i]);
			//测试包围盒 
			//Eigen::AlignedBox3d res = Eigen::AlignedBox3d(randData2[i][0], randData2[i][1]).intersection(Eigen::AlignedBox3d(randData2_[i][0], randData2_[i][1]));
			
			//calcul
			//double dotPro = (randData3[i][1].dot(randData3[i][2] - randData3_[i][0])) * (randData3[i][1].dot(randData3[i][0] - randData3_[i][0])) < eps;
			//Vector3d point = randData3[i][0] + (randData3[i][1].dot(randData3[i][0] - randData3_[i][0]) / dotPro) * (randData3[i][1] - randData3[i][0]);
			//bool res = isPointInTriangle(point, randData3[i]);

		}
		end = clock(); 
		cout << "time = " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
		Sleep(1000);

	}
	cout << "TriangularIntersectC=" << TriangularIntersectC / 3 << endl;
	cout << "count_edgeCrossTri=" << count_edgeCrossTri / 3 << endl;
	cout << "count_pointInTri=" << count_pointInTri / 3 << endl;
	cout << "count_segCrossTri=" << count_segCrossTri / 3 << endl;
	cout << "count_across=" << count_across / 3 << endl;
	cout << "TriangularIntersectC_in=" << TriangularIntersectC_in / 3 << endl;
	cout << "TriangularIntersectC_all=" << TriangularIntersectC_all / 3 << endl;
	TriangularIntersectC = 0;
	count_edgeCrossTri = 0;
	count_pointInTri = 0;
	count_segCrossTri = 0;
	count_across = 0;
	TriangularIntersectC_in = 0;
	TriangularIntersectC_all = 0;
#endif

	return 0;
}


static int enrol = []()->int
{
	return 0;
}();

