#include "pch.h"
using namespace std; //有作用域
using namespace para;
using namespace Eigen;
using namespace psykronix;
#undef max //AlignedBox3d mem-fun
#undef min
//精度
static constexpr float eps = 1e-6;
static constexpr float epsF = FLT_EPSILON;
static constexpr double epsD = DBL_EPSILON;
extern std::atomic<size_t>  count_pointInTri, count_edgeCrossTri, TriangularIntersectC;
auto eps1 = std::numeric_limits<float>::epsilon();
auto eps2 = std::numeric_limits<double>::epsilon();


//#define RSS_TYPE     1
//#define OBB_TYPE     2
constexpr int RSS_TYPE = 1;
constexpr int OBB_TYPE = 2;
// abcdefghijklmnopqrstuvwxyz
// ABCDEFGHIJKLMNOPQRSTUVWXYZ

size_t count1 = 0;
size_t count2 = 0;
size_t totalNum = (size_t)1e8;

// //数组的总大小不能超过0x7fffffff个字节 2G
//static std::array<std::array<Vector3d, 2>,totalNum>* randData2_a = new std::array<std::array<Vector3d, 2>, totalNum>;
//std::vector<std::array<Vector3d, 2>>* randData2 = new std::vector<std::array<Vector3d, 2>>; 
//std::array<std::array<Vector3d, 2>, totalNum>* randData2_ = new std::array<std::array<Vector3d, 2>, totalNum>;
//std::array<std::array<Vector3d, 3>, totalNum>* randData3 = new std::array<std::array<Vector3d, 3>, totalNum>; //with max size 2e7
//std::array<std::array<Vector3d, 3>, totalNum>* randData3_ = new std::array<std::array<Vector3d, 3>, totalNum>;

int main()
{
	array<array<double, 3>, 3> triangle = { { {0,0,0},{1,1,1}, {2,2,2}} };
	auto dir = getExePath();
	//_wirteNumberFile(size_t(1e4));
	cout << "exepath=" << dir << endl;
	int PQP_BV_TYPE = RSS_TYPE | OBB_TYPE;
	int macro1 = PQP_BV_TYPE & RSS_TYPE;
	int macro2 = PQP_BV_TYPE & OBB_TYPE;
	tuple<char, int, double> mytp;
	mytp = { 'a',1,2.0 };

	Eigen::Vector3d P(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
	Eigen::Vector3d Q(std::nan("0"), std::nan("0"), std::nan("0"));

	//bool isTI = TriangleIntersectionTest({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	//double td = getTrianglesDistance(P, Q, { triA0,triA1,triA2 }, { triB0,triB1,triB2 }); //43.905043670162655
	double a = 3;
	float b = 3;
	double c = a - b;
	double d = max(a, c);

	clock_t start, end;
	//Sleep(1000);
	double nums[10] = { rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand() };
	//for (int i = 0; i < sizeof(nums)/sizeof(double); i++)
	//	cout << nums[i] << " | ";
	psykronix::Vertex vec(1, 1, 1);
	psykronix::Vertex vec2 = vec;
	

	start = clock();
	cout << "data number:"<< totalNum <<", load start..." << endl;
	double* readNum = _readNumberFile(size_t(sqrt(totalNum)));
	// the data
	//static std::array<Vector3d, 2>* randData2 = new std::array<Vector3d, 2>[totalNum];
	//static std::array<Vector3d, 2>* randData2_ = new std::array<Vector3d, 2>[totalNum];
	static std::array<Vector3d, 3>* randData3 = new std::array<Vector3d, 3>[totalNum];
	static std::array<Vector3d, 3>* randData3_ = new std::array<Vector3d, 3>[totalNum];
	//static std::array<Vector3d, 3>* randData30 = new std::array<Vector3d, 3>[totalNum];
	//static std::array<Vector3d, 3>* randData30_ = new std::array<Vector3d, 3>[totalNum];
	size_t length = size_t(sqrt(totalNum));// (1e4);
	for (int i = 0; i < length; ++i)
	{
		for (int j = 0; j < length; ++j)
		{
			randData3[i * length + j] = { { {readNum[j + 0],readNum[j + 2],readNum[j + 4]} ,
											{readNum[j + 6],readNum[j + 8],readNum[j + 10]} ,
											{readNum[j + 12],readNum[j + 14],readNum[j + 16]} } };
			randData3_[i * length + j] = { { {readNum[j + 1],readNum[j + 3],readNum[j + 5]} ,
											{readNum[j + 7],readNum[j + 9],readNum[j + 11]} ,
											{readNum[j + 13],readNum[j + 15],readNum[j + 17]} } };
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
			bool res = TriangleIntersectionTest(randData3[i], randData3_[i]);
			//bool res = isTwoTrianglesIntersection2(randData3[i], randData3_[i]);
			//bool r1 = _isSegmentCrossTriangleSurface(_get_rand2(), _get_rand3());
			//bool r2 = _isSegmentCrossTriangleSurface(_get_rand2(), _get_rand3());
			//bool res = _isPointInTriangle(randData3[i][0], randData3_[i]);
			//bool res = TriangularIntersectionTest(randData3[i], randData3_[i]);
			//包围圆
			//auto crA = getTriangleBoundingCircle(randData3[i]);
			//auto crB = getTriangleBoundingCircle(randData3_[i]);
			//bool res = (std::get<0>(crA) - std::get<0>(crB)).norm() > std::get<1>(crA) + std::get<1>(crB);
			//bool res = isTwoTrianglesBoundingBoxIntersect(randData3[i], randData3_[i], 1);
			//
			// 软碰撞
			//double d = getTrianglesDistance(P, Q, randData3[i], randData3_[i]);
			//测试包围盒 
			//Eigen::AlignedBox3d res = Eigen::AlignedBox3d(randData2[i][0], randData2[i][1]).intersection(Eigen::AlignedBox3d(randData2_[i][0], randData2_[i][1]));
				
		}
		end = clock(); 
		cout << "time = " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
		Sleep(1000);
		//cout << "TriangularIntersectC=" << TriangularIntersectC << endl;
		//cout << "count_edgeCrossTri=" << count_edgeCrossTri << endl;
		//cout << "count_pointInTri=" << count_pointInTri << endl;
		TriangularIntersectC = 0;
		count_edgeCrossTri = 0;
		count_pointInTri = 0;
	}
	cout << count1 /3 << endl;
	cout << count2 /3 << endl;

	return 0;
}


static int enrol = []()->int
{
	return 0;
}();

