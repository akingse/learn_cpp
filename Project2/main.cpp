#include "pch.h"
using namespace std; //有作用域
using namespace para;
using namespace Eigen;
using namespace psykronix;

//精度
static const float eps = 1e-6;
static const float epsF = FLT_EPSILON;
static const double epsD = DBL_EPSILON;

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
const size_t totalNum = (size_t)1e8;


// //数组的总大小不能超过0x7fffffff个字节 2G
//static std::array<std::array<Vector3d, 2>,totalNum>* randData2_a = new std::array<std::array<Vector3d, 2>, totalNum>;
//std::vector<std::array<Vector3d, 2>>* randData2 = new std::vector<std::array<Vector3d, 2>>; 
//std::array<std::array<Vector3d, 2>, totalNum>* randData2_ = new std::array<std::array<Vector3d, 2>, totalNum>;
//std::array<std::array<Vector3d, 3>, totalNum>* randData3 = new std::array<std::array<Vector3d, 3>, totalNum>; //with max size 2e7
//std::array<std::array<Vector3d, 3>, totalNum>* randData3_ = new std::array<std::array<Vector3d, 3>, totalNum>;

int main()
{
	auto dir = getExePath();
	cout << "exepath=" << dir << endl;
	int PQP_BV_TYPE = RSS_TYPE | OBB_TYPE;
	int macro1 = PQP_BV_TYPE & RSS_TYPE;
	int macro2 = PQP_BV_TYPE & OBB_TYPE;

	tuple<char, int, double> mytp;
	mytp = { 'a',1,2.0 };

	Eigen::Vector3d P(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
	Eigen::Vector3d Q(std::nan("0"), std::nan("0"), std::nan("0"));

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
	//std::array<Vector3d, 3> trigon = { Vertex(-87, 21), Vertex(-27, -20), Vertex(-90, 95) };
	std::array<Vector3d, 3> trigon = { Vertex(-10, 0), Vertex(70, 0), Vertex(50, 50) };
	auto res = getTriangleBoundingCircle(trigon);
	Eigen::AlignedBox3d box(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
	bool isC = box.contains(Eigen::Vector3d(0.5, 0.5, 0.5));

	std::array<Vector3d, 3> trigonA = { Vertex(-10, -10), Vertex(10, -10), Vertex(0, 10) };
	std::array<Vector3d, 3> trigonB = psykronix::scale(2,2,2) * trigonA;
	isTwoTrianglesIntersection1(trigonA, trigonA);

	start = clock();
	//static std::array<Vector3d, 2>* randData2 = new std::array<Vector3d, 2>[totalNum];
	//static std::array<Vector3d, 2>* randData2_ = new std::array<Vector3d, 2>[totalNum];
	static std::array<Vector3d, 3>* randData3 = new std::array<Vector3d, 3>[totalNum];
	static std::array<Vector3d, 3>* randData3_ = new std::array<Vector3d, 3>[totalNum];
	for (int i = 0; i < totalNum; i++)
	{
		//randData2[i] = _get_rand2();
		//randData2_[i] = _get_rand2();
		randData3[i] = _get_rand3();
		randData3_[i] = _get_rand3();
		// 
		//randData2->at(i) = _get_rand2();
		//randData2_->at(i) = _get_rand2();
		//randData3->at(i) = _get_rand3();
		//randData3_->at(i) = _get_rand3();
	}
	end = clock();
	cout << "data load finish, cost time = " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;

//#pragma omp parallel for //开启omp优化
	for (int i = 0; i < 3; i++)
	{
		start = clock();
		for (int i = 0; i < 1; i++) //1time = 1.429s, python spent 30s
		{
			//auto res = _get_circumcircle_center({ _get_rand() ,_get_rand() ,_get_rand() });
			//double res = _test_custom_calculate(nums);
			// 三角形相交测试
			//bool res = isTwoTrianglesIntersection1(randData3[i], randData3_[i]); //debug=1e7=16.5s,release=1e7=5.1s,omp=0.85
			//bool res = isTwoTrianglesIntersection2(randData3[i], randData3_[i]); //without rand//debug=1e7=9.6s,release=1e7=1.8s,
			//bool r1 = _isSegmentCrossTriangleSurface(_get_rand2(), _get_rand3());
			//bool r2 = _isSegmentCrossTriangleSurface(_get_rand2(), _get_rand3());
			//bool res = _isPointInTriangle(Vector3d(0, 0, 0), _get_rand3());
			//bool res = TriangularIntersectionTest(randData3[i], randData3_[i]);
			//包围圆
			auto crA = getTriangleBoundingCircle(randData3[i]);
			auto crB = getTriangleBoundingCircle(randData3_[i]);
			bool res = (std::get<0>(crA) - std::get<0>(crB)).norm() > std::get<1>(crA) + std::get<1>(crB);
			// 软碰撞
			//double d = _getTriDist(P, Q, randData3[i], randData3_[i]);
			//测试包围盒 
			//Eigen::AlignedBox3d res = Eigen::AlignedBox3d(randData2[i][0], randData2[i][1]).intersection(Eigen::AlignedBox3d(randData2_[i][0], randData2_[i][1]));
				
		}
		end = clock(); 
		cout << "time = " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
		Sleep(1000);
	}
	cout << count1 /3 << endl;
	cout << count2 /3 << endl;

	return 0;
}


static int enrol = []()->int
{
	return 0;
}();

