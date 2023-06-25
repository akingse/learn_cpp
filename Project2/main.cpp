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

//wirte randnum file
int _wirteNumberFile(size_t n)
{
	n = 2 * 3 * 3 * n;
	double* arr = new double[n];
	for (int i = 0; i < n; ++i) 
	{
		arr[i] = static_cast<double>(rand() - 0x3fff);// rand()) / RAND_MAX;
	}
	// 写入文件
	ofstream out("random_1e8.bin", ios::out | ios::binary);
	if (!out.is_open()) {
		cerr << "Error opening file" << endl;
		return -1;
	}
	out.write(reinterpret_cast<char*>(&n), sizeof(int)); // 先写入整个数组大小 
	out.write(reinterpret_cast<char*>(arr), n * sizeof(double));
	out.close();
	return 0;
}

double* _readNumberFile(size_t n)
{
	n = 2 * 3 * 3 * n;
	ifstream in("random_1e8.bin", ios::in | ios::binary);
	if (!in.is_open()) {
		cerr << "Error opening file" << endl;
		return nullptr;
	}
	int read_n;
	in.read(reinterpret_cast<char*>(&read_n), sizeof(int));
	if (read_n != n) {
		cerr << "Incorrect data size read from the input file" << endl;
		return nullptr;
	}
	double* read_arr = new double[read_n];
	in.read(reinterpret_cast<char*>(read_arr), read_n * sizeof(double));
	return read_arr;
}


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

	//bug测试
	Vector3d triA0 = Vector3d(4928967.0000119563, -378740.42555394967, 5318.0000000002756);
	Vector3d triA1 = Vector3d(4928973.9922683481, -378747.41781034082, 5319.5661914788307);
	Vector3d triA2 = Vector3d(4928989.6274289545, -378763.05297094764, 5350.0000000002756);
	Vector3d triB0 = Vector3d(4928967.1581119057, -378706.64252840198, 5357.7254248593727);
	Vector3d triB1 = Vector3d(4928961.6339804111, -378710.43624643586, 5350.0000000000000);
	Vector3d triB2 = Vector3d(4928962.4991871417, -378711.30145316647, 5357.7254248593645);

	Vector3d triA_0 = Vector3d(4924500.816042, -384858.485537, 5350.000000);
	Vector3d triA_1 = Vector3d(4924500.816042, -384856.898474, 5364.085575);
	Vector3d triA_2 = Vector3d(4924500.816042, -384852.216866, 5377.464841);
	Vector3d triB_0 = Vector3d(4924394.810992, -384838.186268, 5384.291939);
	Vector3d triB_1 = Vector3d(4924500.816042, -384838.186268, 5384.291939);
	Vector3d triB_2 = Vector3d(4924500.816042, -384829.477476, 5393.000732);
	bool isTI = TriangleIntersectionTest({ triA_0, triA_1, triA_2 }, { triB_0, triB_1, triB_2 });
	

	double td = getTrianglesDistance(P, Q, { triA0,triA1,triA2 }, { triB0,triB1,triB2 }); //43.905043670162655
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
	Eigen::AlignedBox3d box_(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, -1));
	bool isC = box.contains(Eigen::Vector3d(0.5, 0.5, 0.5));

	std::array<Vector3d, 3> trigonA = { Vertex(-10, -10), Vertex(10, -10), Vertex(0, 10) };
	std::array<Vector3d, 3> trigonB = psykronix::scale(2,2,2) * trigonA;
	isTwoTrianglesIntersection(trigonA, trigonA);

	Eigen::AlignedBox3d box1(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
	Eigen::AlignedBox3d box2(Eigen::Vector3d(0.5, 0.5, 0.5), Eigen::Vector3d(2, 2, 2));
	Eigen::AlignedBox3d box3(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(2, 2, 2));
	Eigen::AlignedBox3d box4(Eigen::Vector3d(1.5, 1.5, 1.5), Eigen::Vector3d(2, 2, 2));

	bool ill0 = (box.diagonal().array() < 0).any();
	bool ill1 = (box_.diagonal().array() < 0).any();
	bool ill2 = ((box1.intersection(box4)).diagonal().array() < 0).any();

	// 判断两个包围盒是否相交
	bool isInter = box1.intersects(box4);
	Eigen::AlignedBox3d box11(Eigen::Vector3d(-0.5, -0.5, -0.5), Eigen::Vector3d(0.5, 0.5, 0.5));
	Eigen::AlignedBox3d box21(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.3, 0.3, 0.3));
	Eigen::AlignedBox3d box31(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.5, 0.5, 0.5));
	// 判断 box2 是否被包含在 box1 内
	bool is_contain0 = box11.contains(box21.min()) && box11.contains(box21.max());
	bool is_contain1 = box11.contains(box11.min()) && box11.contains(box11.max());
	bool is_contain3 = box11.contains(box31.min()) && box11.contains(box31.max());

	Eigen::AlignedBox3d box12(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
	Eigen::AlignedBox3d box22(Eigen::Vector3d(2, 2, 2), Eigen::Vector3d(3, 3, 3));
	// test compare
	double tolerance = 2;
	Eigen::Vector3d tolerance_double = Eigen::Vector3d(tolerance, tolerance, tolerance);
	AlignedBox3d bounding = box12.intersection(box22);
	AlignedBox3d boundingc = box22.intersection(box12);
	bool isSepa = ((bounding.min() - bounding.max() - tolerance_double).array() > 0).any();
	box12.min() = box12.min() - tolerance_double;
	box12.max() = box12.max() + tolerance_double;
	bool isSepa0 = !box12.intersects(box22);


	//Eigen::AlignedBox3d box22(Eigen::Vector3d(.2, .2, .2), Eigen::Vector3d(3, 3, 3));
	Eigen::AlignedBox3d boxInter = box12.intersection(box22);
	bool isSepa2 = ((boxInter.min() - boxInter.max()).array() > 0).any();
	bool isIter2 = box12.intersects(box22);

	start = clock();
	cout << "data number:"<< totalNum <<", load start..." << endl;
	double* readNum = _readNumberFile(size_t(1e4));

	//static std::array<Vector3d, 2>* randData2 = new std::array<Vector3d, 2>[totalNum];
	//static std::array<Vector3d, 2>* randData2_ = new std::array<Vector3d, 2>[totalNum];
	static std::array<Vector3d, 3>* randData3 = new std::array<Vector3d, 3>[totalNum];
	static std::array<Vector3d, 3>* randData3_ = new std::array<Vector3d, 3>[totalNum];
	size_t length = size_t(1e4);
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
		}
	}

	//for (int i = 0; i < totalNum; i++)
	//{
		//randData2[i] = _get_rand2();
		//randData2_[i] = _get_rand2();
		//randData3[i] = _get_rand3();
		//randData3_[i] = _get_rand3();
		//randData2->at(i) = _get_rand2();
		//randData2_->at(i) = _get_rand2();
		//randData3->at(i) = _get_rand3();
		//randData3_->at(i) = _get_rand3();
	//}
	end = clock();
	cout << "data load finish, cost time = " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;

//#pragma omp parallel for //开启omp优化
	for (int i = 0; i < 3; i++)
	{
		start = clock();
		totalNum = 1;
		for (int i = 0; i < totalNum; i++) //1time = 1.429s, python spent 30s
		{
			//auto res = _get_circumcircle_center({ _get_rand() ,_get_rand() ,_get_rand() });
			//double res = _test_custom_calculate(nums);
			// 三角形相交测试
			//bool res = isTwoTrianglesIntersection1(randData3[i], randData3_[i]); //debug=1e7=16.5s,release=1e7=5.1s,omp=0.85
			//bool res = isTwoTrianglesIntersection2(randData3[i], randData3_[i]); //without rand//debug=1e7=9.6s,release=1e7=1.8s,
			//bool r1 = _isSegmentCrossTriangleSurface(_get_rand2(), _get_rand3());
			//bool r2 = _isSegmentCrossTriangleSurface(_get_rand2(), _get_rand3());
			//bool res = _isPointInTriangle(randData3[i][0], randData3_[i]);
			//bool res = TriangularIntersectionTest(randData3[i], randData3_[i]);
			//包围圆
			//auto crA = getTriangleBoundingCircle(randData3[i]);
			//auto crB = getTriangleBoundingCircle(randData3_[i]);
			//bool res = (std::get<0>(crA) - std::get<0>(crB)).norm() > std::get<1>(crA) + std::get<1>(crB);
			//bool res = _isTwoTrianglesBoundingBoxIntersect(randData3[i], randData3_[i], 1);
			//
			// 软碰撞
			double d = getTrianglesDistance(P, Q, randData3[i], randData3_[i]);
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

