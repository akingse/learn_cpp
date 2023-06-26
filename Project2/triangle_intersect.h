#pragma once

typedef std::tuple<char, int, double> TC;
namespace std
{
	//template <class T,class U,class V>
	//class tuple<T, U, V>
	//{
	//public:
	//	T operator[](int i)
	//	{

	//	}
	//};
}
static std::string randNumName = "random_1e8.bin";

inline std::string getExePath() // include<afx.h>
{
	TCHAR buff[MAX_PATH];
	GetModuleFileNameW(NULL, buff, MAX_PATH);
	CString path = buff;
	path = path.Left(path.ReverseFind('\\')); // delete exename
	return (CStringA)path;
}

namespace std
{
	//wirte randnum file
	inline int _wirteNumberFile(size_t n)
	{
		n = 2 * 3 * 3 * n;
		double* arr = new double[n];
		for (int i = 0; i < n; ++i)
		{
			arr[i] = static_cast<double>(rand() - 0x3fff);// rand()) / RAND_MAX;
		}
		// 写入文件
		ofstream out(randNumName, ios::out | ios::binary);
		if (!out.is_open()) {
			cerr << "Error opening file" << endl;
			return -1;
		}
		out.write(reinterpret_cast<char*>(&n), sizeof(int)); // 先写入整个数组大小 
		out.write(reinterpret_cast<char*>(arr), n * sizeof(double));
		out.close();
		return 0;
	}

	inline double* _readNumberFile(size_t n)
	{
		ifstream in(randNumName, ios::in | ios::binary);
		if (!in.is_open()) {
			//cerr << "Error opening file" << endl;
			//return nullptr;
			_wirteNumberFile(n);
			in = ifstream(randNumName, ios::in | ios::binary);
			if (!in.is_open())
				return nullptr;
		}
		n = 2 * 3 * 3 * n;
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
}

inline Vec2 _get_rand()
{
	double r = 100;
	return Vec2(rand(), rand());
}

inline Eigen::Vector3d _to2D(const Eigen::Vector3d& vec3)
{
	return Eigen::Vector3d(vec3.x(), vec3.y(), 0.0);
}

inline std::array<para::BPParaVec, 3> _get_rand3v()
{
	// rand -16384 -> 16384 
	//srand((int)time(0));
	//Sleep(100);
	return std::array<para::BPParaVec, 3> {
		para::BPParaVec(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
			para::BPParaVec(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
			para::BPParaVec(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff) };
}
inline std::array<Eigen::Vector3d, 3> _get_rand3(int i = 0)
{
	if (i == 0)
		return std::array<Eigen::Vector3d, 3> {
		Eigen::Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
			Eigen::Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
			Eigen::Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff) };
	if (i == 1)
		return std::array<Eigen::Vector3d, 3> {
		Eigen::Vector3d(rand(), rand(), rand()),
			Eigen::Vector3d(rand(), rand(), rand()),
			Eigen::Vector3d(rand(), rand(), rand()) };
	if (i == -1)
		return std::array<Eigen::Vector3d, 3> {
		Eigen::Vector3d(rand() - 0x7fff, rand() - 0x7fff, rand() - 0x7fff),
			Eigen::Vector3d(rand() - 0x7fff, rand() - 0x7fff, rand() - 0x7fff),
			Eigen::Vector3d(rand() - 0x7fff, rand() - 0x7fff, rand() - 0x7fff) };
}

inline std::array<Eigen::Vector3d, 2> _get_rand2()
{
	return std::array<Eigen::Vector3d, 2> {
		Eigen::Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
			Eigen::Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff) };
}


static void _test()
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
}