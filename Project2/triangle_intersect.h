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
		n = 2 * 3 * 3 * n;
		ifstream in(randNumName, ios::in | ios::binary);
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
