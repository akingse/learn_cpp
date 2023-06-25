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

inline std::string getExePath() // include<afx.h>
{
	TCHAR buff[MAX_PATH];
	GetModuleFileNameW(NULL, buff, MAX_PATH);
	CString path = buff;
	path = path.Left(path.ReverseFind('\\')); // delete exename
	return (CStringA)path;
}

inline Vec2 _get_rand()
{
	double r = 100;
	return Vec2(rand(), rand());
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
