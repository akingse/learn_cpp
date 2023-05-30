#include "pch.h"
#include <ctime>
#include <time.h>
#include <omp.h>

using namespace para;
using Eigen::Vector3d;


Vec2 _get_rand()
{
	double r = 100;
	return Vec2(rand(), rand());
}

std::array<BPParaVec, 3> _get_rand3v()
{
	// rand -16384 -> 16384 
	//srand((int)time(0));
	//Sleep(100);
	return std::array<BPParaVec, 3> {
			BPParaVec(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
			BPParaVec(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
			BPParaVec(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff) };
}
std::array<Vector3d, 3> _get_rand3()
{
	// rand -16384 -> 16384 
	//srand((int)time(0));
	//Sleep(100);
	return std::array<Vector3d, 3> {
		Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
			Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
			Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff) };
}

int main()
{

	clock_t start, end;
	//Sleep(1000);
	double nums[10] = { rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand() };
	//for (int i = 0; i < sizeof(nums)/sizeof(double); i++)
	//	cout << nums[i] << " | ";
	psykronix::Vertex vec(1, 1, 1);
	psykronix::Vertex vec2 = vec;

//#pragma omp parallel for //开启omp优化
	for (int i = 0; i < 3; i++)
	{
		start = clock();
		for (int i = 0; i < int(1e4); i++) //1time = 1.429s, python spent 30s
		{
			//auto res = _get_circumcircle_center({ _get_rand() ,_get_rand() ,_get_rand() });
			//double res = _test_custom_calculate(nums);
			//bool res = isTwoTrianglesIntersection(tA, tB); //without rand//debug=1e7=9.6s,release=1e7=1.8s,
			bool res = isTwoTrianglesIntersection(_get_rand3(), _get_rand3()); //debug=1e7=16.5s,release=1e7=5.1s,omp=0.85
			//bool res = TriangularIntersectionTest(_get_rand3(), _get_rand3());
		}
		end = clock();   //结束时间
		cout << "time = " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
		Sleep(1000);
	}


	return 0;
}




static int enrol = []()->int
{
	return 0;
}();

