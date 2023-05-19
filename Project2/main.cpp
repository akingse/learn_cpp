#include "pch.h"
#include <ctime>
#include <time.h>
#include <omp.h>

/// <summary>
/// Project2
/// </summary>
/// <returns></returns>

int _get_rand_if(int i)
{
	if (i == 1)
		return 1;
	else if (i == 2)
		return 1;

}


Vec2 _get_rand()
{
	double r = 100;
	return Vec2(rand(), rand());
}

int main()
{
	_get_rand_if(3);
	clock_t start, end;
	start = clock();
	//Sleep(1000);
	double nums[10] = { rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand(), rand() };
	for (int i = 0; i < sizeof(nums)/sizeof(double); i++)
		cout << nums[i] << " | ";

	std::array<BPParaVec, 3> tA = { BPParaVec(rand(), rand(), rand()), BPParaVec(rand(), rand(), rand()), BPParaVec(rand(), rand(), rand()) };
	std::array<BPParaVec, 3> tB = { BPParaVec(rand(), rand(), rand()), BPParaVec(rand(), rand(), rand()), BPParaVec(rand(), rand(), rand()) };
//#pragma omp parallel for //开启omp优化
	for (int i = 0; i < int(1e6); i++) //1time = 1.429s, python spent 30s
	{
		//auto res = _get_circumcircle_center({ _get_rand() ,_get_rand() ,_get_rand() });
		//double res = _test_custom_calculate(nums);
		bool res = _isTwoTriangularIntersection(tA, tB);
	}

	end = clock();   //结束时间
	cout << "time = " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;


	return 0;
}




static int enrol = []()->int
{
	return 0;
}();

