#include "pch.h"
#include <ctime>
#include <time.h>

/// <summary>
/// Project2
/// </summary>
/// <returns></returns>


Vec2 _get_rand()
{
	double r = 100;
	return Vec2(rand(), rand());
}

int main()
{

	clock_t start, end;
	start = clock();
	//Sleep(1000);
//#pragma omp parallel for //开启omp优化
	for (int i = 0; i < int(1e7); i++) //1time = 1.429s, python spent 30s
	{
		auto res = _get_circumcircle_center({ _get_rand() ,_get_rand() ,_get_rand() });
	}

	end = clock();   //结束时间
	cout << "time = " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;


	return 0;
}




static int enrol = []()->int
{
	return 0;
}();

