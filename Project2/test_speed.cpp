#include "pch.h"


static void _test1()
{
	//测试性能
//Debug差10倍，release差5倍
// 计时
	auto start = std::chrono::system_clock::now();

	//for (int i = 0; i < 1e6; i++)
	//{
	//	//随机数
	//	srand((unsigned)time(NULL));
	//	string rd = to_string(rand());
	//	string md0 = para::getMD5(rd);
	//	//MD5_32B md0 = para::getMD5_ULL(rd);
	//}


	//rand函数

	/*
	int num = rand() % 100; //产生0~99这100个整数中的一个随机整数
	rand() % (b-a+1)+ a ;    //表示  a~b 之间的一个随机整数。
	通常rand()产生的随机数在每次运行的时候都是与上一次相同的，这样是为了便于程序的调试。若要产生每次不同的随机数，则可以使用srand( seed )函数进行产生随机化种子，随着seed的不同，就能够产生不同的随机数。

	rand() 会返回一随机数值，范围在 0 至 RAND_MAX 间。
	rand()产生的是假随机数字，每次执行时是相同的。若要不同,以不同的值来初始化它.初始化的函数就是 srand()。
	*/
	int a = RAND_MAX; //32767 =2^(16-1)
	int rd0 = rand() - RAND_MAX / 2;
	int rd = rand() - 0x3fff;


	srand((int)time(0));

	for (int i = 0; i < 1e1; i++) //1e8
	{
		srand((unsigned)time(NULL));
		long long a = rand();//0x12345678;
		long long b = a * 1024; //效率提升20%？
		//long long c = a <<10; //使用随机数后差异很小
	}
	auto end = std::chrono::system_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	std::cout << "花费了"
		<< double(duration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den
		<< "秒" << std::endl;

}

static void _test2()
{
	size_t I = 5;
	size_t J = 5;
	for (int i = 0; i < I; i++)
	{
		for (int j = 0; j < J; j++)
		{
			if (j >= i)
			{
				//cout << "[" << i << "," << j << "]" << endl;
			}
		}
	}
	cout << "==================" << endl;

	//双层for循环，改单层for循环
	//int n = 6;
	//for (int k = 0; k < n * (n + 1) / 2; k++)
	//{
	//	int x = ceil(sqrt(n * (n + 1) - 2 * k + 0.25) + 0.5);
	//	int i = n - x + 1;
	//	int j = k - (n + x) * (n + 1 - x) / 2 + i;
	//	cout << "[" << i << "," << j << "]" << endl;
	//}

	int n = 5;
	int N = n * (n + 1) / 2;
	int i = 0;
	for (int k = 0; k < N; k++)
	{
		if (k>(2*n-1)*(i+1)/2-1)
			i++;
		int j = k - (n + i) * (n + 1 - i) / 2 + i;
		cout << "[" << i << "," << j << "]" << endl;
	}


}

static int enrol = []()->int
{
	_test2();
	return 0;
}();
