#include "pch.h"


static void _test1()
{
	//��������
//Debug��10����release��5��
// ��ʱ
	auto start = std::chrono::system_clock::now();

	//for (int i = 0; i < 1e6; i++)
	//{
	//	//�����
	//	srand((unsigned)time(NULL));
	//	string rd = to_string(rand());
	//	string md0 = para::getMD5(rd);
	//	//MD5_32B md0 = para::getMD5_ULL(rd);
	//}


	//rand����

	/*
	int num = rand() % 100; //����0~99��100�������е�һ���������
	rand() % (b-a+1)+ a ;    //��ʾ  a~b ֮���һ�����������
	ͨ��rand()�������������ÿ�����е�ʱ��������һ����ͬ�ģ�������Ϊ�˱��ڳ���ĵ��ԡ���Ҫ����ÿ�β�ͬ��������������ʹ��srand( seed )�������в�����������ӣ�����seed�Ĳ�ͬ�����ܹ�������ͬ���������

	rand() �᷵��һ�����ֵ����Χ�� 0 �� RAND_MAX �䡣
	rand()�������Ǽ�������֣�ÿ��ִ��ʱ����ͬ�ġ���Ҫ��ͬ,�Բ�ͬ��ֵ����ʼ����.��ʼ���ĺ������� srand()��
	*/
	int a = RAND_MAX; //32767 =2^(16-1)
	int rd0 = rand() - RAND_MAX / 2;
	int rd = rand() - 0x3fff;


	srand((int)time(0));

	for (int i = 0; i < 1e1; i++) //1e8
	{
		srand((unsigned)time(NULL));
		long long a = rand();//0x12345678;
		long long b = a * 1024; //Ч������20%��
		//long long c = a <<10; //ʹ�������������С
	}
	auto end = std::chrono::system_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	std::cout << "������"
		<< double(duration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den
		<< "��" << std::endl;

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

	//˫��forѭ�����ĵ���forѭ��
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
