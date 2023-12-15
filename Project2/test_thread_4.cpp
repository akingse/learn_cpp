#include "pch.h"
using namespace std;
//https://blog.csdn.net/weixin_40582034/article/details/132040437?spm=1001.2014.3001.5501

//C++11 bind function forward 

//std::bind ��һ������ģ�壬������ <functional> ͷ�ļ��С����������ǽ�һ���ɵ��ö��󣨱��纯��������ָ�롢��Ա��������Ա����ָ��ȵȣ�
// �Լ����ɸ������󶨵�һ���µĺ��������ϣ��γ�һ���µĿɵ��ö���

int _add(int a, int b) {
	return a + b;
}

static int main1()
{
	auto add_five = std::bind(_add, 5, std::placeholders::_1);
	std::cout << add_five(3) << std::endl; // ��� 8


	return 0;
}

//C++11 �������� std::function ���ͣ�����һ����̬������װ�࣬���������洢�͵�������ɵ��ö��󣬰�������ָ�롢��������Lambda ���ʽ�ȵȡ�

static int main2() 
{
	std::function<int(int, int)> add = [](int x, int y) { return x + y; };
	std::cout << add(3, 4) << std::endl; // ��� 7

	if (add)//add.operator bool();
		cout << "not empty" << endl;
	add = nullptr;
	if (add)//add.operator bool();
		cout << "not empty" << endl;
	std::function<void()> func1 = []() { std::cout << "Hello, world!" << std::endl; };
	std::function<void()> func2;
	//std::cout << std::boolalpha << (func1 == func2) << std::endl; // ��� false
	func2 = func1;
	//std::cout << std::boolalpha << (func1 == func2) << std::endl; // ��� true

	return 0;
}
//std::forward ��׼���е�һ������ģ�壬��������ת��������������Ҫ�������ں���ģ���н�����Ĳ����������ǵ�ֵ�����ֵ����ֵ������ת����
// �Ա������ǵ�ֵ��𲻱䣬�Ӷ����ⲻ��Ҫ�Ŀ������ƶ�������

void foo(int& x) {
	std::cout << "lvalue" << std::endl;
	++x;
}

void foo(int&& x) {
	std::cout << "rvalue" << std::endl;
	++x;
}

template <typename T>
void bar(T&& x) {
	auto res = std::forward<T>(x);
	foo(std::forward<T>(x));
}

static int main3() 
{
	int i = 42;
	bar(i);          // ��� "lvalue"
	std::cout << i;  // ��� 43
	bar(42);         // ��� "rvalue"
	return 0;
}


//future async
int func()
{
	int res = 0;
	for (int i = 0; i < 1000; i++)
		res++;
	return res;
}


static int main4()
{
	//����Ҫ�ֶ������̣߳����������߳�
	std::future<int>  futrue_result = std::async(std::launch::async, func); //��ִ̨�е��첽�̣߳����������futrue_result������
	//cout << func() << endl;
	cout << futrue_result.get() << endl;

	std::packaged_task<int()> task(func);
	auto futrue_result2 = task.get_future();
	std::thread th(move(task)); //packaged_task�޿������죬ֻ��move
	cout << func() << endl;
	th.join();
	cout << futrue_result2.get() << endl;


	return 0;
}

// promise ��ͬ�߳�֮�䴫�ݱ���
void func2(std::promise<int>& val)
{
	val.set_value(1000);
}
//��һ���߳�������ֵ������һ���߳��л�ȡֵ

static int main5()
{
	std::promise<int> p;
	auto futrue_result = p.get_future();

	thread th(func2, std::move(p)); //����ʹ������
	th.join();
	cout << futrue_result.get() << endl;

	return 0;
}

//atomic ԭ�Ӳ���
//#define _USING_ATOMIC //�ٶȸ���
#ifdef _USING_ATOMIC
atomic<int> count0 = 0;
#else
int count0 = 0;
static mutex mtx;
#endif // _USING_ATOMIC

static void func3()
{
	for (int i = 0; i < 1000000; ++i)
	{
#ifdef _USING_ATOMIC
		count0++;
#else
		mtx.lock();
		count0++;
		mtx.unlock();
#endif
	}

}

static int main6()
{
	//��ʱ
	auto startT = chrono::duration_cast<chrono::microseconds>(
		chrono::system_clock::now().time_since_epoch()).count();
	thread th1(func3);
	thread th2(func3);
	th1.join();
	th2.join();
	cout << count0 << endl;
	auto endT = chrono::duration_cast<chrono::microseconds>(
		chrono::system_clock::now().time_since_epoch()).count();
	cout << "time_cost:"<<endT - startT << endl;

	return 0;
}

static int enrol = []()->int
	{
		//main1();
		//main2();
		//main3();
		//main4();
		//main5();
		main6();
		return 0;
	}();
