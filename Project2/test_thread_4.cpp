#include "pch.h"
using namespace std;
//https://blog.csdn.net/weixin_40582034/article/details/132040437?spm=1001.2014.3001.5501

//C++11 bind function forward 

//std::bind 是一个函数模板，定义在 <functional> 头文件中。它的作用是将一个可调用对象（比如函数、函数指针、成员函数、成员函数指针等等）
// 以及若干个参数绑定到一个新的函数对象上，形成一个新的可调用对象。

int _add(int a, int b) {
	return a + b;
}

static int main1()
{
	auto add_five = std::bind(_add, 5, std::placeholders::_1);
	std::cout << add_five(3) << std::endl; // 输出 8


	return 0;
}

//C++11 中引入了 std::function 类型，它是一个多态函数封装类，可以用来存储和调用任意可调用对象，包括函数指针、函数对象、Lambda 表达式等等。

static int main2() 
{
	std::function<int(int, int)> add = [](int x, int y) { return x + y; };
	std::cout << add(3, 4) << std::endl; // 输出 7

	if (add)//add.operator bool();
		cout << "not empty" << endl;
	add = nullptr;
	if (add)//add.operator bool();
		cout << "not empty" << endl;
	std::function<void()> func1 = []() { std::cout << "Hello, world!" << std::endl; };
	std::function<void()> func2;
	//std::cout << std::boolalpha << (func1 == func2) << std::endl; // 输出 false
	func2 = func1;
	//std::cout << std::boolalpha << (func1 == func2) << std::endl; // 输出 true

	return 0;
}
//std::forward 标准库中的一个函数模板，用于完美转发参数。它的主要作用是在函数模板中将传入的参数按照它们的值类别（左值或右值）进行转发，
// 以保持它们的值类别不变，从而避免不必要的拷贝和移动操作。

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
	bar(i);          // 输出 "lvalue"
	std::cout << i;  // 输出 43
	bar(42);         // 输出 "rvalue"
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
	//不需要手动创建线程，不阻塞主线程
	std::future<int>  futrue_result = std::async(std::launch::async, func); //后台执行的异步线程，结果保存在futrue_result对象里
	//cout << func() << endl;
	cout << futrue_result.get() << endl;

	std::packaged_task<int()> task(func);
	auto futrue_result2 = task.get_future();
	std::thread th(move(task)); //packaged_task无拷贝构造，只能move
	cout << func() << endl;
	th.join();
	cout << futrue_result2.get() << endl;


	return 0;
}

// promise 不同线程之间传递变量
void func2(std::promise<int>& val)
{
	val.set_value(1000);
}
//在一个线程中设置值，在另一个线程中获取值

static int main5()
{
	std::promise<int> p;
	auto futrue_result = p.get_future();

	thread th(func2, std::move(p)); //不能使用重载
	th.join();
	cout << futrue_result.get() << endl;

	return 0;
}

//atomic 原子操作
//#define _USING_ATOMIC //速度更快
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
	//计时
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
