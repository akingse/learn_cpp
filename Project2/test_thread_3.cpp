#include "pch.h"
using namespace std;

//程序员陈子青
//http://www.seestudy.cn/

void _add(int& x)
{
	x++;
}
void _addp(int* x)
{
	(*x)++;
}

static int main1()
{
	int a = 1;
	//int* a = new int(1);
	thread th1(_add, ref(a)); //std::invoke error
	if (th1.joinable())
		th1.join();
	cout << a << endl;
	return 0;
}

thread th2;
void test1()
{
	int a = 1; //局部变量不可使用ref
	th2 = thread(_add, ref(a));
}
static int main2()
{
	test1();
	if (th2.joinable())
		th2.join();
	return 0;
}

static int main3()
{
	int* ptr = new int(1);
	thread th3(_addp, ptr);
	delete ptr;
	//ptr = nullptr;
	if (th3.joinable())
		th3.join();

	cout << *ptr << endl;
	return 0;
}

class A
{
private:
	friend void thread_foo();
	void foo()
	{
		cout << "hello " << endl;
	}
public:
	void print(string str)
	{
		cout << "hello " << str << endl;
	}
};

void thread_foo()
{
	shared_ptr<A> pa = make_shared<A>();
	thread th3(&A::foo, pa);
	if (th3.joinable())
		th3.join();
}

static int main4()
{
	A a;
	string str;
	//thread th3(&A::print, ref(a), str);
	shared_ptr<A> pa = make_shared<A>();
	thread th3(&A::print, &a, str);
	thread th4(&A::print, pa, str);
	if (th3.joinable())
		th3.join();

	return 0;
}

//mutex
int shared_data = 0;
int shared_data2 = 0;
static std::mutex mtx;
void func(int n) 
{
	for (int i = 0; i < 10000; ++i) 
	{
		mtx.lock(); //只允许一个线程进行{区间}操作
		shared_data++;
		shared_data2++;
		//std::cout << "Thread " << n<< " increment shared_data to " << shared_data << std::endl;
		mtx.unlock();
	}
}

static int main5()
{
	std::thread t1(func,1);
	std::thread t2(func,2);
	t1.join();
	t2.join();
	std::cout << "shared_data = " << shared_data << std::endl;
	std::cout << "shared_data2 = " << shared_data2 << std::endl;
	return 0;
}

// dead lock
mutex m1, m2;
int i1 = 0, i2 = 0;
void func1()
{
	for (int i = 0; i < 1000; i++)
	{
		m1.lock(); //只要两个线程获取mutex的顺序不一样，无论如何unlock，必然会造成死锁
		m2.lock();
		//i1++;
		m1.unlock();
		m2.unlock();
	}
}
void func2()
{
	for (int i = 0; i < 1000; i++)
	{
		m2.lock();
		m1.lock();
		//i2++;
		m2.unlock();
		m1.unlock();
	}
}

static int main6()
{
	std::thread t1(func1);
	std::thread t2(func2);
	t1.join();
	t2.join();
	cout << "over" << endl;
	return 0;
}

static std::timed_mutex t_mtx;
void func3()
{
	for (int i = 0; i < 2; ++i)
	{
		//std::lock_guard<mutex> lg(mtx);
		//std::unique_lock<mutex> ug(mtx);
		//std::unique_lock<mutex> ug(mtx,std::defer_lock);
		std::unique_lock<timed_mutex> ug(t_mtx,std::defer_lock);
		//ug.lock();

		//this_thread::sleep_for(ch_seconds(5));
		ug.try_lock_for(chrono::seconds(2));
		shared_data++;
	}
}

static int main7()
{
	std::thread t1(func3);
	std::thread t2(func3);
	t1.join();
	t2.join();

	cout << shared_data << endl;
	cout << "over" << endl;
	return 0;
}


static int enrol = []()->int
	{
		//main1();
		//main2();
		//main3();
		//main4();
		//main5();
		//main6();
		main7();
		//main8();
		//main9();
		//main10();
		return 0;
	}();
