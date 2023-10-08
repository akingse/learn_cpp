#include "pch.h"
using namespace std;

//https://blog.csdn.net/weixin_43971373/article/details/119678930

//创建线程的4种方法

void thread_task(int i)
{
	std::cout << "hello thread " << i << std::endl;
}

static int main1() // void
{
	std::thread t(thread_task, 5);
	t.join();

	return 0;
}

static int main2() //lambda
{
	auto thread_task = [](int i) {std::cout << "hello thread " << i << std::endl; };
	std::thread t(thread_task, 5);
	t.join();

	return 0;
}


class MyClass
{
public:
	void thread_task(int i)
	{
		std::cout << "hello thread " << i << std::endl;
	}

	void operator()(int i)
	{
		std::cout << "hello thread " << i << std::endl;
	}
};

static int main3() //成员函数
{
	MyClass myclass;
	//std::thread t(&MyClass::thread_task, myclass, 5);		//这种调用会调用myclass的副本
	std::thread t(&MyClass::thread_task, &myclass, 5);
	t.join();

	return 0;
}


static int main4() //可调用对象
{
	MyClass myclass;
	std::thread t(myclass, 5);
	t.join();

	return 0;
}

//-------------------------------------------------------------------------------------------------
// mustex
//-------------------------------------------------------------------------------------------------

/*
线程之间的锁主要有互斥锁、条件锁、自旋锁、读写锁、递归锁（一般锁功能越强大，性能越低）


*/
int a = 0;
std::mutex mtx;

void thread_task1()
{
	for (int i = 0; i < 100000; ++i)
	{
		mtx.lock(); //加锁，在某个线程获取了互斥量的锁后，不允许别的线程再获取锁
		++a;
		mtx.unlock();
	}
}

void thread_task2()
{
	for (int i = 0; i < 100000; ++i)
	{
		mtx.lock();
		++a;
		mtx.unlock();
	}
}

static int main5()
{
	std::thread t1(thread_task1);
	std::thread t2(thread_task2);
	t1.join();
	t2.join();
	std::cout << a << std::endl;
	return 0;
}


std::timed_mutex mtx_t;
void fireworks() {
	// waiting to get a lock: each thread prints "-" every 200ms:
	//函数接受一个时间范围，表示在这一段时间范围之内线程如果没有获得锁则被阻塞住
	while (!mtx_t.try_lock_for(std::chrono::milliseconds(200))) 
	{
		std::cout << "-";
	}
	// got a lock! - wait for 1s, then this thread prints "*"
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	std::cout << "*\n";
	mtx_t.unlock();
}

static int main6()
{
	std::thread m_threads[10];
	// spawn 10 threads:
	for (int i = 0; i < 10; ++i)
		m_threads[i] = std::thread(fireworks);

	for (auto& th : m_threads) 
		th.join();

	return 0;
}

std::mutex mtx1;
std::mutex mtx2;

void thread_task3()
{
	for (int i = 0; i < 100000; ++i)
	{
		mtx1.lock();
		mtx2.lock();
		++a;
		mtx2.unlock();
		mtx1.unlock();
	}
}

void thread_task4()
{
	for (int i = 0; i < 100000; ++i)
	{
		mtx2.lock();
		mtx1.lock();
		++a;
		mtx1.unlock();
		mtx2.unlock();
	}
}

static int main7()
{
	std::thread t1(thread_task3); //死锁没有任何反应
	std::thread t2(thread_task4);
	t1.join();
	t2.join();
	std::cout << a << std::endl;
	return 0;
}


//使用了lock_guard后就不应该对mutex再使用lock或unlock了。
//在定义时构造函数中会lock，析构函数会自动unlock。
void thread_task5()
{
	for (int i = 0; i < 100000; ++i)
	{
		std::lock_guard<std::mutex> lock(mtx);
		//mtx.lock();
		++a;
		//mtx.unlock();
	}
}

void thread_task6()
{
	for (int i = 0; i < 100000; ++i)
	{
		std::lock_guard<std::mutex> lock(mtx);
		//mtx.lock();
		++a;
		//mtx.unlock();
	}
}

static int main8()
{
	std::thread t1(thread_task5);
	std::thread t2(thread_task6);
	t1.join();
	t2.join();
	std::cout << a << std::endl;
	return 0;
}


void thread_task7()
{
	for (int i = 0; i < 100000; ++i)
	{
		std::lock(mtx1, mtx2);
		//mtx1.lock();
		//mtx2.lock();
		++a;
		mtx2.unlock();
		mtx1.unlock();
	}
}

void thread_task8()
{
	for (int i = 0; i < 100000; ++i)
	{
		std::lock(mtx1, mtx2);
		//mtx1.lock();
		//mtx2.lock();
		++a;
		mtx1.unlock();
		mtx2.unlock();
	}
}

static int main9()
{
	std::thread t1(thread_task7);
	std::thread t2(thread_task8);
	t1.join();
	t2.join();
	std::cout << a << std::endl;
	return 0;
}


void thread_task9()
{
	for (int i = 0; i < 100000; ++i)
	{
		std::lock(mtx1, mtx2);
		std::lock_guard<std::mutex> lock1(mtx1, std::adopt_lock);
		std::lock_guard<std::mutex> lock2(mtx2, std::adopt_lock);
		++a;
	}
}

void thread_task10()
{
	for (int i = 0; i < 100000; ++i)
	{
		std::lock(mtx1, mtx2);
		std::lock_guard<std::mutex> lock1(mtx1, std::adopt_lock);
		std::lock_guard<std::mutex> lock2(mtx2, std::adopt_lock);
		++a;
	}
}

static int main10()
{
	std::thread t1(thread_task9);
	std::thread t2(thread_task10);
	t1.join();
	t2.join();
	std::cout << a << std::endl;
	return 0;
}

void thread_task11()
{
	for (int i = 0; i < 100000; ++i)
	{
		std::unique_lock<std::mutex> lock(mtx);
		++a;
	}
}

void thread_task12()
{
	for (int i = 0; i < 100000; ++i)
	{
		std::unique_lock<std::mutex> lock(mtx);
		++a;
	}
}

static int main11()
{
	std::thread t1(thread_task11);
	std::thread t2(thread_task12);
	t1.join();
	t2.join();
	std::cout << a << std::endl;
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
	//main7();
	//main8();
	//main9();
	//main10();
	//main11();
	return 0;
}();
