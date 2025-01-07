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
		std::unique_lock<mutex> ug(mtx);
		//std::unique_lock<mutex> ug(mtx,std::defer_lock);
		shared_data++;
	}
}
void func4()
{
	for (int i = 0; i < 2; ++i)
	{
		std::unique_lock<timed_mutex> ug(t_mtx, std::defer_lock); //延迟锁
		//ug.lock();

		if (ug.try_lock_for(chrono::seconds(2))) //等待2s,超时跳过
		{
			this_thread::sleep_for(chrono::seconds(5));
			shared_data++;
		}

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

// std::call_once //保证单例 log只被执行一次
class Log;
static Log* plog = nullptr;
static std::once_flag once;
class Log
{
public:
	Log() = default;
	Log(const Log& log) = delete;
	Log operator=(const Log& log) = delete;
	static Log& getInstance()
	{
		//if (!log)
		//	log = new Log;
		std::call_once(once, init); //此方法保证一些单例new的操作，只被执行一次；
		return *plog;
	}
	static void init()
	{
		if (!plog)
			plog = new Log;
	}
	void print(string msg)
	{
		cout << __TIME__ << msg << endl;
	}
};

static void func5()
{
	Log::getInstance().print("error");
}

static int main8()
{
	std::thread t1(func5);
	std::thread t2(func5);
	t1.join();
	t2.join();
	return 0;
}

// condition variable
// producer and consumer 生产者消费者模型
// 生产者 -> 生产任务 -> 任务队列
// 任务队列 -> 消费者 -> 完成任务

//#include<condition_variable>
std::queue<int> g_queue;
std::condition_variable g_cv;

void producer()
{
	for (int i = 0; i < 10; i++)
	{
		std::unique_lock<mutex> ulock(mtx);
		g_queue.push(i);
		//通知consumer
		g_cv.notify_one();
		cout << "producer task_"<< i << endl;
		this_thread::sleep_for(chrono::milliseconds(200));
	}
}

void consumer()
{
	while (true)
	{
		// mepty 等待
		std::unique_lock<mutex> ulock(mtx);
		//if (g_queue.empty())
		//g_cv.wait(ulock, !g_queue.empty());
		g_cv.wait(ulock, []() { // producer for-loop finish, this wait always block
			return !g_queue.empty();
			});
		int j = g_queue.front();
		g_queue.pop();
		cout << "consumer deal_" << j << endl;
		//this_thread::sleep_for(chrono::milliseconds(100));

	}
}
//predict 预测
//predicate 谓语
static int main9()
{
	std::thread t1(producer);
	std::thread t2(consumer);
	t1.join();
	t2.join();
	return 0;
}

//线程池
//维护一个thread vector 和task queue

class ThreadPool
{
private:
	vector<thread> m_threads;
	queue<function<void()>> m_tasks;
	mutex m_mtx;
	condition_variable m_cv;
	bool m_stop;

public:
	ThreadPool(int n):m_stop(false)
	{
		for (int i = 0; i < n; i++)
			m_threads.emplace_back([this]() //thread不支持拷贝构造，只能使用emplace
				{
					while (true)
					{
						std::unique_lock<mutex> ulock(m_mtx);
						m_cv.wait(ulock, [this]() {
							return !m_tasks.empty() || m_stop;
							});
						if (m_stop && m_tasks.empty())
							return;
						function<void()> task(move(m_tasks.front()));
						m_tasks.pop();
						ulock.unlock();
						task(); //execute
					}
				});
	}
	~ThreadPool()
	{
		{
			unique_lock<mutex> ulock(m_mtx);
			m_stop = true;
		}
		m_cv.notify_all();
		for (auto& iter : m_threads)
		{
			iter.join();
		}
	}
	template<class F, class... Args>
	void enqueue(F&& f, Args&&... args) //函数模板里&&是通用引用,相当于完美转发
	{
		function<void()> task = std::bind(forward<F>(f), forward<Args>(args)...);
		{
			unique_lock<mutex> ulock(m_mtx);
			m_tasks.emplace(move(task));
		}
		m_cv.notify_one();


	}


};



static int main10()
{
	ThreadPool tpool(5); //最好是单例类
	for (int i = 0; i < 10; i++)
	{
		tpool.enqueue([i]() {
			unique_lock<mutex> ulock(mtx);

			cout << "task_" << i << endl;
			this_thread::sleep_for(chrono::seconds(1));
			cout << "task_" << i <<"done"<< endl;


			});
	}


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
		return 0;
	}();
