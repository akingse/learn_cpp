#include "pch.h"
using namespace std;

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <unistd.h> //linux

// 命令调取 CpuID
#define CpuNum "wmic cpu get DeviceID"
// 命令调取 Cpu 物理核数
#define CpuCoreNum "wmic cpu get NumberOfCores"
// 命令调取 Cpu 逻辑核数
#define CpuLogicalCoreNum "wmic cpu get NumberOfLogicalProcessors"

#define __AddressWidth "wmic cpu get AddressWidth"
#define __Architecture "wmic cpu get Architecture"
#define __AssetTag "wmic cpu get AssetTag"
#define __Availability "wmic cpu get Availability"
#define __Caption "wmic cpu get Caption"
#define __Characteristics "wmic cpu get Characteristics"
#define __ConfigManagerErrorCode "wmic cpu get ConfigManagerErrorCode"
#define __ConfigManagerUserConfig "wmic cpu get ConfigManagerUserConfig"
#define __CpuStatus "wmic cpu get CpuStatus"
#define __CreationClassName "wmic cpu get CreationClassName"
#define __CurrentClockSpeed "wmic cpu get CurrentClockSpeed"
#define __CurrentVoltage "wmic cpu get CurrentVoltage"
#define __DataWidth "wmic cpu get DataWidth"
#define __Description "wmic cpu get Description"
#define __DeviceID "wmic cpu get DeviceID"
#define __ErrorCleared "wmic cpu get ErrorCleared"
#define __ErrorDescription "wmic cpu get ErrorDescription"
#define __ExtClock "wmic cpu get ExtClock"
#define __Family "wmic cpu get Family"
#define __InstallDate "wmic cpu get InstallDate"
#define __L2CacheSize "wmic cpu get L2CacheSize"
#define __L2CacheSpeed "wmic cpu get L2CacheSpeed"
#define __L3CacheSize "wmic cpu get L3CacheSize"
#define __L3CacheSpeed "wmic cpu get L3CacheSpeed"
#define __LastErrorCode "wmic cpu get LastErrorCode"
#define __Level "wmic cpu get Level"
#define __LoadPercentage "wmic cpu get LoadPercentage"
#define __Manufacturer "wmic cpu get Manufacturer"
#define __MaxClockSpeed "wmic cpu get MaxClockSpeed"
#define __Name "wmic cpu get Name"
#define __NumberOfCores "wmic cpu get NumberOfCores"
#define __NumberOfEnabledCore "wmic cpu get NumberOfEnabledCore"
#define __NumberOfLogicalProcessors "wmic cpu get NumberOfLogicalProcessors"
#define __OtherFamilyDescription "wmic cpu get OtherFamilyDescription"
#define __PartNumber "wmic cpu get PartNumber"
#define __PNPDeviceID "wmic cpu get PNPDeviceID"
#define __PowerManagementCapabilities "wmic cpu get PowerManagementCapabilities"
#define __PowerManagementSupported "wmic cpu get PowerManagementSupported"
#define __ProcessorId "wmic cpu get ProcessorId"
#define __ProcessorType "wmic cpu get ProcessorType"
#define __Revision "wmic cpu get Revision"
#define __Role "wmic cpu get Role"
#define __SecondLevelAddressTranslationExtensions                              \
    "wmic cpu get SecondLevelAddressTranslationExtensions"
#define __SerialNumber "wmic cpu get SerialNumber"
#define __SocketDesignation "wmic cpu get SocketDesignation"
#define __Status "wmic cpu get Status"
#define __StatusInfo "wmic cpu get StatusInfo"
#define __Stepping "wmic cpu get Stepping"
#define __SystemCreationClassName "wmic cpu get SystemCreationClassName"
#define __SystemName "wmic cpu get SystemName"
#define __ThreadCount "wmic cpu get ThreadCount"
#define __UniqueId "wmic cpu get UniqueId"
#define __UpgradeMethod "wmic cpu get UpgradeMethod"
#define __Version "wmic cpu get Version"
#define __VirtualizationFirmwareEnabled                                        \
    "wmic cpu get VirtualizationFirmwareEnabled"
#define __VMMonitorModeExtensions "wmic cpu get VMMonitorModeExtensions"
#define __VoltageCaps "wmic cpu get VoltageCaps"

// 获取 Cpu 信息
void getCpuInformation(const char* command)
{
	// 获取windows命令回执
	FILE* winCommand = _popen(command, "r");
	char buf[100] = {};

	if (!winCommand)
	{
		perror("popen");
		exit(EXIT_FAILURE);
	}
	else
	{
		// 输出命令回执
		while (fgets(buf, sizeof(buf) - 1, winCommand) != 0)
		{
			printf("%s", buf);
			memset(buf, 0, sizeof(buf));
		}

		_pclose(winCommand);
	}
}

int GetNoOfProcessors()
{
	SYSTEM_INFO si;

	GetSystemInfo(&si);

	return si.dwNumberOfProcessors;
}

static int main0()
{
	// 具体核数要数ID个数
	getCpuInformation(CpuNum);
	// 具体物理核数直接给出
	getCpuInformation(CpuCoreNum);
	// 具体逻辑核数直接给出
	getCpuInformation(CpuLogicalCoreNum);

	return 0;
}

static int enrol = []()->int
{
	main0();
	//main1();
	//main2();

	return 0;
}();



//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
// 
// 基础知识
//C++11多线程 https://blog.csdn.net/weixin_42193704/article/details/113920419

//1 函数
void myprint(/*int i*/)
{

	cout << "\a" << endl;
	cout << "child thread begin." << endl;
	cout << "child thread end." << endl;
	//Sleep(1000);
	cout << "\a" << endl;


	//while (!true)
	//{
	//	cout << "child thread." << endl;
	//	Sleep(1 * 1000);
	//}
}

//2 operator重载
class A
{
public:
	void operator()()
	{
		cout << "child thread begin." << endl;
		cout << "child thread end." << endl;
	}
};

//3 lambda
auto my_lambda = [] {
	cout << "child thread begin." << endl;
	cout << "child thread end." << endl;
};

//4 成员函数
class  MyThread
{
public:
	MyThread() {};
	 ~MyThread() {};
	void my_thread_in()
	{
		cout << "No.4 thread start" << endl;
	}
	void operator ()()
	{
		cout << "No.3 thread start " << endl;
	}

private:

};



//---------------------------------------------------
class A2
{
public:
	int m_i;

	A2(int a) :m_i(a) { cout << "This is Construct" << endl; }
	A2(const A2& a) :m_i(a.m_i) { cout << "This is Copy Construct" << endl; }
	~A2() { cout << "This is Destruct" << endl; }

};

void print_A2(int i, const A2& a)
{
	cout << a.m_i << endl;
}

int square(int x)
{
	return x * x;
}

class A3
{
public:
	mutable int m_iX;
	A3(int x) : m_iX(x) {
		cout << "constru  " << this << "   Thread_id: " << this_thread::get_id() << endl;
	}
	A3(const A3& a) {
		m_iX = a.m_iX;
		cout << "copycon  " << this << "  Thread_id: " << this_thread::get_id() << endl;
	}
	~A3() {
		cout << "destru  " << this << "  Thread_id: " << this_thread::get_id() << endl;
	}
};

void fun(const A3& b) {
	b.m_iX = 20;              // 修改b.m_iX的值并输出
	cout << b.m_iX << endl;
	cout << "子线程  " << &b << "  Thread_id: " << this_thread::get_id() << endl;
}


int main00()
{
	A3 a3(10);
	//thread t(fun, a3);
	thread t(fun, std::ref(a3));
	t.join();
	cout << a3.m_iX << endl;           // 输出a.m_iX
	return 0;

	//-----------------------------------------------------------------------------------------------

	//explicit
	/*
	一、 operator隐式类型转换
	二、构造函数的隐式类型转换
	*/
	print_A2(0, 1); //隐式转换



	int&& rRef = square(5); //C++11 引入了右值引用的概念，以表示一个本应没有名称的临时对象。
	cout << rRef << endl;
	cout << rRef << endl;

	//-----------------------------------------------------------------------------------------------
	//第二节
	//thread myt_obj(myprint);
	//A a;
	//thread myt_obj(a);
	thread myt_obj(my_lambda);
	if (myt_obj.joinable())
		myt_obj.join(); //阻塞
	//if (myt_obj.joinable())
		//myt_obj.detach();//后台运行  //主线程逐个等待不太好，引入detach
	cout << "I love China." << "" << endl;
	cout << "I love China." << "" << endl;
	cout << "I love China." << "" << endl;

	//while (!true)
	//{
	//	cout << "main thread." << endl;
	//	Sleep(2*1000);
	//}

	//-----------------------------------------------------------------------------------------------
	return 0;
}

/*
C++11线程创建的方式

1.函数传递方式

2.lambda表达式

3.重载运算符

4.成员函数做为线程入口
*/


void myThread_value(int t)//带参数传递的方式
{
	for (int i = 0; i < 10; i++)
		cout << "test " << t << endl;
}

void myThread(string& a)//定义线程入口函数
{
	cout << "a = " << a << endl;
	a = "ttt";
}
void myThreadP(unique_ptr<string> a)//定义线程入口函数
{
	cout << "a value " << *a << endl;
	*a= "finish";
	return;
}
//std::move()   它唯一的功能是将一个左值强制转化为右值引用，继而可以通过右值引用使用该值
//std::ref()  传引用， 有些api的传递引用实际上是拷贝一个副本使用这个函数则可以保证一定是引用的
void run_thread()
{

	std::thread thread_test1(myThread_value, 2); //传值
	thread_test1.join();

	//string a = "I am a man";
	//std::thread thread_test(myThread, std::ref(a));//定义 线程变量 定义的时候线程就会开始执行

	unique_ptr<string> a(new string("I am a man"));
	std::thread thread_test(myThreadP, std::move(a));//定义 线程变量 定义的时候线程就会开始执行
	thread_test.join();
	cout << "-->a " << a << endl;//使用move以后原先的实效了，这样看来使用智能指针应该是只能实现传指针
}
/*
同步互斥锁
1.最简单的加锁 mutex.lock(); mutex.unlock();
2.自动加锁&自动解锁 	std::lock_guard<std::mutex>变量名
3.同时锁住多个互斥量
std::lock()来实现同时锁定多个互斥量（注意最少要锁住两个，一个不行），


*/
int t = 0;
mutex testMutex;
void MyThreadLock()
{
	for (int i=0;i<1e5;i++)
	{
		//使用的是两个api lock()（锁住）unlock()(解开锁)这两个api 要配套使用 必须先锁住然后解锁 不然就会出现死锁也就是程序卡住
		testMutex.lock();
		t++; //锁住，保护数据
		testMutex.unlock();

		//定义变量的时候自动加锁 回收变量的时候自动解锁 这里利用局部变量的特性实现加锁
		std::lock_guard<std::mutex> lock(testMutex);
		t++;
	}
}
/*
Mutex 又称互斥量
mutex 系列类(四种)

std::mutex，最基本的 Mutex 类。
std::recursive_mutex，递归 Mutex 类。
std::time_mutex，定时 Mutex 类。
std::recursive_timed_mutex，定时递归 Mutex 类。

Lock 类（两种）

std::lock_guard，与 Mutex RAII 相关，方便线程对互斥量上锁。
std::unique_lock，与 Mutex RAII 相关，方便线程对互斥量上锁，但提供了更好的上锁和解锁控制。
*/
mutex testMutex1;
mutex testMutex2;
void myThread1()
{
	std::this_thread::sleep_for(std::chrono::seconds(1));//廷时秒确定testcx1点锁了
	std::lock(testMutex1, testMutex2);
	cout << "all lock " << endl;//所有锁都锁上了才能执行这一句
	testMutex2.unlock();
	testMutex1.unlock();
	//std::lock(testMutex1, testMutex2);
	cout << "all 1ock two" << endl;
}
void myThread2()
{
	testMutex1.lock();
	cout << "mutex1 1ock" << endl; ;
	std::this_thread::sleep_for(std::chrono::seconds(1));//延时-秒更好的能查看到std::dack等待的效果
	testMutex1.unlock();
	cout << "mutex1 unlock" << endl;
	std::this_thread::sleep_for(std::chrono::seconds(1));
}


static int main_thread()//不用就会忘
{ 
	//thread 创建方式
	// std::thread thread_test(myThread_function);
	// std::thread thread_test(my_lambda_function);
	// 
	//MyThread  Thread; //成员函数
	// std::thread thread_test(Thread);
	// std::thread thread_test(&MyThread::my_thread_in, &Thread);
	//thread_test.detach();
	/*
	detach
	1.使用detach（）会和主线程分离
	2.使用detach（）不能使用局部变量
	*/

	//if(thread_test.joinable())
	//	thread_test.join();
	
	//run_thread();
	//thread thread1(MyThreadLock);
	//thread thread2(MyThreadLock);
	//cout << t << endl;

	thread thread1(myThread1);
	thread thread2(myThread2);
	thread1.join();
	thread2.join();
	return 0;
}
