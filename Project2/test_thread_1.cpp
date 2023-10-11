#include "pch.h"
using namespace std;

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <unistd.h> //linux

// �����ȡ CpuID
#define CpuNum "wmic cpu get DeviceID"
// �����ȡ Cpu �������
#define CpuCoreNum "wmic cpu get NumberOfCores"
// �����ȡ Cpu �߼�����
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

// ��ȡ Cpu ��Ϣ
void getCpuInformation(const char* command)
{
	// ��ȡwindows�����ִ
	FILE* winCommand = _popen(command, "r");
	char buf[100] = {};

	if (!winCommand)
	{
		perror("popen");
		exit(EXIT_FAILURE);
	}
	else
	{
		// ��������ִ
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
	// �������Ҫ��ID����
	getCpuInformation(CpuNum);
	// �����������ֱ�Ӹ���
	getCpuInformation(CpuCoreNum);
	// �����߼�����ֱ�Ӹ���
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
// ����֪ʶ
//C++11���߳� https://blog.csdn.net/weixin_42193704/article/details/113920419

//1 ����
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

//2 operator����
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

//4 ��Ա����
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
	b.m_iX = 20;              // �޸�b.m_iX��ֵ�����
	cout << b.m_iX << endl;
	cout << "���߳�  " << &b << "  Thread_id: " << this_thread::get_id() << endl;
}


int main00()
{
	A3 a3(10);
	//thread t(fun, a3);
	thread t(fun, std::ref(a3));
	t.join();
	cout << a3.m_iX << endl;           // ���a.m_iX
	return 0;

	//-----------------------------------------------------------------------------------------------

	//explicit
	/*
	һ�� operator��ʽ����ת��
	�������캯������ʽ����ת��
	*/
	print_A2(0, 1); //��ʽת��



	int&& rRef = square(5); //C++11 ��������ֵ���õĸ���Ա�ʾһ����Ӧû�����Ƶ���ʱ����
	cout << rRef << endl;
	cout << rRef << endl;

	//-----------------------------------------------------------------------------------------------
	//�ڶ���
	//thread myt_obj(myprint);
	//A a;
	//thread myt_obj(a);
	thread myt_obj(my_lambda);
	if (myt_obj.joinable())
		myt_obj.join(); //����
	//if (myt_obj.joinable())
		//myt_obj.detach();//��̨����  //���߳�����ȴ���̫�ã�����detach
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
C++11�̴߳����ķ�ʽ

1.�������ݷ�ʽ

2.lambda���ʽ

3.���������

4.��Ա������Ϊ�߳����
*/


void myThread_value(int t)//���������ݵķ�ʽ
{
	for (int i = 0; i < 10; i++)
		cout << "test " << t << endl;
}

void myThread(string& a)//�����߳���ں���
{
	cout << "a = " << a << endl;
	a = "ttt";
}
void myThreadP(unique_ptr<string> a)//�����߳���ں���
{
	cout << "a value " << *a << endl;
	*a= "finish";
	return;
}
//std::move()   ��Ψһ�Ĺ����ǽ�һ����ֵǿ��ת��Ϊ��ֵ���ã��̶�����ͨ����ֵ����ʹ�ø�ֵ
//std::ref()  �����ã� ��Щapi�Ĵ�������ʵ�����ǿ���һ������ʹ�������������Ա�֤һ�������õ�
void run_thread()
{

	std::thread thread_test1(myThread_value, 2); //��ֵ
	thread_test1.join();

	//string a = "I am a man";
	//std::thread thread_test(myThread, std::ref(a));//���� �̱߳��� �����ʱ���߳̾ͻῪʼִ��

	unique_ptr<string> a(new string("I am a man"));
	std::thread thread_test(myThreadP, std::move(a));//���� �̱߳��� �����ʱ���߳̾ͻῪʼִ��
	thread_test.join();
	cout << "-->a " << a << endl;//ʹ��move�Ժ�ԭ�ȵ�ʵЧ�ˣ���������ʹ������ָ��Ӧ����ֻ��ʵ�ִ�ָ��
}
/*
ͬ��������
1.��򵥵ļ��� mutex.lock(); mutex.unlock();
2.�Զ�����&�Զ����� 	std::lock_guard<std::mutex>������
3.ͬʱ��ס���������
std::lock()��ʵ��ͬʱ���������������ע������Ҫ��ס������һ�����У���


*/
int t = 0;
mutex testMutex;
void MyThreadLock()
{
	for (int i=0;i<1e5;i++)
	{
		//ʹ�õ�������api lock()����ס��unlock()(�⿪��)������api Ҫ����ʹ�� ��������סȻ����� ��Ȼ�ͻ��������Ҳ���ǳ���ס
		testMutex.lock();
		t++; //��ס����������
		testMutex.unlock();

		//���������ʱ���Զ����� ���ձ�����ʱ���Զ����� �������þֲ�����������ʵ�ּ���
		std::lock_guard<std::mutex> lock(testMutex);
		t++;
	}
}
/*
Mutex �ֳƻ�����
mutex ϵ����(����)

std::mutex��������� Mutex �ࡣ
std::recursive_mutex���ݹ� Mutex �ࡣ
std::time_mutex����ʱ Mutex �ࡣ
std::recursive_timed_mutex����ʱ�ݹ� Mutex �ࡣ

Lock �ࣨ���֣�

std::lock_guard���� Mutex RAII ��أ������̶߳Ի�����������
std::unique_lock���� Mutex RAII ��أ������̶߳Ի��������������ṩ�˸��õ������ͽ������ơ�
*/
mutex testMutex1;
mutex testMutex2;
void myThread1()
{
	std::this_thread::sleep_for(std::chrono::seconds(1));//͢ʱ��ȷ��testcx1������
	std::lock(testMutex1, testMutex2);
	cout << "all lock " << endl;//�������������˲���ִ����һ��
	testMutex2.unlock();
	testMutex1.unlock();
	//std::lock(testMutex1, testMutex2);
	cout << "all 1ock two" << endl;
}
void myThread2()
{
	testMutex1.lock();
	cout << "mutex1 1ock" << endl; ;
	std::this_thread::sleep_for(std::chrono::seconds(1));//��ʱ-����õ��ܲ鿴��std::dack�ȴ���Ч��
	testMutex1.unlock();
	cout << "mutex1 unlock" << endl;
	std::this_thread::sleep_for(std::chrono::seconds(1));
}


static int main_thread()//���þͻ���
{ 
	//thread ������ʽ
	// std::thread thread_test(myThread_function);
	// std::thread thread_test(my_lambda_function);
	// 
	//MyThread  Thread; //��Ա����
	// std::thread thread_test(Thread);
	// std::thread thread_test(&MyThread::my_thread_in, &Thread);
	//thread_test.detach();
	/*
	detach
	1.ʹ��detach����������̷߳���
	2.ʹ��detach��������ʹ�þֲ�����
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
