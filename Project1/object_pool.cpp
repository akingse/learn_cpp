#include "pch.h"
using namespace std;

// 内存管理，借出，归还置空
// list 链表解决 double等小内存对象

template<class Object>
class ObjectPool2
{

private:
	size_t m_size;
	//std::list<Object> m_pool;
	std::vector<Object> m_pool;
	//Gene gene;

public:
	ObjectPool2(size_t size) :m_size(size)
	{
		//_nSize = size;
		for (size_t n = 0; n < m_size; n++)
		{
			m_pool.push_back(move(Object()));
		}
	}
	virtual ~ObjectPool2()
	{
		auto iter = m_pool.begin();
		//auto name = typeid(iter).name();
		while (iter != m_pool.end())
		{
			//delete* iter;
			++iter;
		}
		m_size = 0;
	}

	Object requestObject()
	{
		Object pObj;// = NULL;
		if (m_size == 0)
		{
			pObj = Object(); //Object();
		}
		else
		{//为了防止冲突，每次获取对象的时候，会从链表的头进行获取，然后将头删除
			pObj = m_pool[m_size];
			//m_pool.pop_back();
			--m_size;
		}
		return pObj;
	}

	void returnObject(Object pObj)
	{
		//与获取对象相同，回收对象时，将对象加入到对象池的尾部，这样就不会和获取对象有所冲突；
		//m_pool.push_back(pObj);
		++m_size;
	}

};


template<class Object>
class ObjectPool
{

private:
	size_t m_size;
	std::list<Object*> m_pool;
	//Gene gene;

public:
	ObjectPool(size_t size) :m_size(size)
	{
		//_nSize = size;
		for (size_t n = 0; n < m_size; n++)
		{
			m_pool.push_back(new Object());
		}
	}
	virtual ~ObjectPool()
	{
		auto iter = m_pool.begin();
		//auto name = typeid(iter).name();
		while (iter != m_pool.end())
		{
			delete* iter;
			++iter;
		}
		m_size = 0;
	}

	Object* requestObject()
	{
		Object* pObj = NULL;
		if (m_size == 0)
		{
			pObj = new Object(); //Object();
		}
		else
		{//为了防止冲突，每次获取对象的时候，会从链表的头进行获取，然后将头删除
			pObj = m_pool.front(); //返回第一个元素
			m_pool.pop_front(); //删除第一个元素
			--m_size;
		}
		return pObj;
	}

	void returnObject(Object* pObj)
	{
		//与获取对象相同，回收对象时，将对象加入到对象池的尾部，这样就不会和获取对象有所冲突；
		m_pool.push_back(pObj);
		++m_size;
	}

};




class Test
{
public:
	double m_a;
	Test(double a=1):m_a(a) {}
	~Test() {}
	void Print()
	{
		//cout << "Test" << endl;
	}
};


//内存碎片
//不定长度内存

int mainpool()
{
	//Gene gnrc;
	//cout << sizeof(gnrc) << endl;
	//cout << sizeof(Gene(1)) << endl;
	//cout << sizeof(Gene(1.0)) << endl;
	//cout << sizeof(Vec2) << endl;
	//cout << sizeof(Vec3) << endl;
	////cout << sizeof(Gene(Vec2())) << endl;
	//Gene vec= Gene(Vec3());
	//cout << sizeof(vec) << endl;

	std::chrono::hours;   // 时
	std::chrono::minutes;   // 分
	std::chrono::seconds;   // 秒
	std::chrono::milliseconds;   // 毫秒
	std::chrono::microseconds;   // 微妙
	std::chrono::nanoseconds;   // 纳秒

	//测试用时
	//double a[long long(1e5)]; //vs不让设1e6
	//double* a = new double[1e6];
	vector<double> tVec;
	vector<double*> ptVec;
	vector<Vec3> vVec(1e6);
	vector<Vec3*> pvVec(1e6);
	Vec3* a = new Vec3[1e6];

	auto startTime0 = std::chrono::system_clock::now();
	//for (int j = 0; j < 1e1; j++)
		for (int i = 0; i < 1e6; i++)
		{
			//double a; //定义最快 debug下 1ms
			//double a = i; //定义赋值 2ms/million
			//a[i] = i; //索引赋值 2ms/million
			//a[i] = i; //new索引赋值 2ms/million
			// 
			//tVec.push_back(i); //70ms
			double* a = new double; //new较慢 100ms/million
			//delete a; // 总计150ms
			//a = nullptr; //置空速度忽略不计
			//double* a = new double(i); //速度接近 100ms/million
			//ptVec.push_back(a); //170ms
			//tVec.push_back(*a); //170ms
			//delete a; // 总计150ms
			
			
			//Vec3 vec; //6ms
			//Vec3 vec(i,i,i); //6ms
			//vVec.push_back(Vec3(i, i, i)); //240ms/million
			//Vec3* vec = new Vec3; //130ms 与类初始化无关
			//delete vec; //总计170ms
			
			//Vec3* vec = new Vec3(i,i,i); //130ms
			//pvVec.push_back(vec); // 230ms/million
			//delete vec; //总计170ms
			
			//a[i] = Vec3(i, i, i); //索引赋值对象 20ms
			//pvVec[i] = &vec; //定义+取址赋值 10ms

			//pvVec[i] = new Vec3(i, i, i); //new很费时 150ms

		}
	
		for (int i = 0; i < 1e5; i++)
			auto iter = a[i];
		//cout << i <<  << endl;


	auto endTime0 = std::chrono::system_clock::now();
	long long timeusing0 = std::chrono::duration_cast<std::chrono::microseconds>(endTime0 - startTime0).count();
	std::cout << "time= " << timeusing0 << "microseconds" << std::endl;


	

	ObjectPool2<Vec3> obj(1e6); //1e6
	//auto startTime = std::chrono::system_clock::now();
	for (int i = 0; i < 1e6; i++)
	{
		Vec3 pA;
		//Gene* pA;
		//Vec3 pA = obj.requestObject();
		pA.x = i;
		pA.y = i;
		pA.z = i;
		//pA->m_a=i;
		//pA->m_imp;
		//obj.returnObject(pA);
	}

	//auto name = typeid(startTime).name();
	//cout << name << endl;
	//for (int i = 0; i < 1e6; i++)
	//{
	//	Vec3* pA;
	//	//Gene* pA;
	//	pA = obj.requestObject();
	//	pA->x=i;
	//	pA->y=i;
	//	pA->z=i;
	//	//pA->m_a=i;
	//	//pA->m_imp;
	//	//obj.returnObject(pA);
	//}
	//计时
	//auto endTime = std::chrono::system_clock::now();
	//std::cout << "time:" << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() << std::endl;


	int num = 1;
	float totaltime = 0.0;
	vector<int> randList;

	for (int j = 0; j < num; j++)
	{ 
		auto startTime = std::chrono::system_clock::now();
		for (int i = 0; i < 1e6; i++)
		{
			double* a = new double(i);
			//*a = i;
			delete a;
		}
		auto endTime = std::chrono::system_clock::now();
		float timeusing = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
		std::cout << "new+del time:" << timeusing << " 毫秒" << std::endl;
		//测试结果，debug 150ms; release 50ms
		totaltime += timeusing;
	}
	//std::cout << "ave time:" << totaltime/ float(num) << " 毫秒" << std::endl;

	//分别测试 new-delete
	vector<double*> numlist;
	vector<double> randlist;
	for (int i = 0; i < 1e6; i++)
	{
		//numlist.push_back(nullptr);
		randlist.push_back(rand());

	}
	auto startTime = std::chrono::system_clock::now();
	for (int i = 0; i < 1e6; i++)
	{
		double* a = new double(i);
		numlist.push_back(a);
		//numlist[i] = &(randlist[i]); //numlist[i]=a; //push_back(double) push_back(double*) 时间一致
	}
	auto endTime = std::chrono::system_clock::now();
	float timeusing = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
	std::cout << "new time:" << timeusing << " 毫秒" << std::endl;

	auto startTime2 = std::chrono::system_clock::now();
	for (int i = 0; i < 1e6; i++)
	{
		//double* a = numlist[i];
		delete numlist[i];
		numlist[i] = nullptr;
	}
	auto endTime2 = std::chrono::system_clock::now();
	float timeusing2 = std::chrono::duration_cast<std::chrono::milliseconds>(endTime2 - startTime2).count();
	std::cout << "delete time:" << timeusing2 << " 毫秒" << std::endl;

	srand((unsigned)time(NULL)); //0-32767
	vector<int> count = {0,0,0,0,0,0,0,0,0,0};
	startTime2 = std::chrono::system_clock::now();
	for (int i = 0; i < 1e6; i++) //实践证明，随机数稳定
	{
		int ran = int(10 * rand() / 32767.0);
		switch (ran)
		{
		case 0:
		{
			//count[0]++;
			double* a11 = new double(i);
			delete a11;
			break;
		}
		case 1:
		{
			//count[1]++;
			string* str = new string("hello world");
			delete str;
			break;
		}
		case 2:
		{
			//count[2]++;
			Vec3* v = new Vec3();
			delete v;
			break;
		}
		case 3:
		{
			//count[3]++;
			double* p = new double[100];
			delete[] p;
			break;
		}
		case 4:
		{
			//count[4]++;
			char* cs = new char[64];
			delete[] cs;
			break;
		}
		case 5:
		{
			//count[5]++;
			Gene gene1;
			break;
		}
		case 6:
		{
			//count[6]++;
			Gene* gnrc = new Gene;
			delete gnrc;
			break;
		}
		case 7:
		{
			int* ptr = new int[32];
			delete ptr;
			//count[7]++;
			break;
		}
		case 8:
		{
			typedef vector<int> VECINT;
			VECINT* vec = new VECINT;
			delete vec;
			//count[8]++;
			break;
		}
		case 9:
		{
			typedef map<char*, int> MAP;
			MAP* map = new MAP;
			delete map;
			//count[9]++;
			break;
		}
		}
	}
	endTime2 = std::chrono::system_clock::now();
	timeusing2 = std::chrono::duration_cast<std::chrono::milliseconds>(endTime2 - startTime2).count();
	std::cout << "rand time:" << timeusing2 << " 毫秒" << std::endl;

	return 0;
}