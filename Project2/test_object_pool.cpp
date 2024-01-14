#include "pch.h"

using namespace std;

//对象池模式（Object Pool Pattern）

template<class T, size_t nPoolSize>
class PPCObjectPool
{
private:
	class NodeHeader 
	{
	public:
		int nID;//内存块编号
		int nRef; //引用次数
		NodeHeader* pNext; // 下一块位置
		bool bPool; //是否在内存池中

	private:
		//预留
		char d1;
		char d2;
	};

	NodeHeader* _pHeader;
	//对象池内存缓冲区地址
	char* _pBuffer;
	std::mutex _mutex;

public:

	PPCObjectPool()
	{
		_pBuffer = nullptr;
	}
	~PPCObjectPool()
	{
		if (!_pBuffer)
			delete[] _pBuffer;
	}

public:
	//初始化对象池
	void initPool()
	{
		assert(nullptr == _pBuffer);
		if (nullptr != _pBuffer)
			return;

		//计算对象池的大小
		//string type = typeid(T).name();
		//int siz = sizeof(T);
		//int sizn = sizeof(NodeHeader); //24

		size_t realSize = sizeof(T) + sizeof(NodeHeader);
		size_t n = nPoolSize * realSize;
		_pBuffer = new char[n];
		//初始化对象池
		_pHeader = (NodeHeader*)_pBuffer;
		_pHeader->bPool = true;
		_pHeader->nID = 1;
		_pHeader->nRef = 0;
		_pHeader->pNext = nullptr;
		NodeHeader* pTemp1 = _pHeader;
		for (size_t n = 2; n < nPoolSize; n++)
		{
			NodeHeader* pTemp2 = (NodeHeader*)(_pBuffer + (n * realSize));
			pTemp2->bPool = true;
			pTemp2->nID = n;
			pTemp2->nRef = 0;
			pTemp2->pNext = nullptr;
			pTemp1->pNext = pTemp2;
			pTemp1 = pTemp2;
		}
	}
	//申请对象
	void* newObjectMemory(size_t Size) 
	{
		if (nullptr == _pBuffer)
		{
			initPool();
		}
		std::lock_guard<std::mutex> lg(_mutex);

		NodeHeader* pReturn = nullptr;
		if (nullptr == _pHeader) 
		{
			pReturn = (NodeHeader*)new char(sizeof(T) + sizeof(NodeHeader));
			pReturn->bPool = false;
			pReturn->nID = -1;
			pReturn->nRef = 1;
			pReturn->pNext = nullptr;
		}
		else 
		{
			pReturn = _pHeader;
			_pHeader = _pHeader->pNext;
			//断言
			assert(0 == pReturn->nRef);
			pReturn->nRef = 1;
		}
		//返回的内存空间也没有NodeHeader的包头
		return ((char*)pReturn + sizeof(NodeHeader));
	}
	//释放对象
	void deleteObjectMemory(void* pMem)
	{
		//传进来的值没有NodeHeader的包头
		//所以传进来的指针需要向左移动sizeof（NodeHeader）个字节
		NodeHeader* pBlock = (NodeHeader*)((char*)pMem - sizeof(NodeHeader));
		assert(1 == pBlock->nRef);
		if (pBlock->bPool)
		{
			std::lock_guard<std::mutex> lg(_mutex);
			if (--pBlock->nRef != 0)
				return;
			pBlock->pNext = _pHeader;
			_pHeader = pBlock;
		}
		else
		{
			if (--pBlock->nRef != 0)
				return;
			delete[] pBlock;
		}

	}
};


template <class T, size_t nPoolSize>
class PPCObjectPoolTemp
{
private:
	//typedef PPCObjectPool<T, nPoolSize> classTypePool;
	static PPCObjectPool<T, nPoolSize>& objectPool()
	{
		static PPCObjectPool<T, nPoolSize> sPool;//静态池
		return sPool;
	}

public:
	void* operator new(size_t size)
	{
		return objectPool().newObjectMemory(size);
	}

	void operator delete(void* p)
	{
		objectPool().deleteObjectMemory(p);
	}

	
	template<typename ...Args> // 模板函数,不定参数可传入
	static T* createObject(Args ... args)
	{
		T* obj = new T(args...);
		return obj;
	}

	static void deleteObject(T* obj)
	{
		delete obj;
	}



};


//class A和B申请对象池，当用户申请对象A或者B时，可以直接从池中申请，当申请其他对象时，需要直接向内存申请。
//当池中的内存用完时，也可以向内存申请。（主要思想与内存池相似）

const int times = 1e1;

class A :public PPCObjectPoolTemp <A, times>
{
private:
	int Anum; 
public:
	A() 
	{
		Anum = 0;
		printf("A created\n");
	}
	~A() 
	{
		printf("A deleted\n");
	}
};


class B :public PPCObjectPoolTemp<B, times>
{
public:
	int num;
	//double dou=1;
	int num2;
public:
	B(int Num) 
	{
		num = Num;
		//printf("B created\n");
	}

	~B() 
	{
		//printf("B deleted\n");
	}
};


int main_main_pool()
{
	int a = 123;
	char c = a; //the ascii of 123
	A* a2 = A::createObject();
	A::deleteObject(a2);

	/*B* bb = new B(times);
	auto startTime = std::chrono::system_clock::now();

	for (int i = 0; i < times; i++)
	{
		bb->createObject(i);
		Sleep(1000);
	}
	auto endTime = std::chrono::system_clock::now();
	std::cout << "time:" << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() << std::endl;

	delete bb;*/

	
	B* b2 = B::createObject(20);
	B::deleteObject(b2);

	return 0;

	//public class Client 
	//{
	//	public static void main(String[] args) 
	//	{ 
	//		ObjectPool pool = new ObjectPool(10, 50);        
	//		IPooledObject object = pool.borrowObject();        
	//		object.operation();        
	//		pool.returnObject(object);        
	//		System.out.println(); 
	//	}    
	//	//抽象对象    
	//	interface IPooledObject 
	//	{        void operation();    }    
	//	//具体对象    
	//	static class ConcretePoolObject implements IPooledObject 
	//	{        
	//		public void operation() 
	//		{            
	//			System.out.println("doing");        
	//		}    
	//	}    
	//	//对象池    
	//	static class ObjectPool 
	//	{        private int step = 10;                      
	//	//当对象不够用的时候，每次扩容的数量        
	//	private int minCount;        
	//	private int maxCount;        
	//	private Vector<IPooledObject> returneds;     
	//	//保存未借出的对象        
	//	private Vector<IPooledObject> borroweds;     
	//	//保存已被借出的对象        
	//	//初始化对象池        
	//	public ObjectPool(int minCount,int maxCount)
	//	{            
	//		borroweds = new Vector<IPooledObject>();            
	//		returneds = new Vector<IPooledObject>();            
	//		this.minCount = minCount;           
	//		this.maxCount = maxCount;            
	//		refresh(this.minCount);        
	//	}        
	//	//因为内部状态具备不变性，所以作为缓存的键        
	//	public IPooledObject borrowObject() {            
	//		IPooledObject next = null;            
	//		if(returneds.size() > 0)
	//		{                
	//			Iterator<IPooledObject> i = returneds.iterator();                
	//			while (i.hasNext())
	//			{                    
	//				next = i.next();                    
	//				returneds.remove(next);                    
	//				borroweds.add(next);                    
	//				return next;                
	//			}            
	//		}
	//		else
	//		{                
	//			//计算出剩余可创建的对象数                
	//			int count = (maxCount - minCount);                
	//			//剩余可创建的数量大于单次固定创建的对象数 //则再初始化一批固定数量的对象                  
	//			refresh(count > step ? step : count);            
	//		}            
	//		return next;        }        
	//	//不需要使用的对象归还重复利用        
	//	public void returnObject(IPooledObject pooledObject)
	//	{            returneds.add(pooledObject);            
	//		if(borroweds.contains(pooledObject))
	//		{                
	//			borroweds.remove(pooledObject);            
	//		}
	//	}        
	//	private void refresh(int count)
	//	{            
	//		for (int i = 0; i < count; i++) 
	//		{                
	//				returneds.add(new ConcretePoolObject());            
	//		}        
	//	}    
	//	
	//}

}

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
	Test(double a = 1) :m_a(a) {}
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
	vector<int> count = { 0,0,0,0,0,0,0,0,0,0 };
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
		//case 5:
		//{
		//	//count[5]++;
		//	Gene gene1;
		//	break;
		//}
		//case 6:
		//{
		//	//count[6]++;
		//	Gene* gnrc = new Gene;
		//	delete gnrc;
		//	break;
		//}
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