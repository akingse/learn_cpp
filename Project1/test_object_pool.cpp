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
