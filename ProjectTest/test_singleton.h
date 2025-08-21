#pragma once
//使用纯虚基类-单例类静态
//static std::map<std::type_index, void*> m_id2ptr;
void* __getImplementation(std::type_index id);

 //<summary>
 //几何管理类实现类（虚基类）
 //</summary>
//class BPBfaPlacedHandle;
typedef size_t BPBfaPlacedHandle;

class __ParaGeometryManagerImplementation 
    //public __Implementation<__ParaGeometryManagerImplementation>
{
private:
    // make singleton construct
    __ParaGeometryManagerImplementation(const __ParaGeometryManagerImplementation&) = delete;
    __ParaGeometryManagerImplementation(__ParaGeometryManagerImplementation&&) = delete;
    void operator=(const __ParaGeometryManagerImplementation&) = delete;
    void operator=(__ParaGeometryManagerImplementation&&) = delete;
private:
    static __ParaGeometryManagerImplementation* s_instance;
public:
    //__ParaGeometryManagerImplementation() {};
    //virtual ~__ParaGeometryManagerImplementation() {};
    __ParaGeometryManagerImplementation()
    {
        //__setImplementation(typeid(T), this);
    }
    virtual ~__ParaGeometryManagerImplementation()
    {
		//if (s_instance != nullptr)
  //      {
  //          delete s_instance;
  //          s_instance == nullptr;
  //      }
    }

    inline static bool ready()
    {
        return s_instance != nullptr;
    }
    inline static __ParaGeometryManagerImplementation& instance()
    {
        __ParaGeometryManagerImplementation* s_instance = (__ParaGeometryManagerImplementation*)__getImplementation(typeid(__ParaGeometryManagerImplementation));

        if (!s_instance)
        return *s_instance;
    }

public:
	virtual BPBfaPlacedHandle getPlacedHandleByEntityId(const size_t entityId) const = 0;
	virtual size_t getEntityIdByPlacedHandle(const BPBfaPlacedHandle& handle) const = 0;
};

//chatgpt

//懒汉式（Lazy Initialization
namespace chatgpt1
{
    //线程不安全
    class Singleton 
    {
    public:
    static Singleton* getInstance() {
        if (!instance)
            instance = new Singleton();
        return instance;
    }

    private:
        Singleton() {}
        static Singleton* instance;
    };

    Singleton* Singleton::instance = nullptr; //in cpp

}

namespace chatgpt2
{
    //线程安全
    class Singleton 
    {
    public:
    static Singleton* getInstance() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!instance)
            instance = new Singleton();
        return instance;
    }

    private:
        Singleton() {}
        static Singleton* instance;
        static std::mutex mutex_;
    };

    Singleton* Singleton::instance = nullptr;

}

//饿汉式（Eager Initialization）
namespace chatgpt3
{
    //程序启动时就实例化，线程安全
    class Singleton 
    {
    public:
    static Singleton* getInstance() 
    {
        return instance;
    }

    private:
        Singleton() {}
        static Singleton* instance;
    };

    Singleton* Singleton::instance = new Singleton();
}

namespace chatgpt4
{
    //C++11引入的std::call_once可以简化线程安全的单例实现：
    class Singleton 
    {
    public:
    static Singleton& getInstance() {
        std::call_once(initFlag, initInstance);
        return *instance;
    }

    private:
        Singleton() {}
        static void initInstance() {
            instance = new Singleton();
        }
        static Singleton* instance;
        static std::once_flag initFlag;
    };

    Singleton* Singleton::instance = nullptr;
    std::once_flag Singleton::initFlag;
}

namespace chatgpt5
{
    //利用函数的局部静态变量的特性实现单例，保证线程安全并且延迟初始化：
    class Singleton 
    {
    public:
    static Singleton& getInstance() 
    {
        static Singleton instance; // 局部静态变量
        return instance;
    }

    private:
        Singleton() {}
        Singleton(const Singleton&) = delete; // 禁止拷贝构造
        Singleton& operator=(const Singleton&) = delete; // 禁止赋值
    };
    /*
    局部静态变量的作用域限制在其定义的函数内，外部无法直接访问，这有助于封装。
    局部静态变量的生命周期从第一次调用包含它的函数开始，直到程序结束。它的内存只会分配一次，且只初始化一次。
    在C++11及更高版本中，局部静态变量的初始化是线程安全的
    */
}
