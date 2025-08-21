#pragma once
//ʹ�ô������-�����ྲ̬
//static std::map<std::type_index, void*> m_id2ptr;
void* __getImplementation(std::type_index id);

 //<summary>
 //���ι�����ʵ���ࣨ����ࣩ
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

//����ʽ��Lazy Initialization
namespace chatgpt1
{
    //�̲߳���ȫ
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
    //�̰߳�ȫ
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

//����ʽ��Eager Initialization��
namespace chatgpt3
{
    //��������ʱ��ʵ�������̰߳�ȫ
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
    //C++11�����std::call_once���Լ��̰߳�ȫ�ĵ���ʵ�֣�
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
    //���ú����ľֲ���̬����������ʵ�ֵ�������֤�̰߳�ȫ�����ӳٳ�ʼ����
    class Singleton 
    {
    public:
    static Singleton& getInstance() 
    {
        static Singleton instance; // �ֲ���̬����
        return instance;
    }

    private:
        Singleton() {}
        Singleton(const Singleton&) = delete; // ��ֹ��������
        Singleton& operator=(const Singleton&) = delete; // ��ֹ��ֵ
    };
    /*
    �ֲ���̬�������������������䶨��ĺ����ڣ��ⲿ�޷�ֱ�ӷ��ʣ��������ڷ�װ��
    �ֲ���̬�������������ڴӵ�һ�ε��ð������ĺ�����ʼ��ֱ����������������ڴ�ֻ�����һ�Σ���ֻ��ʼ��һ�Ρ�
    ��C++11�����߰汾�У��ֲ���̬�����ĳ�ʼ�����̰߳�ȫ��
    */
}
