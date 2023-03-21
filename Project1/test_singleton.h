#pragma once
class test_singleton
{
};

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