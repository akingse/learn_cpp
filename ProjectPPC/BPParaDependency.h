#pragma once
namespace para
{

    //singleton template, copy from BPParametric
    class DependencyRegistry
    {
    private:
        DLLEXPORT_PPC DependencyRegistry() = default;
        DLLEXPORT_PPC ~DependencyRegistry() = default;
        DependencyRegistry(const DependencyRegistry&) = delete;
        DependencyRegistry(DependencyRegistry&&) = delete;

    private:
        struct FunctionPointer
        {
            //FunctionPointer() = default;
            void* m_value = nullptr;
            std::type_index m_type = typeid(void);
        };
        // not export if using this project only
        DLLEXPORT_PPC static std::map<std::string, FunctionPointer> sm_implementations;

    public:
        static DependencyRegistry& getInstance()
        {
            static DependencyRegistry instance;
            return instance;
        }

        template<class T> //registerImplementation
        bool set(const std::string& name, T* impl)
        {
            if (sm_implementations.find(name) != sm_implementations.end())
                return false;
            FunctionPointer ptr;
            ptr.m_value = impl;
            ptr.m_type = typeid(T);
            //std::cout << ptr.m_type.name() << std::endl;
            sm_implementations.insert({ name,ptr });
            return true;
        }

        template<class T> //callImplementation
        T* get(const std::string& name) const
        {
            if (sm_implementations.find(name) == sm_implementations.end())
                return nullptr;
            FunctionPointer ptr = sm_implementations.at(name);
            if (ptr.m_type != typeid(T) || ptr.m_value == nullptr) //普通函数指针无法和function<>比较
                return nullptr;
            return (T*)ptr.m_value;
        }
    };

    //sharedptr version, copy from chatgpt
    class DependencyInversion
    {
    private:
        DLLEXPORT_PPC DependencyInversion() = default;
        DLLEXPORT_PPC ~DependencyInversion() = default;
        DependencyInversion(const DependencyInversion&) = delete;
        DependencyInversion(DependencyInversion&&) = delete;

    private:
        template<typename T>
        struct Holder
        {
            T m_value;
            Holder(T value) : m_value(value) {}
        };

        // not export if using this project only
        DLLEXPORT_PPC static std::map<std::string, std::shared_ptr<void>> sm_implementations;
        DLLEXPORT_PPC static std::mutex sm_mutex;

    public:
        static DependencyInversion& getInstance()
        {
            static DependencyInversion instance;
            return instance;
        }

        template<class T> //registerImplementation
        bool set(const std::string& name, T impl)
        {
            std::lock_guard<std::mutex> lock(sm_mutex);
            if (sm_implementations.find(name) != sm_implementations.end())
                return false;
            sm_implementations[name] = std::make_shared<Holder<T>>(impl);
            return true;// srore holder
        }

        template<class T>
        T* get(const std::string& name)
        {
            std::lock_guard<std::mutex> lock(sm_mutex);
            auto it = sm_implementations.find(name);
            if (it == sm_implementations.end())
                return nullptr;
            auto holder = std::static_pointer_cast<Holder<T>>(it->second);
            //return holder ? std::make_shared<T>(holder->m_value) : nullptr;
            return holder ? &holder->m_value : nullptr;
        }

    };
}

namespace para
{
    class TreeNode;
    class TreeNodePtr;
    // Interface 接口
    class Interface 
    {
    public:
        //virtual void doSomething() = 0;
        virtual std::shared_ptr<TreeNode> create() = 0;
        virtual std::vector<unsigned char> serial(const std::shared_ptr<TreeNode>& csg) = 0;
        virtual std::shared_ptr<TreeNode> deserial(const std::vector<unsigned char>& data) = 0;
    };

    //必须使用智能指针，否则typeid不认为提前声明的class相同
    DLLEXPORT_PPC std::vector<unsigned char> serializition(const std::shared_ptr<TreeNodePtr>& node);
    DLLEXPORT_PPC std::shared_ptr<TreeNodePtr> deserializition(const std::vector<unsigned char>& data);
}
