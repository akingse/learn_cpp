#pragma once
//using namespace ppc;

//singleton, template
class DependencyRegistry 
{
private:
    DependencyRegistry() = default;
    ~DependencyRegistry() = default;
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
    DLLEXPORT_1 static std::map<std::string, FunctionPointer> sm_implementations;

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
        if (ptr.m_type != typeid(T) || ptr.m_value == nullptr) //��ͨ����ָ���޷���function<>�Ƚ�
            return nullptr;
        return (T*)ptr.m_value;
    }
};

//python parametric component
namespace ppc
{
    class TreeNode;
    class TreeNodePtr;
    // Interface �ӿ�
    class Interface 
    {
    public:
        //virtual void doSomething() = 0;
        virtual std::shared_ptr<TreeNode> create() = 0;
        virtual std::vector<unsigned char> serial(const std::shared_ptr<TreeNode>& csg) = 0;
        virtual std::shared_ptr<TreeNode> deserial(const std::vector<unsigned char>& data) = 0;
    };

    //����ʹ������ָ�룬����typeid����Ϊ��ǰ������class��ͬ
    DLLEXPORT_1 std::vector<unsigned char> serializition(const std::shared_ptr<TreeNodePtr>& node);
    DLLEXPORT_1 std::shared_ptr<TreeNodePtr> deserializition(const std::vector<unsigned char>& data);
}