#pragma once

// Registry 类，用于注册和管理实现
// 
//singleton
//class Interface;

class DependencyRegistry 
{
private:

    struct FunctionPointer
    {
        //FunctionPointer() = default;
        void* m_value = nullptr;
        std::type_index m_type = typeid(void);
    };

    //std::vector<Interface*> implementations;
    DLLEXPORT_1 static std::map<std::string, FunctionPointer> sm_implementations;

public:
    static DependencyRegistry& getInstance() 
    {
        static DependencyRegistry instance;
        return instance;
    }

    template<class T> //registerImplementation
    static void set(const std::string& name, T* impl)
    {
        if (sm_implementations.find(name) != sm_implementations.end())
            return;
        //implementations.push_back(impl);
        FunctionPointer ptr;// = new FunctionPointer();
        ptr.m_value = impl;
        ptr.m_type = typeid(T);
        sm_implementations.insert({ name,ptr });
    }

    template<class T> //callImplementation
    static T* get(const std::string& name)
    {
        if (sm_implementations.find(name) == sm_implementations.end())
            return nullptr;
        FunctionPointer ptr = sm_implementations.at(name);
        if (ptr.m_type != typeid(T))
            return nullptr;
        return (T*)ptr.m_value;
    }


};


class TreeNode;
// Interface 接口
class Interface 
{
public:
    //virtual void doSomething() = 0;
    virtual std::shared_ptr<TreeNode> create() = 0;
    virtual std::vector<unsigned char> serial(const std::shared_ptr<TreeNode>& csg) = 0;
    virtual std::shared_ptr<TreeNode> deserial(const std::vector<unsigned char>& data) = 0;
};

