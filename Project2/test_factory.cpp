#include "pch.h"
using namespace std;

/*
* typedef int PARA
* 虽然在功能上，typedef可以看作一个跟int PARA分离的动作，但语法上typedef属于存储类声明说明符，
因此严格来说，typedef int PARA整个是一个完整的声明。
typedef int a[10];
typedef void (*p)(void);
把a声明为具有10个int元素的数组的类型别名，p是一种函数指针的类型别名。

*/
typedef void* (*PTRCreateObject)(void); //函数指针/类指针

class ClassFactory //单例类，提供 typeindex <=> *fun 映射map，以及set/get方法
{
private:
    map<string, PTRCreateObject> m_classMap;
    map<type_index, PTRCreateObject> m_class_map;
    ClassFactory() {}; //构造函数私有化

public:
    void* getClassByName(const string& className);
    void registClass(const string& name, PTRCreateObject method);
    //by typeindex
    void registClassByTypeindex(const type_index& typeName, PTRCreateObject method);
    void* getClassByTypeindex(const type_index& typeName);
    static ClassFactory& getInstance();
};

void* ClassFactory::getClassByName(const string& className) 
{
    map<string, PTRCreateObject>::const_iterator iter;
    iter = m_classMap.find(className);
    if (iter == m_classMap.end())
        return NULL;
    else
        return iter->second();
}

void ClassFactory::registClass(const string& name, PTRCreateObject method) 
{
    m_classMap.insert(pair<string, PTRCreateObject>(name, method));
}

ClassFactory& ClassFactory::getInstance() 
{
    static ClassFactory sLo_factory;
    return sLo_factory;
}

void ClassFactory::registClassByTypeindex(const type_index& typeName, PTRCreateObject method)
{
    m_class_map.insert(pair<type_index, PTRCreateObject>(typeName, method));
}

void* ClassFactory::getClassByTypeindex(const type_index& typeName)
{
    map<type_index, PTRCreateObject>::const_iterator iter;
    iter = m_class_map.find(typeName);
    if (iter == m_class_map.end())
        return nullptr;
    else
        return iter->second();
}





//--------------------------------------------------------



class RegisterAction //注册类
{
public:
    RegisterAction(string className, PTRCreateObject ptrCreateFn) 
    {
        ClassFactory::getInstance().registClass(className, ptrCreateFn);
    }
};

// test class B
class TestClassB {
public:
    void m_print() {
        cout << "hello TestClassB" << endl;
    };
};

// brief: 创建类实例的回调函数
TestClassB* createObjTestClassB() 
{
    return new TestClassB;
}

// 注册动作类的全局实例
RegisterAction g_creatorRegisterTestClassB("TestClassB", (PTRCreateObject)createObjTestClassB);




//宏定义
/*
#：表示将对应变量字符串化，简单来说就是相当于在宏变量的两边各加上一个双引号使其变成一个字符串
##：表示把宏参数名与宏定义代码序列中的标识符连接在一起，形成一个新的标识符。可以是符号拼符号，可以是数字拼符号，可以是数字拼数字
#@：表示将单字符标记符变换为单字符，即加单引号
*/

#define REGISTER(className) 											\
	className* objectCreator##className(){     							\
        return new className;                                         	\
    }                                                                  	\
    RegisterAction g_creatorRegister##className(                        \
		#className,(PTRCreateObject)objectCreator##className)


//-----------------------------------------------------
// 


// test class
class TestClass {
public:
    void m_print() {
        cout << "hello TestClass" << endl;
    };
};
REGISTER(TestClass);

int main_factory()
{
    TestClass* ptrObj = (TestClass*)ClassFactory::getInstance().getClassByName("TestClass");
    ptrObj->m_print();

    TestClass* ptrIdx = (TestClass*)ClassFactory::getInstance().getClassByTypeindex(typeid(TestClass));
    ptrIdx->m_print();

    type_index a1 = typeid(int);
    type_index a2 = typeid(int*);


    cout << endl;
    return 0;
}
