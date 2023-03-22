#include "pch.h"
using namespace std;

/*
* typedef int PARA
* ��Ȼ�ڹ����ϣ�typedef���Կ���һ����int PARA����Ķ��������﷨��typedef���ڴ洢������˵������
����ϸ���˵��typedef int PARA������һ��������������
typedef int a[10];
typedef void (*p)(void);
��a����Ϊ����10��intԪ�ص���������ͱ�����p��һ�ֺ���ָ������ͱ�����

*/
typedef void* (*PTRCreateObject)(void); //����ָ��/��ָ��

class ClassFactory //�����࣬�ṩ typeindex <=> *fun ӳ��map���Լ�set/get����
{
private:
    map<string, PTRCreateObject> m_classMap;
    map<type_index, PTRCreateObject> m_class_map;
    ClassFactory() {}; //���캯��˽�л�

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



class RegisterAction //ע����
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

// brief: ������ʵ���Ļص�����
TestClassB* createObjTestClassB() 
{
    return new TestClassB;
}

// ע�ᶯ�����ȫ��ʵ��
RegisterAction g_creatorRegisterTestClassB("TestClassB", (PTRCreateObject)createObjTestClassB);




//�궨��
/*
#����ʾ����Ӧ�����ַ�����������˵�����൱���ں���������߸�����һ��˫����ʹ����һ���ַ���
##����ʾ�Ѻ��������궨����������еı�ʶ��������һ���γ�һ���µı�ʶ���������Ƿ���ƴ���ţ�����������ƴ���ţ�����������ƴ����
#@����ʾ�����ַ���Ƿ��任Ϊ���ַ������ӵ�����
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
