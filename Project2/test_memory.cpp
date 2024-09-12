#include "pch.h"
using namespace std;
/*
### 内存泄漏的情况

1，new without delete

2，智能指针相互持有

3，单例类

4，结构体


*/


int& func()
{
	int* a = new int(1);
	return *a;
}

//单例类
class Manage
{
public:
    static Manage& get_instance();
    //static s_instance();
};

//单例类,只有一个实例的类

//Manage* Manage::s_instance = nullptr;

//结构体
struct C_struct
{
    double vd; //不会漏
    string ch = "cha"; //string可变长

    ~C_struct()
    {

    }
};
//delete需要指定类型

//具名要求


int main_me()
{
	int& ra = func(); //出函数栈，野指针
	ra = 2;

	char* pchar = new char;
	delete pchar; //析构函数会释放内存，但不会置空；
	pchar = nullptr;

	//深拷浅拷
	//左值右值
	int a = 123;
	int b = a;//深拷
	int& c = a;//浅拷
	int* p = &a;//浅拷
    //Manager::get_instance().m_id2name[i++] = "name";

	return 0;
}

static void test_sharedptr_1()
{
    std::shared_ptr<int> ptr = make_shared<int>(1);
    ptr.reset(); //shared_ptr释放内存
    return;
}


static int enrol = []()->int
    {
        test_sharedptr_1();
        cout << "test_memory_leak finished.\n" << endl;
        return 0;
    }();
