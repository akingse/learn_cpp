#include "pch.h"
using namespace std;
/*
### 内存泄漏的情况

1，new without delete

2，智能指针相互持有

3，单例类

4，结构体


*/

class C
{
public:

    double* val_指针;
    std::shared_ptr<std::vector<double>> vla_智能指针;
    C()
    {
        val_指针 = new double(1.3);
        std::cout << "C()" << std::endl;
    }
    C(const C& src)//:
        //vla_智能指针(new std::vector<double>(*src.vla_智能指针));
    {
        const std::vector<double>& vc = *src.vla_智能指针;
        std::vector<double>* vp = new std::vector<double>(vc);
        //vla_智能指针.reset(vp);
        std::cout << "C(const C&)" << std::endl;
    }
    ~C()
    {
        std::cout << "~C()" << std::endl;
    }
};
int main_ml()
{
    //引用计数
//引用计数是计算机编程语言中的一种内存管理技术，是指将资源（可以是对象、内存或磁盘空间等等）的被引用次数保存起来，当被引用次数变为零时就将其释放的过程。
    shared_ptr<C> pterA(new C);


    C 于檬涵家 = C();
    //C 于檬涵老家 = 于檬涵家; // 深拷
    C& 于招财家 = 于檬涵家; // 浅拷
    C* 于招财家的快递地址 = &于招财家; // 浅拷
    C* 于檬涵家的快递地址 = &于檬涵家; // 浅拷
    return 0;
}


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



