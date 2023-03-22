#include "pch.h"
using namespace std;


class A
{
public:
    void print()
    {
        std::cout << "A" << std::endl;
    }

    int a;
    virtual ~A() {}
};

class B : virtual public A
{
public:
    void print()
    {
        std::cout << "B" << std::endl;
    }

    int b;
};

class C : virtual public A
{
public:
    void print()
    {
        std::cout << "C" << std::endl;
    }

    int c;
};

class D : public B, public C
{
public:
    void print()
    {
        std::cout << "D" << std::endl;
    }

    int d=0;
};
//----------------------------------------------------------------------------------------------

//class Base
//{
//public:
//    virtual Base& print() {
//        Base base;
//        cout << "Base print called" << endl;
//        return base;
//    }
//};
//
//class SubClass : public Base {
//
//public:
//    SubClass& print() {
//        SubClass subclass;
//        cout << "SUBClass called" << endl;
//        return subclass;
//
//    }
//};


class Base {
public:
    virtual void print1() {
        cout << "Base print1 called" << endl;
    }

    virtual void print2() {
        cout << "Base print2 called" << endl;
    }

    virtual void print3() {
        cout << "Base print3 called" << endl;
    }
    virtual ~Base() {
        cout << "Base disconstruction called" << endl;
    }
};

class SubClass : public Base 
{
public:
    void print1() {
        cout << "SubClass print1 called" << endl;
    }

    void print3() {
        cout << "SubClass print3 called" << endl;
    }

    virtual ~SubClass() {
        cout << "SubClass disconstruction called" << endl;
    }

};

// RTTI(Runtime Type Identification) 运行时类型识别
int main_rtti(int argc, char* argv[])
{
    D d;
    A* a_ptr = &d;
    B* b_ptr = &d;
    C* c_ptr = &d;

    std::cout << (typeid(d).name()) << std::endl;
    std::cout << (typeid(*a_ptr).name()) << std::endl;
    std::cout << (typeid(*b_ptr).name()) << std::endl;
    std::cout << (typeid(*c_ptr).name()) << std::endl;

    //std::cout << typeid(*a_ptr).name() << std::endl;
    //long* vtbl_A = (long*)*(long*)a_ptr;
    ////std::cout << ((std::type_info*)*(vtbl_A - 1))->name() << std::endl << std::endl;

    //std::cout << typeid(*b_ptr).name() << std::endl;
    //long* vtbl_B = (long*)*(long*)b_ptr;
    //std::cout << ((std::type_info*)*(vtbl_B - 1))->name() << std::endl << std::endl;

    //std::cout << typeid(*c_ptr).name() << std::endl;
    //long* vtbl_C = (long*)*(long*)c_ptr;
    //std::cout << ((std::type_info*)*(vtbl_C - 1))->name() << std::endl;

    Base* base = new SubClass;

    typedef void (*p)();
    ((p)(*(long long*)*(long long*)base))();
    ((p)(*((long long*)*(long long*)base + 0)))();
    ((p)(*((long long*)*(long long*)base + 1)))();
    ((p)(*((long long*)*(long long*)base + 2)))();

    long long* tail = (long long*)*((long long*)*(long long*)base + 3);
    long long* tailAdd1 = (long long*)((long long*)*(long long*)base + 4);

    delete base;


 //   //获取 对象虚表
 //   Base* base = new SubClass;
 //   //base->print();
 //   //getchar();
 //   size_t a1 = sizeof(int);
 //   size_t a2 = sizeof(long);
 //   size_t a3 = sizeof(long long);
 //   size_t a4 = sizeof(void*);

	//auto ptr1 = *(long long*)base;    //这个是指得到对象空间最开始的4字节内容，也就是虚表地址
 //   auto ptr2 = (long long*)*(long long*)base;    //这样做虚表的首地址就变成了int* 了
 //   auto ptr3 = *(long long*)*(long long*)base;    //得到这个数组的首元素了！

 //   auto ptr4 = *((long long*)*(long long*)base + 0);//    第1个虚函数
 //   auto ptr5 = *((long long*)*(long long*)base + 1);//    第2个虚函数

 //   typedef void (*p)();
 //   ((p)(*((long long*)*(long long*)base + 0)))();    //这样就可以调用了
 //   long long* pp = (long long*)*((long long*)*(long long*)base + 3);

    return 0;
}


//void typeid1(void)
//const char* TypeToName(const char* name)
//{
//    const char* __name = abi::__cxa_demangle(name, nullptr, nullptr, nullptr);
//    return __name;
//}
