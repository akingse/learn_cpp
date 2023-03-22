#include "pch.h"
class B
{
public:
    int m_imp;
    B(int b=-1) :m_imp(b)
    { 
        std::cout << "B Constructor" << std::endl; 
    }
    B(const B& b) 
    { 
        std::cout << "B copy Constructor" << std::endl; 
    }
};

class A
{
private:
    B* m_b;
public:
    A(int a=0) : 
        m_b(new B(a)) //默认构造
    { 
        std::cout << "A Constructor" << std::endl; 
    }
    A(const A& src) 
        //m_b(new B(*(src.m_b))) //拷贝构造
    {
        m_b = new B;
        //m_b = src.m_b; // 拷指针
        *m_b = *src.m_b; // 拷贝值
        /*
        以下情况都会调用拷贝构造函数：
（1）一个对象以值传递的方式传入函数体
（2）一个对象以值传递的方式从函数返回
（3）一个对象需要通过另外一个对象进行初始化。
        */
        std::cout << "A Copy Constructor" << std::endl;
    }

    //std::move函数可以以非常简单的方式将左值转换为右值引用。
    //通过std::move，可以避免不必要的拷贝操作。std::move是为性能而生。
    A(A&& src) : 
        m_b(src.m_b) //移动构造
    {
        src.m_b = nullptr;
        std::cout << "A Move Constructor" << std::endl;
    }
    
    A& operator=(A&& src) noexcept //移动赋值
    {
        if (this == &src)
            return *this;

        delete m_b;
        m_b = src.m_b;
        src.m_b = nullptr;
        std::cout << "operator=(const A&& src)" << std::endl;
        return *this;
    }
    A& operator=(const A& src) noexcept //赋值
    {
        if (this == &src)
            return *this;

        delete m_b;
        m_b = new B(*(src.m_b));
        std::cout << "operator=(const A& src)" << std::endl;
        return *this;
    }

    ~A() 
    { 
        if (m_b);
            delete m_b; 
    }

};

static A getA()
{
    A a(1);
    //std::cout << "=======================" << std::endl;
    return a;
}

int main_c()
{
    //int si = sizeof(int);
    //si = sizeof(long);
    //si = sizeof(long long);
    //A a;
    //a= getA();
    //std::vector<A> list; //内存是否泄漏
    //for (int i = 0; i < 1e1; i++)
    //{
    //    A a0(0);
    //    A a1(1);
    //    list.push_back(a0);
    //    list.push_back(a1);
    //}
    //int siz = sizeof(list);

    //getA();
    A a = getA();//移动构造
    std::cout << "================================================" << std::endl;
    A a1(a);//拷贝构造，对象作为入参


    std::cout << "================================================" << std::endl;
    A a2(std::move(a1));//移动构造

    std::cout << "================================================" << std::endl;
    a2 = getA();//移动赋值

    return 0;

    std::cout << "================================================" << std::endl;
    a2 = a1;//拷贝赋值
    return 0;

}