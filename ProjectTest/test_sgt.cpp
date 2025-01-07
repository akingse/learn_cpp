#include "pch.h"

using namespace std;
class Only
{
private:
    Only() {}   //构造函数是私有的，这样就不能在其它地方创建该实例
    static Only* p;  //定义一个唯一指向实例的静态指针，并且是私有的。
    static int m_b;
    /*
    静态数据被看作类类型内的全局变量。对于非静态数据成员，每个对象都有自己的副本，而静态数据成员在整个类种只有一份，
    可以被类的所有对象共享访问。与全局变量相比，静态数据成员有以下两个优点：
    静态数据成员没有进入程序的全局作用域，只是在类的作用域内；
    可以实现信息隐藏，静态成员可以是private成员，而全局变量则不能；

    static数据成员不是属于某个特定对象，因而不能在构造函数中初始化。
    static数据成员在类定义之外初始化，在定义时要使用类名字限定静态成员，但不需要重复出现static关键字。

    静态数据成员从属于类，非静态数据成员从属于对象；
    静态数据成员存放于静态存储区，由本类的所有对象共享，生命期不依赖于对象；
    非静态数据成员独立存放于各个对象中，生命期依赖于对象，随对象的创建而存在，随对象的销毁而消亡；

    */

public:
    static Only* GetInstance() //定义一个公有函数，可以获取这个唯一的实例，并且在需要时创建该实例。
    {
        if (p == NULL)  //判断是否第一次调用  
            p = new Only;
        return p;
    }
    static void show()
    {
        cout << m_b << endl;
    }
    static int setB(int b)
    {
        m_b = b;
        return m_b;
    }


    class Garbo
    {
    public:
        ~Garbo()
        {
            if (Only::p)
            {
                delete Only::p;
                cout << "delete only success " << endl;
            }
        }
    };
    static Garbo carbo;
};


int Only::m_b = 66;  //static定义的数据成员必须在类外初始化，因为它是整个类的一部分，而不是属于某个对象。
Only* Only::p = NULL;
Only::Garbo Only::carbo;


int main_singleton()
{


    Only* a1 = Only::GetInstance();
    cout << a1 << endl;
    a1->show();
    Only* a2 = Only::GetInstance();
    cout << a2 << endl;
    a2->show();

    a1->setB(11);
    cout << a1 << endl;
    a1->show();

    return 0;
}



