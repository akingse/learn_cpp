
#include "pch.h"
/*
std::atomic为C++11封装的原子数据类型。

从功能上看，简单地说，原子数据类型不会发生数据竞争，能直接用在多线程中而不必我们用户对其进行添加互斥资源锁的类型。
从实现上，大家可以理解为这些原子类型内部自己加了锁。

原子操作是一种以单个事务来执行的操作，这是一个“不可再分且不可并行的”操作，其他线程只能看到操作完成前或者完成后的资源状态，不存在中间状态可视。
从底层来看，原子操作是一些硬件指令，其原子性是由硬件保证的，C++11对原子操作抽象出统一的接口，避免使用时嵌入平台相关的代码来支持跨平台使用。


大家可以把原子操作理解成一种：不需要用到互斥量加锁（无锁）技术的多线程并发编程方式。
原子操作：在多线程中不会被打断的程序执行片段。
从效率上来说，原子操作要比互斥量的方式效率要高。
互斥量的加锁一般是针对一个代码段，而原子操作针对的一般都是一个变量。
原子操作，一般都是指“不可分割的操作”；也就是说这种操作状态要么是完成的，要么是没完成的，不可能出现半完成状态。

*/
// void(*)(void)  --表示一个返回值为void，没有参数的函数指针
//std::atomic<void(*)(int a)> g;
std::atomic<void(*)()> fun;
// int(*p)(int, int);

void coutFun1()
{
    std::cout << "data1--------" << std::endl;;
}

void coutFun2()
{
    std::cout << "data2" << std::endl;;
}

void threadFun()
{
    while (true)
    {
        Sleep(2000);
        fun = coutFun1;
        Sleep(2000);
        fun = coutFun2;
    }
}

//子线程向主线程发消息

int main_t2()
{
    std::thread myThread(threadFun);
    while (true)
    {
        void(*f1)() = fun.exchange(coutFun2);//原子地替换原子对象的值并获得它先前持有的值
        if (f1!=nullptr)
            f1();
        Sleep(100);
        std::cout << std::endl;
    }
    myThread.join();
    return 0;
}


// atomic::exchange example
#include <iostream>       // std::cout
#include <atomic>         // std::atomic
#include <thread>         // std::thread
#include <vector>         // std::vector

std::atomic<bool> ready(false);
std::atomic<bool> winner(false);

void count1m(int id) 
{
    while (!ready) {}                  // wait for the ready signal
    for (int i = 0; i < 1000000; ++i) {}   // go!, count to 1 million
    if (!winner.exchange(true)) 
    { std::cout << "thread #" << id << " won!\n"; }
};

int main_t1()
{
    std::vector<std::thread> threads;
    std::cout << "spawning 10 threads that count to 1 million...\n";
    for (int i = 1; i <= 10; ++i) threads.push_back(std::thread(count1m, i));
    ready = true;
    for (auto& th : threads) 
        th.join();

    return 0;
}

#include <thread>
#include <atomic>
#include <iostream>
#include <list>

using namespace std;

//atomic_int iCount(0);
int iCount(0);

void threadfun1()
{
    for (int i = 0; i < 10; i++)
    {
        printf("iCount:%d\r\n", iCount++);
    }
}
void threadfun2()
{
    for (int i = 0; i < 10; i++)
    {
        printf("iCount:%d\r\n", iCount--);
    }
}


int test_atomic()
{
    bool lhs = 0;
    bool rhs = 1;
    bool rs = !(lhs ^ rhs); // 异或

    std::list<thread> lstThread;
    for (int i = 0; i < 2; i++)
    {
        lstThread.push_back(thread(threadfun1));
    }
    for (int i = 0; i < 2; i++)
    {
        lstThread.push_back(thread(threadfun2));
    }

    for (auto& th : lstThread)
    {
        th.join();
    }

    printf("finally iCount:%d\r\n",  iCount);
    //int x = iCount.load(memory_order_relaxed);
    //printf("finally iCount:%d\r\n", x);
    return 0;
}

static int _enrol = []()->int
{
    //test_alg();
    return 0;
}();
