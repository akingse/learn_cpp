
#include "pch.h"
/*
std::atomicΪC++11��װ��ԭ���������͡�

�ӹ����Ͽ����򵥵�˵��ԭ���������Ͳ��ᷢ�����ݾ�������ֱ�����ڶ��߳��ж����������û����������ӻ�����Դ�������͡�
��ʵ���ϣ���ҿ������Ϊ��Щԭ�������ڲ��Լ���������

ԭ�Ӳ�����һ���Ե���������ִ�еĲ���������һ���������ٷ��Ҳ��ɲ��еġ������������߳�ֻ�ܿ����������ǰ������ɺ����Դ״̬���������м�״̬���ӡ�
�ӵײ�������ԭ�Ӳ�����һЩӲ��ָ���ԭ��������Ӳ����֤�ģ�C++11��ԭ�Ӳ��������ͳһ�Ľӿڣ�����ʹ��ʱǶ��ƽ̨��صĴ�����֧�ֿ�ƽ̨ʹ�á�


��ҿ��԰�ԭ�Ӳ�������һ�֣�����Ҫ�õ������������������������Ķ��̲߳�����̷�ʽ��
ԭ�Ӳ������ڶ��߳��в��ᱻ��ϵĳ���ִ��Ƭ�Ρ�
��Ч������˵��ԭ�Ӳ���Ҫ�Ȼ������ķ�ʽЧ��Ҫ�ߡ�
�������ļ���һ�������һ������Σ���ԭ�Ӳ�����Ե�һ�㶼��һ��������
ԭ�Ӳ�����һ�㶼��ָ�����ɷָ�Ĳ�������Ҳ����˵���ֲ���״̬Ҫô����ɵģ�Ҫô��û��ɵģ������ܳ��ְ����״̬��

*/
// void(*)(void)  --��ʾһ������ֵΪvoid��û�в����ĺ���ָ��
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

//���߳������̷߳���Ϣ

int main_t2()
{
    std::thread myThread(threadFun);
    while (true)
    {
        void(*f1)() = fun.exchange(coutFun2);//ԭ�ӵ��滻ԭ�Ӷ����ֵ���������ǰ���е�ֵ
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
    bool rs = !(lhs ^ rhs); // ���

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
