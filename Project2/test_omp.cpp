#include "pch.h"
//https://zhuanlan.zhihu.com/p/397670985

#include<omp.h>
#include<iostream>
using namespace std;
static int main1()
{
#pragma omp parallel
	{
		cout << "Hello, world!" << endl;
	}
}

int test0()
{
    int numThreads = 0;
#pragma omp parallel
    {
        // 获取线程数量
#pragma omp master
        {
            numThreads = omp_get_num_threads();
            std::cout << "线程数量：" << numThreads << std::endl;
        }

        // 获取当前线程编号
        int threadID = omp_get_thread_num();
        std::cout << "Hello, World! 我是线程 " << threadID << std::endl;
    }

    return 0;
}

int sharedData = 0;
omp_lock_t mpLock;

void updateSharedData() {
    // 加锁
    omp_set_lock(&mpLock);

    // 访问共享数据
    sharedData += 1;
    std::cout << "线程 " << omp_get_thread_num() << " 更新了共享数据：" << sharedData << std::endl;

    // 解锁
    omp_unset_lock(&mpLock);
}

int test1()
{
    // 初始化互斥锁
    omp_init_lock(&mpLock);

#pragma omp parallel num_threads(4)
    {
        updateSharedData();
    }

    // 销毁互斥锁
    omp_destroy_lock(&mpLock);

    return 0;
}

static int enrol = []()->int
{
    //test0();
    cout << "test test_omp finish" << endl;
    return 0;
}();

