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
        // ��ȡ�߳�����
#pragma omp master
        {
            numThreads = omp_get_num_threads();
            std::cout << "�߳�������" << numThreads << std::endl;
        }

        // ��ȡ��ǰ�̱߳��
        int threadID = omp_get_thread_num();
        std::cout << "Hello, World! �����߳� " << threadID << std::endl;
    }

    return 0;
}

int sharedData = 0;
omp_lock_t mpLock;

void updateSharedData() {
    // ����
    omp_set_lock(&mpLock);

    // ���ʹ�������
    sharedData += 1;
    std::cout << "�߳� " << omp_get_thread_num() << " �����˹������ݣ�" << sharedData << std::endl;

    // ����
    omp_unset_lock(&mpLock);
}

int test1()
{
    // ��ʼ��������
    omp_init_lock(&mpLock);

#pragma omp parallel num_threads(4)
    {
        updateSharedData();
    }

    // ���ٻ�����
    omp_destroy_lock(&mpLock);

    return 0;
}

static int enrol = []()->int
{
    //test0();
    cout << "test test_omp finish" << endl;
    return 0;
}();

