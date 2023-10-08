#include "pch.h"
using namespace std;


// ����Ҫִ�е�forѭ������
void performTask(int start, int end, const std::vector<int>& data)
{
    for (int i = start; i < end; ++i)
    {
        // ������ִ����ļ�������
        // ���磺data[i] = data[i] * 2;
        std::cout << "Thread ID: " << std::this_thread::get_id() << ", Index: " << i << std::endl;
    }
}

static int main1()
{
    // ������һ�������������ݵ�����
    std::vector<int> data(10000);

    // �����߳�����
    int numThreads = 8;

    // ����ÿ���߳�Ҫ��������ݷ�Χ
    int chunkSize = data.size() / numThreads;
    int remaining = data.size() % numThreads;

    // �����̶߳���
    std::vector<std::thread> m_threads;

    // �����̲߳�ִ������
    int start = 0;
    int end = 0;
    for (int i = 0; i < numThreads; ++i)
    {
        start = end;
        end = start + chunkSize;

        // ���ǵ������������һ���̵߳����ݷ�Χ��չ
        if (i == numThreads - 1)
        {
            end += remaining;
        }

        // �����̲߳�ִ������
        m_threads.push_back(std::thread(performTask, start, end, std::ref(data)));
    }

    // �ȴ������߳����
    for (auto& thread : m_threads)
    {
        thread.join();
    }

    // ��������Լ�����������ִ����������

    return 0;
}




static int enrol = []()->int
{
    main1();
	return 0;
}();

