#include "pch.h"
using namespace std;


// 定义要执行的for循环任务
void performTask(int start, int end, const std::vector<int>& data)
{
    for (int i = start; i < end; ++i)
    {
        // 在这里执行你的计算任务
        // 例如：data[i] = data[i] * 2;
        std::cout << "Thread ID: " << std::this_thread::get_id() << ", Index: " << i << std::endl;
    }
}

static int main1()
{
    // 假设有一个包含大量数据的向量
    std::vector<int> data(10000);

    // 定义线程数量
    int numThreads = 8;

    // 计算每个线程要处理的数据范围
    int chunkSize = data.size() / numThreads;
    int remaining = data.size() % numThreads;

    // 创建线程对象
    std::vector<std::thread> m_threads;

    // 启动线程并执行任务
    int start = 0;
    int end = 0;
    for (int i = 0; i < numThreads; ++i)
    {
        start = end;
        end = start + chunkSize;

        // 考虑到余数，将最后一个线程的数据范围扩展
        if (i == numThreads - 1)
        {
            end += remaining;
        }

        // 创建线程并执行任务
        m_threads.push_back(std::thread(performTask, start, end, std::ref(data)));
    }

    // 等待所有线程完成
    for (auto& thread : m_threads)
    {
        thread.join();
    }

    // 在这里可以继续处理结果或执行其他操作

    return 0;
}




static int enrol = []()->int
{
    main1();
	return 0;
}();

