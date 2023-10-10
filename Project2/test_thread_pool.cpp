#include "pch.h"
using namespace std;


//#include <iostream>
//#include <thread>
//#include <functional>
//#include <vector>
//#include <queue>
//#include <condition_variable>

class ThreadPool {
public:
	ThreadPool(size_t numThreads) : stop(false) 
	{
		for (size_t i = 0; i < numThreads; ++i) 
		{
			m_threads.emplace_back([this] 
				{
					while (true) 
					{
						std::function<void()> task;
						{
							std::unique_lock<std::mutex> lock(m_queueMutex);
							m_condition.wait(lock, [this] { return stop || !m_taskQueue.empty(); });

							if (stop && m_taskQueue.empty()) 
							{
								return;
							}
							task = std::move(m_taskQueue.front());
							m_taskQueue.pop();
						}
						task();
					}
				});
		}
	}

	~ThreadPool() 
	{
		{
			std::unique_lock<std::mutex> lock(m_queueMutex);
			stop = true;
		}
		m_condition.notify_all();
		for (std::thread& thread : m_threads) 
		{
			thread.join();
		}
	}

	template<typename F, typename... Args>
	void enqueue(F&& function, Args&&... args) 
	{
		std::function<void()> task = std::bind(std::forward<F>(function), std::forward<Args>(args)...);
		{
			std::unique_lock<std::mutex> lock(m_queueMutex);
			m_taskQueue.emplace(std::move(task));
		}
		m_condition.notify_one();
	}

private:
	std::vector<std::thread> m_threads;
	std::queue<std::function<void()>> m_taskQueue;
	std::mutex m_queueMutex;
	std::condition_variable m_condition;
	bool stop;
};

// 示例任务函数
void printNumber(int number) 
{
	std::cout << "Number: " << number << std::endl;
}

static int main1() {
	// 创建线程池，指定线程数量
	ThreadPool pool(4);

	// 添加任务到线程池
	for (int i = 0; i < 10; ++i) {
		pool.enqueue(printNumber, i);
	}

	// 等待所有任务完成
	std::this_thread::sleep_for(std::chrono::seconds(1));

	return 0;
}

//---------------------------------------------------------------------------------------------------
// 
//---------------------------------------------------------------------------------------------------
// 
//Boost 线程池位于组件 asio 中，是一种固定大小的线程池。
#include <boost/asio.hpp>
/* 互斥锁 */
std::mutex mutex_iostream;

void my_task(void)
{
	std::lock_guard<std::mutex> lg(mutex_iostream);
	std::cout.flush();
	std::cout << "This is my task." << std::endl;
	std::cout.flush();
}

static int main2(int argc, const char** argv)
{
	/* 定义一个4线程的线程池 */
	boost::asio::thread_pool tp(4);

	/* 将函数投放到线程池 */
	for (int i = 0; i < 5; ++i) {
		boost::asio::post(tp, my_task);
	}

	/* 将语句块投放到线程池 */
	for (int i = 0; i < 5; ++i) {
		boost::asio::post(
			tp,
			[]() {
				std::lock_guard<std::mutex> lg(mutex_iostream);
				std::cout.flush();
				std::cout << "This is lambda task." << std::endl;
				std::cout.flush();
			});
	}

	/* 退出所有线程 */
	tp.join();
	return 0;
}



class fixed_thread_pool {
public:
	explicit fixed_thread_pool(size_t thread_count)
		: data_(std::make_shared<data>()) {
		for (size_t i = 0; i < thread_count; ++i) {
			std::thread([data = data_] {
				std::unique_lock<std::mutex> lk(data->mtx_);
				for (;;) {
					if (!data->tasks_.empty()) {
						auto current = std::move(data->tasks_.front());
						data->tasks_.pop();
						lk.unlock();
						current();
						lk.lock();
					}
					else if (data->is_shutdown_) {
						break;
					}
					else {
						data->cond_.wait(lk);
					}
				}
				}).detach();
		}
	}

	fixed_thread_pool() = default;
	fixed_thread_pool(fixed_thread_pool&&) = default;

	~fixed_thread_pool() {
		if ((bool)data_) {
			{
				std::lock_guard<std::mutex> lk(data_->mtx_);
				data_->is_shutdown_ = true;
			}
			data_->cond_.notify_all();
		}
	}

	template <class F>
	void execute(F&& task) {
		{
			std::lock_guard<std::mutex> lk(data_->mtx_);
			data_->tasks_.emplace(std::forward<F>(task));
		}
		data_->cond_.notify_one();
	}

private:
	struct data {
		std::mutex mtx_;
		std::condition_variable cond_;
		bool is_shutdown_ = false;
		std::queue<std::function<void()>> tasks_;
	};
	std::shared_ptr<data> data_;
};

static int enrol = []()->int
{
	//main1();
	return 0;
}();

