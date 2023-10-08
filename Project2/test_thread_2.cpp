#include "pch.h"
using namespace std;
//https://zhuanlan.zhihu.com/p/613630658

//多线程5文件
//thread
//mutex
//atomic
//condition variable
//future


int g_i = 0;
std::mutex g_i_mutex;  // protects g_i，用来保护g_i

void safe_increment()
{
	const std::lock_guard<std::mutex> lock(g_i_mutex);
	++g_i;
	std::cout << std::this_thread::get_id() << ": " << g_i << '\n';// g_i_mutex自动解锁}int main(){
	std::cout << "main id: " << std::this_thread::get_id() << std::endl;
	std::cout << "main: " << g_i << '\n';

	std::thread t1(safe_increment);
	std::thread t2(safe_increment);

	t1.join();
	t2.join();

	std::cout << "main: " << g_i << '\n';
}


struct Box
{
	explicit Box(int num) : num_things{ num } {}
	int num_things;
	std::mutex m;
};
void transfer(Box& from, Box& to, int num)
{
	// defer_lock表示暂时unlock，默认自动加锁
	std::unique_lock<std::mutex> lock1(from.m, std::defer_lock);
	std::unique_lock<std::mutex> lock2(to.m, std::defer_lock);//两个同时加锁
	std::lock(lock1, lock2);//或者使用lock1.lock()

	from.num_things -= num;
	to.num_things += num;//作用域结束自动解锁,也可以使用lock1.unlock()手动解锁
}
}

static int main1()
{
	Box acc1(100);
	Box acc2(50);

	//std::ref 用于包装按引用传递的值。
	//std::cref 用于包装按const引用传递的值
	std::thread t1(transfer, std::ref(acc1), std::ref(acc2), 10);
	std::thread t2(transfer, std::ref(acc2), std::ref(acc1), 5);

	t1.join();
	t2.join();
	std::cout << "acc1 num_things: " << acc1.num_things << std::endl;
	std::cout << "acc2 num_things: " << acc2.num_things << std::endl;
}

    // 创建线程对象
    std::vector<std::thread> m_threads;

static std::mutex mtx;
std::condition_variable cv;
int cargo = 0;
bool shipment_available()
{
	return cargo != 0;
}
void consume(int n)
{
	for (int i = 0; i < n; ++i)
	{
		std::unique_lock<std::mutex> lck(mtx);//自动上锁
		//第二个参数为false才阻塞（wait），阻塞完即unlock，给其它线程资源
		cv.wait(lck, shipment_available);// consume:
		std::cout << cargo << '\n';
		cargo = 0;
	}
}
static int main2()
{
	std::thread consumer_thread(consume, 10);
	for (int i = 0; i < 10; ++i)
	{
		//每次cargo每次为0才运行。
		while (shipment_available())
			std::this_thread::yield();
		std::unique_lock<std::mutex> lck(mtx);
		cargo = i + 1;
		cv.notify_one();
	}

	consumer_thread.join();
	return 0;
}

//std::condition_variable cv;
int value;
void read_value()
{
	std::cin >> value;
	cv.notify_one();
}
static int main3()
{
	std::cout << "Please, enter an integer (I'll be printing dots): \n";
	std::thread th(read_value);

	std::mutex mtx;
	std::unique_lock<std::mutex> lck(mtx);
	while (cv.wait_for(lck, std::chrono::seconds(1)) == std::cv_status::timeout)
	{
		std::cout << '.' << std::endl;
	}
	std::cout << "You entered: " << value << '\n';

	th.join();
	return 0;
}


//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

const int MAX_THREADS = 1000; //最大线程数目
template <typename T>
class threadPool
{
public:
	threadPool(int number = 1);//默认开一个线程
	~threadPool();
	std::queue<T> tasks_queue; //任务队列
	bool append(T* request);
	void worker(void* arg);
	//往请求队列＜task_queue＞中添加任务<T >
private:
	//工作线程需要运行的函数,不断的从任务队列中取出并执行
	//static void* worker(void arg);
	void run();
private:
	std::vector<std::thread> work_threads; //工作线程

	std::mutex queue_mutex;
	std::condition_variable condition;  //必须与unique_lock配合使用
	bool stop;
};//end class//构造函数，创建线程

template <typename T>
threadPool<T>::threadPool(int number) : stop(false)
{
	if (number <= 0 || number > MAX_THREADS)
		throw std::exception();
	for (int i = 0; i < number; i++)
	{
		std::cout << "created Thread num is : " << i << std::endl;
		work_threads.emplace_back(worker, this);
		//添加线程
		//直接在容器尾部创建这个元素，省去了拷贝或移动元素的过程。
	}
}
template <typename T>
inline threadPool<T>::~threadPool()
{
	std::unique_lock<std::mutex> lock(queue_mutex);
	stop = true;

	condition.notify_all();
	for (auto& ww : work_threads)
		ww.join();//可以在析构函数中join
}
//添加任务
template <typename T>
bool threadPool<T>::append(T* request)
{
	//操作工作队列时一定要加锁，因为他被所有线程共享
	queue_mutex.lock();//同一个类的锁
	tasks_queue.push(request);
	queue_mutex.unlock();
	condition.notify_one();  //线程池添加进去了任务，自然要通知等待的线程
	return true;
}//单个线程
template <typename T>
void threadPool<T>::worker(void* arg)
{
	threadPool pool = (threadPool*)arg;
	pool->run();//线程运行
	return pool;
}
template <typename T>
void threadPool<T>::run()
{
	while (!stop)
	{
		std::unique_lock<std::mutex> lk(this->queue_mutex);
		/*　unique_lock() 出作用域会自动解锁　*/
		this->condition.wait(lk, [this]
			{
				return !this->tasks_queue.empty();
			});//如果任务为空，则wait，就停下来等待唤醒//需要有任务，才启动该线程，不然就休眠
		if (this->tasks_queue.empty())//任务为空，双重保障
		{
			assert(0 && "断了");//实际上不会运行到这一步，因为任务为空，wait就休眠了。
			continue;
		}
		else 
		{
			T* request = tasks_queue.front();
			tasks_queue.pop();
			if (request)//来任务了，开始执行
				request->process();
		}
	}
}

static int enrol = []()->int
{
	//main1();
	//main2();
	//main3();
	//main4();
	//main5();
	//main6();
	//main7();
	//main8();
	//main9();
	//main10();
	return 0;
}();

