#include "pch.h"
using namespace std;
//https://zhuanlan.zhihu.com/p/613630658

//���߳�5�ļ�
//thread
//mutex
//atomic
//condition variable
//future


int g_i = 0;
std::mutex g_i_mutex;  // protects g_i����������g_i

void safe_increment()
{
	const std::lock_guard<std::mutex> lock(g_i_mutex);
	++g_i;
	std::cout << std::this_thread::get_id() << ": " << g_i << '\n';// g_i_mutex�Զ�����}int main(){
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
	// defer_lock��ʾ��ʱunlock��Ĭ���Զ�����
	std::unique_lock<std::mutex> lock1(from.m, std::defer_lock);
	std::unique_lock<std::mutex> lock2(to.m, std::defer_lock);//����ͬʱ����
	std::lock(lock1, lock2);//����ʹ��lock1.lock()

	from.num_things -= num;
	to.num_things += num;//����������Զ�����,Ҳ����ʹ��lock1.unlock()�ֶ�����
}
}

static int main1()
{
	Box acc1(100);
	Box acc2(50);

	//std::ref ���ڰ�װ�����ô��ݵ�ֵ��
	//std::cref ���ڰ�װ��const���ô��ݵ�ֵ
	std::thread t1(transfer, std::ref(acc1), std::ref(acc2), 10);
	std::thread t2(transfer, std::ref(acc2), std::ref(acc1), 5);

	t1.join();
	t2.join();
	std::cout << "acc1 num_things: " << acc1.num_things << std::endl;
	std::cout << "acc2 num_things: " << acc2.num_things << std::endl;
}

    // �����̶߳���
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
		std::unique_lock<std::mutex> lck(mtx);//�Զ�����
		//�ڶ�������Ϊfalse��������wait���������꼴unlock���������߳���Դ
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
		//ÿ��cargoÿ��Ϊ0�����С�
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

const int MAX_THREADS = 1000; //����߳���Ŀ
template <typename T>
class threadPool
{
public:
	threadPool(int number = 1);//Ĭ�Ͽ�һ���߳�
	~threadPool();
	std::queue<T> tasks_queue; //�������
	bool append(T* request);
	void worker(void* arg);
	//��������У�task_queue�����������<T >
private:
	//�����߳���Ҫ���еĺ���,���ϵĴ����������ȡ����ִ��
	//static void* worker(void arg);
	void run();
private:
	std::vector<std::thread> work_threads; //�����߳�

	std::mutex queue_mutex;
	std::condition_variable condition;  //������unique_lock���ʹ��
	bool stop;
};//end class//���캯���������߳�

template <typename T>
threadPool<T>::threadPool(int number) : stop(false)
{
	if (number <= 0 || number > MAX_THREADS)
		throw std::exception();
	for (int i = 0; i < number; i++)
	{
		std::cout << "created Thread num is : " << i << std::endl;
		work_threads.emplace_back(worker, this);
		//����߳�
		//ֱ��������β���������Ԫ�أ�ʡȥ�˿������ƶ�Ԫ�صĹ��̡�
	}
}
template <typename T>
inline threadPool<T>::~threadPool()
{
	std::unique_lock<std::mutex> lock(queue_mutex);
	stop = true;

	condition.notify_all();
	for (auto& ww : work_threads)
		ww.join();//����������������join
}
//�������
template <typename T>
bool threadPool<T>::append(T* request)
{
	//������������ʱһ��Ҫ��������Ϊ���������̹߳���
	queue_mutex.lock();//ͬһ�������
	tasks_queue.push(request);
	queue_mutex.unlock();
	condition.notify_one();  //�̳߳���ӽ�ȥ��������ȻҪ֪ͨ�ȴ����߳�
	return true;
}//�����߳�
template <typename T>
void threadPool<T>::worker(void* arg)
{
	threadPool pool = (threadPool*)arg;
	pool->run();//�߳�����
	return pool;
}
template <typename T>
void threadPool<T>::run()
{
	while (!stop)
	{
		std::unique_lock<std::mutex> lk(this->queue_mutex);
		/*��unique_lock() ����������Զ�������*/
		this->condition.wait(lk, [this]
			{
				return !this->tasks_queue.empty();
			});//�������Ϊ�գ���wait����ͣ�����ȴ�����//��Ҫ�����񣬲��������̣߳���Ȼ������
		if (this->tasks_queue.empty())//����Ϊ�գ�˫�ر���
		{
			assert(0 && "����");//ʵ���ϲ������е���һ������Ϊ����Ϊ�գ�wait�������ˡ�
			continue;
		}
		else 
		{
			T* request = tasks_queue.front();
			tasks_queue.pop();
			if (request)//�������ˣ���ʼִ��
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

