atomic[原子锁](https://zh.cppreference.com/w/cpp/atomic/atomic)

平凡类型
std::is_trivially_copyable<T>::value
std::is_copy_constructible<T>::value
std::is_move_constructible<T>::value
std::is_copy_assignable<T>::value
std::is_move_assignable<T>::value

[知乎](https://zhuanlan.zhihu.com/p/613630658)

```c
//before C++11
#include <pthread.h> //linux
#include <windows.h> //windows

#include <thread> //C++11
//它解决了跨平台的问题，提供了管理线程、保护共享数据、线程间同步操作、原子操作等类。



```

![img](https://pic2.zhimg.com/80/v2-76e5e48c9c1d60f9868452cfc9ce7d85_1440w.webp)

多进程：操作系统对进程提供了大量的保护机制，更容易写出相对安全的代码；进程间的通信，无论是使用信号、套接字，还是文件、管道等方式，其使用要么比较复杂，要么就是速度较慢或者两者兼而有之。运行多个线程的开销很大，操作系统要分配很多的资源来对这些进程进行管理。

多线程：线程是轻量级的进程，每个线程可以独立的运行不同的指令序列，但是线程不独立的拥有资源，依赖于创建它的进程而存在。同一进程内的多个线程能够很方便的进行数据共享以及通信，也就比进程更适用于并发操作。由于缺少操作系统提供的保护机制，在多线程共享数据及通信时，就需要程序员做更多的工作以保证对共享数据段的操作是以预想的操作顺序进行的，并且要极力的避免死锁(deadlock)。

#### 创建thread

```v
std::thread myThread ( thread_fun);
//函数形式为void thread_fun()
myThread.join();

std::thread myThread ( thread_fun(100));
myThread.join();
//函数形式为void thread_fun(int x)

std::thread (thread_fun,1).detach();
//直接创建线程，没有名字
//函数形式为void thread_fun(int x)
```

### this_thread

| 函数        | 使用                                                  | 说明                       |
| ----------- | ----------------------------------------------------- | -------------------------- |
| get_id      | std::this_thread::get_id()                            | 获取线程id                 |
| yield       | std::this_thread::yield()                             | 放弃线程执行，回到就绪状态 |
| sleep_for   | std::this_thread::sleep_for(std::chrono::seconds(1)); | 暂停1秒                    |
| sleep_until | 如下                                                  | 一分钟后执行吗，如下       |

```c
using std::chrono::system_clock;
std::time_t tt = system_clock::to_time_t(system_clock::now());
struct std::tm * ptm = std::localtime(&tt);
cout << "Waiting for the next minute to begin...\n";
++ptm->tm_min; //加一分钟
ptm->tm_sec = 0; //秒数设置为0//暂停执行，到下一整分执行
this_thread::sleep_until(system_clock::from_time_t(mktime(ptm)));
```

## mutex

mutex头文件主要声明了与互斥量(mutex)相关的类。mutex提供了4种互斥类型，如下表所示。

| 类型                       | 说明                |
| -------------------------- | ------------------- |
| std::mutex                 | 最基本的 Mutex 类。 |
| std::recursive_mutex       | 递归 Mutex 类。     |
| std::time_mutex            | 定时 Mutex 类。     |
| std::recursive_timed_mutex | 定时递归 Mutex 类。 |

std::mutex 是C++11 中最基本的互斥量，std::mutex 对象提供了独占所有权的特性——即不支持递归地对 std::mutex 对象上锁，而 std::recursive_lock 则可以递归地对互斥量对象上锁。

```c
int a = 0;
std::mutex mtx;

void thread_task1()
{
	for (int i = 0; i < 100000; ++i)
	{
		mtx.lock();
		++a;
		mtx.unlock();
	}
}
```



| std::mutex function |                                                              |
| ------------------- | ------------------------------------------------------------ |
| lock()              | 调用线程将锁住该互斥量。线程调用该函数会发生下面 3 种情况：<br/><br/>如果该互斥量当前没有被锁住，则调用线程将该互斥量锁住，直到调用 unlock之前，该线程一直拥有该锁。<br/>如果当前互斥量被其他线程锁住，则当前的调用线程被阻塞住。<br/>如果当前互斥量被当前调用线程锁住，则会产生死锁(deadlock)。<br/> |
| unlock()            | 解锁，释放对互斥量的所有权，如果没有锁的所有权尝试解锁会导致程序异常。 |
| try_lock()          | 尝试锁住互斥量，如果互斥量被其他线程占有，则当前线程也不会被阻塞。线程调用该函数也会出现下面 3 种情况：<br/><br/>如果当前互斥量没有被其他线程占有，则该线程锁住互斥量，直到该线程调用 unlock 释放互斥量。<br/>如果当前互斥量被其他线程锁住，则当前调用线程返回 false，而并不会被阻塞掉。<br/>如果当前互斥量被当前调用线程锁住，则会产生死锁(deadlock)。<br/> |
| try_lock_for()      |                                                              |
| try_lock_until()    |                                                              |
|                     |                                                              |
|                     |                                                              |



std::recursive_mutex 与 std::mutex 一样，也是一种可以被上锁的对象，但是和 std::mutex 不同的是，std::recursive_mutex 允许同一个线程对互斥量多次上锁（即递归上锁），来获得对互斥量对象的多层所有权

std::time_mutex 比 std::mutex 多了两个成员函数，try_lock_for()，try_lock_until()。

#### lock 类

`std::lock_guard`：与 mutex RAII 相关，方便线程对互斥量上锁。
`std::unique_lock`：与 mutex RAII 相关，方便线程对互斥量上锁，但提供了更好的上锁和解锁控制。





---

# Multi-threading

 多线程是为了同步完成多项任务，不是为了提高运行效率（最大运行效率由CPU决定），而是为了提高资源使用效率来提高系统的效率。线程是在同一时间需要完成多项任务的时候实现的；

 

### 线程-进程

对于不同进程来说，它们具有独立的数据空间，要进行数据的传递只能通过通信的方式进行，这种方式不仅费时，而且很不方便。线程则不然，由于同一进程下的线程之间共享数据空间，所以一个线程的数据可以直接为其他线程所用，这不仅快捷，而且方便

 并发：两个或者更多的任务（独立的活动）同时发生（进行）：一个程序同时执行多个独立的任务；

进程：一个可执行程序运行起来了，就叫创建了一个进程。进程就是运行起来的可执行程序。

线程：用来执行代码的。线程这个东西，可以理解为一条代码的执行通路

线程并不是越多越好，每个线程，都需要一个独立的堆栈空间（大约1M），线程之间的切换要保存很多中间状态，切换也会耗费本该属于程序运行的时间



### lambda表达式

C++11的std::thread，std::thread配合lambda表达式创建个线程运行，很方便；

匿名函数（Anonymous function），其构造了一个可以在其作用范围内捕获变量的函数对象。

实际为一个仿函数functor，编译器后会生成一个匿名类（这个类重载了()运算符）

[capture] (parameters) specifiers -> return_type { body }

[函数对象参数] (操作符重载函数参数) mutable 或 exception 声明 -> 返回值类型 {函数体}



### C++11并发与多线程

[老狂](https://www.bilibili.com/video/BV1Yb411L7ak)

《课程目录》

章节1前言

  课时1.1 前言

章节2正式开讲

  课时2.1 并发基本概念及实现，进程、线程基本概念

  课时2.2 线程启动、结束，创建线程多法、join，detach

  课时2.3 线程传参详解，detach()大坑，成员函数做线程函数

  课时2.4 创建多个线程、数据共享问题分析、案例代码

  课时2.5 互斥量概念、用法、死锁演示及解决详解

  课时2.6 unique_lock详解

  课时2.7 单例设计模式共享数据分析、解决，call_once

  课时2.8 condition_variable、wait、notify_one、notify_all

  课时2.9 async、future、packaged_task、promise

  课时2.10 future其他成员函数、shared_future、atomic

  课时2.11 std::atomic续谈、std::async深入谈

  课时2.12 windows临界区、其他各种mutex互斥量

  课时2.13 补充知识、线程池浅谈、数量谈、总结

章节3结束语

  课时3.1 课程总结与展望

[课程笔记](https://blog.csdn.net/qq_38231713/category_10001159.html)

---

### 多线程-线程池

优势：切换代价小，适合IO切花，充分利用CPU多核

posix thread

C++11 thread



