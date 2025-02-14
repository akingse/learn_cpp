#include "pch.h"

//基础用法
template<typename T>
class Func1
{
public:
	template<typename ...T> static auto fun(T... args);
	template<> static auto fun()
	{
		return 1.0;
	}
	template<typename Head, typename ...Args>static auto fun(Head head, Args... args)
	{
		return T::fun(head, fun(args...));
	}
};
class Sum1
{
public:
	template<typename A, typename B>
	static auto fun(const A& a, const B& b)
	{
		return a + b;
	}
};
class Multiply1
{
public:
	template<typename A, typename B>
	static auto fun(const A& a, const B& b)
	{
		return a * b;
	}
};
int main0()
{
	std::cout << Func1<Sum1>::fun(1.0, 2.0, 3, 4ull) << std::endl;
	std::cout << Func1<Multiply1>::fun(1.0, 2.0, 3, 4ull) << std::endl;
	return 0;
}

template<typename T>
class Func
{
public:
	template<typename ...T> static auto fun(T... args);
	template<> static auto fun()
	{
		return 1.0;
	}
	template<typename Head, typename ...Args>static auto fun(Head head, Args... args)
	{
		return T::fun(head, fun(args...));
	}
};
class Sum
{
public:
	template<typename A, typename B>
	static auto fun(const A& a, const B& b)
	{
		return a + b;
	}
};
class Multiply
{
public:
	template<typename A, typename B>
	static auto fun(const A& a, const B& b)
	{
		return a * b;
	}
};
int main_1()
{
	std::cout << Func<Sum>::fun(1.0, 2.0, 3, 4ull) << std::endl;
	std::cout << Func<Multiply>::fun(1.0, 2.0, 3, 4ull) << std::endl;
	return 0;
}



/*
#用途
1. 类型以后确定
2. 降低重复代码
3. 为类/接口提供代码段或变量
4. 不定参数数量的，可变长模板 ...
模板实现条件编译
#使用技巧
特化，偏特化
类型推导
autoex
*/

//---------------------------------------------------------------------------
using namespace std;
/// <summary>
/// www.cplusplus.com
/// </summary>
/// <returns></returns>


class A2 {};
class B : public A2 {};

//重载的坑
void fun(bool b)
{
	cout << "this is fun bool" << endl;
}
void fun(string c)
{
	cout << "this is fun char" << endl;
}

//class C;
//void fun(C &c){} //弱依赖


template<typename T, typename D>
class C
{
public:
	C()
	{
		C::print1();
	}
};


template <typename T>
class D
{
public:
	static void print1()
	{
		cout << "print1" << endl;

	}
		

};



template <typename ...T> //可变长模板
double sum(T... args)
{}//偏特化

template<> 
double sum()
{
	return 0;
}

template <typename Head, typename ...Args> 
double sum(Head he, Args... arg)
{
	return he + sum(arg...); //递归模板
}
//入参打包
//templeta 编译时
//python grnc 运行时


template <typename A, typename B>
void divide(const A& a, const B& b)
{
	//if (true)
}

int main_temp()
{
	auto s = sum(1, 2, 3.0);
	cout << sum(1, 2, 3.0) << endl;
	//推断基础关系
	//template <typename T1, typename T2>
	//int a = 0;
	//cout << typeid(a).name() << endl;
	//std::vector<int,A> vec; //提供new内存空间的方法
	auto c = 1ULL + 1UL;
	cout << c << endl;
	//内存分配，allocate
	bool b = true;
	//char* ch = "cha";
	cout << "bool="<<b << endl;
	fun(true);
	fun(string("cha"));

	//D<int> d;
	//C< int,D<int>> g; //编译慢，展开大


	//虚函数表（虚表）
	//static_assert(std::is_base_of(B,A),"raise error");

	//具名要求
	/* python list
	* list [1,2,3]
	
	
	*/

	/*
	* 自动类型推导
	* 降低重复代码
	* vector<stl>内存分配方法
	* 为类提供代码段
	* lambda表达式
	* 可变长模板
	* 条件编译
	
	*/


#ifdef NAME

#endif


#if _DEBUG

#else

#endif
	return 0;
}


template <typename S>
void collide(std::vector<S>* node, string* front_list)
{

}

static void _test1()
{
	std::vector<int>* node = new std::vector<int>(1);
	//collide(node);
	//破案了，fcl里面是模板函数声明和实现分离了
}

static int enrol = []()->int
	{
		_test1();
		cout << "test_triangle_intersect finished.\n" << endl;
		return 0;
	}();

