#include "pch.h"

//�����÷�
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
#��;
1. �����Ժ�ȷ��
2. �����ظ�����
3. Ϊ��/�ӿ��ṩ����λ����
4. �������������ģ��ɱ䳤ģ�� ...
ģ��ʵ����������
#ʹ�ü���
�ػ���ƫ�ػ�
�����Ƶ�
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

//���صĿ�
void fun(bool b)
{
	cout << "this is fun bool" << endl;
}
void fun(string c)
{
	cout << "this is fun char" << endl;
}

//class C;
//void fun(C &c){} //������


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



template <typename ...T> //�ɱ䳤ģ��
double sum(T... args)
{}//ƫ�ػ�

template<> 
double sum()
{
	return 0;
}

template <typename Head, typename ...Args> 
double sum(Head he, Args... arg)
{
	return he + sum(arg...); //�ݹ�ģ��
}
//��δ��
//templeta ����ʱ
//python grnc ����ʱ


template <typename A, typename B>
void divide(const A& a, const B& b)
{
	//if (true)
}

int main_temp()
{
	auto s = sum(1, 2, 3.0);
	cout << sum(1, 2, 3.0) << endl;
	//�ƶϻ�����ϵ
	//template <typename T1, typename T2>
	//int a = 0;
	//cout << typeid(a).name() << endl;
	//std::vector<int,A> vec; //�ṩnew�ڴ�ռ�ķ���
	auto c = 1ULL + 1UL;
	cout << c << endl;
	//�ڴ���䣬allocate
	bool b = true;
	//char* ch = "cha";
	cout << "bool="<<b << endl;
	fun(true);
	fun(string("cha"));

	//D<int> d;
	//C< int,D<int>> g; //��������չ����


	//�麯�������
	//static_assert(std::is_base_of(B,A),"raise error");

	//����Ҫ��
	/* python list
	* list [1,2,3]
	
	
	*/

	/*
	* �Զ������Ƶ�
	* �����ظ�����
	* vector<stl>�ڴ���䷽��
	* Ϊ���ṩ�����
	* lambda���ʽ
	* �ɱ䳤ģ��
	* ��������
	
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
	//�ư��ˣ�fcl������ģ�庯��������ʵ�ַ�����
}

static int enrol = []()->int
	{
		_test1();
		cout << "test_triangle_intersect finished.\n" << endl;
		return 0;
	}();

