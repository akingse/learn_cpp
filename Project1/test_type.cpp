#include "pch.h"
using namespace std;

//std::ref可以在模板传参的时候传入引用，否则无法传递真正的引用
//std::cref() 用于const引用的版本。
template<class T>
void temp_function(T arg)
{
	std::cout << "com=" << &arg << "\n";
	arg++;
}
void f(int& i) {
	for (int j = 0; j < i; j++) {
		cout << i << endl;
	}
}


int main_type()
{

	//左值右值
	int num = 1;
	int& numR1 = num; //非常量的引用必须为左值
	const int& numR = 1; //常量的引用可以为右值
	int&& ar = 1; //右值引用

	int&& i = 123;
	int&& j = std::move(i);
	//int&& k = i;//编译不过，这里i是一个左值，右值引用只能引用右值



	int* p = new int;
	*p = 1;
	cout << *p << endl;
	cout << sizeof(*p) << endl; //4
	cout << sizeof(p) << endl; //8

	p = nullptr;
	cout << sizeof(p) << endl;
	cout << sizeof(*p) << endl;
	/*int a = 0;
	const int& b = 1;*/

	int ii = 5;
	//thread t([&]() {f(ii); });//引用未使用ref(i);
	// thread t(f, ref(ii));
	//t.join();

	//type_info decorated undecorated-name 装饰名
	const std::type_info& a = typeid(int); //初始化
	//typeid()，返回类型信息const std::type_info&。这种类型是不能赋值的
	//为了解决这个问题，C++引入了std::type_index类。这个类可以理解为封装了一个指向typeinfo的指针。
	std::type_index b = typeid(int);
	cout << *a.name() << endl;
	cout << a.hash_code() << endl;
	cout << b.name() << endl;
	cout << b.hash_code() << endl;	//返回这个类型的哈希值（具有唯一性）
	bool bl=a.before(a);

	auto name = typeid(int).name(); //string operator=自动转型
	auto name2= typeid(name).name(); //char const * __ptr64
	cout << typeid(int).name() << endl;
	auto name3 = ref(typeid(int));



	cout << "origin=" << i << endl;
	auto m1 = ref(i);
	cout << typeid(m1).name() << endl;
	++m1;
	cout << i << endl;



	int count = 10;
	int& rcount = count;
	temp_function(count);
	std::cout << count << std::endl;//10
	temp_function(rcount);
	std::cout << "main=" << &rcount << endl;
	std::cout << count << std::endl;//10
	//使用std::ref可以在模板传参的时候传入引用，thread的方法传递引用的时候，否则无法传递。
	//ref()方法的返回值是reference_wrapper类型, 这个类的源码大概的意思就是维持一个指针，并且重载操作符；
	temp_function(std::ref(rcount));//传引用
	std::cout << count << std::endl;//11

	std::cin.get();//等待输入



	cout << "----------------------------------" << endl;
	
	int n1 = 0, n2 = 0;
	++n1;
	cout << n1 << endl; //1
	n2++;
	cout << n2 << endl; //1

	n1 = 0, n2 = 0;
	cout << ++n1 << endl; //1 (i=i+1)
	cout << n2++ << endl; //0

	for (int i = 0; i < 1; ++i)
	{
		cout << i << endl;
	}
	for (int i = 0; i < 1; i++)
	{
		cout << i << endl;
	}


	//---------------------------------------------
	//隐式类型转换
	int m = 97;
	char c = (char)m;//显式把m转为char类型
	char c1 = char(m);
	char c2 = m; //隐式转换
	cout << c << endl;
	cout << c1 << endl;
	cout << c2 << endl;


	return 0;
}

