//多线程编程
#include "pch.h"

using namespace std;

int g_Value = 0;
class CLambda
{
protected:
	int m_Value;
public:
	void Test1(int InValue)
	{
		int Value = 0;
		auto a1 = [](int x) {/*仅能访问全局外部变量*/};
		auto a2 = [Value](int x) {/*值传递局部变量Value*/};
		auto a3 = [this](int x) {/*值传递this指针*/};
		auto a4 = [&Value](int x) {/*引用传递局部变量Value*/};
		auto a5 = [=](int x) {/*值传递所有可访问的外部变量*/};
		auto a6 = [&](int x) {/*引用传递所有可访问的外部变量*/};
		auto a7 = [=, &Value](int x) {/*引用传递局部变量Value，值传递所有其他可访问的外部变量*/};
		auto a8 = [&, Value](int x) {/*值传递局部变量Value，引用传递所有其他可访问的外部变量*/};
	}
};
/*
[]：默认不捕获任何变量；
[=]：默认以值捕获所有变量；
[&]：默认以引用捕获所有变量；
[x]：仅以值捕获x，其它变量不捕获；
[&x]：仅以引用捕获x，其它变量不捕获；
[=, &x]：默认以值捕获所有变量，但是x是例外，通过引用捕获；
[&, x]：默认以引用捕获所有变量，但是x是例外，通过值捕获；
[this]：通过引用捕获当前对象（其实是复制指针）；
[*this]：通过传值方式捕获当前对象；
*/

void myfunction(int i) {  // function:
	std::cout << ' ' << i;
}

struct myclass {           // function object type:  one paramter
	void operator() (int i) 
	{ std::cout << ' ' << i; }
} myobject;

struct myclass_twoParam {           // function object type:  two or more paramter
	void operator() (int i, int ii) 
	{ std::cout << ' ' << i << " " << ii; }
} myobject_TwoParam;



int main_lamd() 
{
	std::vector<int> myvector = {10,20,30};

	std::cout << "myvector contains:";
	for_each(myvector.begin(), myvector.end(), myfunction);  //给 for_each 传递一个函数
	std::cout << '\n';

	// or:
	std::cout << "myvector contains:";
	for_each(myvector.begin(), myvector.end(), myobject);   //给 for_each 传递一个函数对象33   std::cout << '\n'; 34 35   // or: 36   std::cout << "myvector contains:";//给 for_each 传递一个lamdba37   for_each (myvector.begin(), myvector.end(), [&](int i){myobject_TwoParam(i, 100);}); //当可调用对象或者函数的参数数量超过两个时， 可以给 for_each传递一个lamdba临时可调用对象，用这个临时可调用对象将原可调用对象包裹一次。 38   std::cout << '\n'; 3940   return 0; 
	std::cout << '\n';
	// or: 
	std::cout << "myvector contains:";//给 for_each 传递一个lamdba
	for_each(myvector.begin(), myvector.end(), [&](int i) {myobject_TwoParam(i, 100); }); //当可调用对象或者函数的参数数量超过两个时， 可以给 for_each传递一个lamdba临时可调用对象，用这个临时可调用对象将原可调用对象包裹一次。 
	std::cout << '\n';
	
	/*
	std::for_each
	第一个参数表示一段区间的开始，可以是迭代器，也可以是数组的某个位置；
	第二个参数表示一段区间的结束，可以是迭代器，也可以是数组的某个位置；
	第三个参数表示一个可调用对象，可以是函数，可以是函数对象，也可以是lamdba；
	std::for_each 返回第三个参数，或者第三个参数的右值引用。
	*/

	std::vector<int> some_list = { 1,2,3 };
	int total = 0;
	int total_2 = 0;
	/*for (int i = 0; i < 5; ++i)
		some_list.push_back(i);*/
	/*std::for_each(begin(some_list), end(some_list), [&total](int x)
		{
			total += x;
		});*/
	std::for_each(some_list.begin(), some_list.end(), [&total_2](int x)
		{
			total_2 += x;
		});
	//[capture] (parameters) specifiers->return_type{ body }
	//capture 捕获的外部变量列表，通过逗号分隔，可进行传值捕获或者引用捕获，lambda表达式与这些捕获的外部变量会构成一个闭包（Closure），外部变量为闭包的成员变量
	cout << total << endl;
	cout << total_2 << endl;
	int a = 2;
	auto f = [&total](int a) ->int {total += 1; cout << "lambda=" << total << endl; return total; };
	f(1);
	cout << typeid(f).name() << endl;
	//if (a <= 1) { a != 2; };
	long long num = 2;
	cout << num << endl;
	cout << int(num) << endl;
	
	


	return 0;
}