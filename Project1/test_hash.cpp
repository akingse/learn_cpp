#include "pch.h"
using namespace std;
using namespace std::literals;

#include <iostream>
#include <string>
#include <string_view>
#include <functional>
#include <memory_resource>
//#include <hash_map> is deprecated and will be REMOVED.
using namespace stdext;


using std::cout;
using std::endl;

/*
C++中常用的二进制运算符有六个，以下分别介绍：

1.  &  :  与操作.作用于两个二进制数,当然也可以对整型数据进行操作(当两边为整型数据会自动转化为二进制数).只有当对应位全为1时结果为1:

　　　　01011001 & 00101001

　　　　结果为:00001001

2. | :或操作.和1的与操作类似.用来合并值.只有当两个对应位都为0,结果位为0:

　　　　01011001 | 00101001

　　　　结果为:01111001

3. ^ :异或操作.这个运算符当两个值在某一位上相同时结果位为0,不同结果为1.如一个是1一个是0,结果位是1;两个都为1或者0结果位是0:

　　　　01011001^00101001

　　　　结果为:01110000

4.~ :求补操作.这个运算符只对一个二进制数据进行操作,对该数每一位取反,即1变为0;0变为1.例如:

　　　　~01011001

　　　　结果为:10100110

5-6.移位操作符.这两个操作符用来对一个值中的位左移或右移某个特定数字的位数.">>"右移操作."<<"左移操作.例如:

　　　　01011001>>2　　01011001<<2

　　　　结果为:0010110　　01100100
*/

class MyClass
{
public:
	MyClass() :
		str("hello"),
		data(0)
	{
	}

	bool operator==(const MyClass& rhs) const { return (data == rhs.data) && (str == rhs.str); }
	//因为unordered_set或者unordered_map,	//中需要对元素是否相同进行判断
	/*
	优点：因为内部实现了哈希表，因此其查找速度非常的快
缺点：哈希表的建立比较耗费时间
应用场景：对于查找问题，unordered_map会更加高效一些，因此遇到查找问题，常会考虑一下用unordered_map
	*/

public: //
	int data;
	std::string str;
};

//注意这里是将自己写的偏特化也同样加入到std中，因为他的模板是在std里面的，
//具体形式可以自己简单查看一下源码中的实现形式
//然后照着写一个自己的版本就行了。
namespace std
{
	template<>
	struct hash<MyClass> //: public __hash_base<size_t, MyClass>   //标准库中有这个继承，查看一下其实只是继承两个typedef而已，
	{
		size_t operator()(const MyClass& rhs) const noexcept    //这个const noexpect一定要写上去
		{
			return (std::hash<int>()(rhs.data)) ^ (std::hash<std::string>()(rhs.str) << 1); //当然，可以使用其他的方式来组合这个哈希值,
																							//这里是cppreference里面的例子，产生的数够乱就行。
		}
	};
}

void display(unordered_map<string, double> myrecipe, string str)
{
	cout << str << endl;
	for (auto& x : myrecipe)
		cout << x.first << ": " << x.second << endl;
	cout << endl;
}

int main_hash()
{

	unordered_map<Vec3, int> vmap;
	Vec3* vec = new Vec3(1,2);
	//vmap.insert({ move(*vec),1 });
	vmap.insert({ *vec,1 });
	delete vec;
	
	size_t a = 0b1010;
	size_t b = 0b1100;
	size_t c1 = (a ^ b) << 1;

	MyClass c;
	std::hash<MyClass> myHash;  //创建一个函数对象
	std::cout << myHash(c) << std::endl;

	//注意这第三个参数是typename _Hash = hash < _Value >， 是可写可不写的，因为他是有默认形式的，写出来就是这样
	std::unordered_map<MyClass, char, std::hash<MyClass>> m;	//这第三个参数

	std::unordered_set<MyClass> s;	//和上面的是一个意思，第二个参数是typename _Hash = hash < _Value >，可写可不写， 这里我是没写的。
	s.insert(c);
	s.insert(c);
	//std::unordered_map< Vec3Wrap, size_t> vwmap;
	//Vec3* v1 = new Vec3(1, 1);
	//Vec3* v2 = new Vec3(2, 2);
	//Vec3* v3 = new Vec3(3, 3);
	//Vec3Wrap vp1(v1);
	//Vec3Wrap vp2(v2);
	//vwmap.insert({ vp1,1 });

	//std::vector<Vec3Wrap> vecList;
	//vecList.push_back(vp1);
	//delete v1;
	//v1 = nullptr;
	//vecList.push_back(move(vp2));

	std::unordered_map<Vec3::Vec3Wrap, size_t> vwmap;

	std::unordered_map < MyClass, int> umap;
	MyClass* mc = new MyClass();
	mc->data = 1;
	mc->str = "first";
	umap.insert({ *mc,1 });
	delete mc;
	//std::cin.get();


	//---------------------------------------------------------
//	//cout << "hello hash" << endl;
//	auto sv = "Stand back! I've got jimmies!"sv;
//	int aa = 1;
//	std::string str(sv);
//	std::pmr::string pmrs(sv); // 使用默认分配器
//	std::cout << std::hash<std::string_view>{}(sv) << '\n';
//	std::cout << std::hash<std::string>{}(str) << '\n';
//	std::cout << std::hash<std::pmr::string>{}(pmrs) << '\n';
//
//
//	size_t hn = std::hash<std::string_view>{}("hello");
//	size_t hn1 = std::hash<std::string_view>{}("hash");
//	//size_t hn2 = std::hash<Vec1>{}(Vec1());
//	//std::unordered_map<Vec1, int> umap;
////	size_t hn2 = std::hash<Vec3>{}(Vec3());

	size_t hn3 = 0;

	//test MD5
	string message = "hello";
	cout << "md5(\"" << message << "\") = "
		<< MD5(message).toStr();
	//------------------------------------------------------

	unordered_map < string, double>
		myrecipe,
		mypantry = { {"milk",2.0},{"flour",1.5} };

	/****************插入*****************/
	pair<string, double> myshopping("baking powder", 0.3);
	myrecipe.insert(myshopping);                        // 复制插入
	myrecipe.insert(make_pair<string, double>("eggs", 6.0)); // 移动插入
	myrecipe.insert(mypantry.begin(), mypantry.end());  // 范围插入
	myrecipe.insert({ {"sugar",0.8},{"salt",0.1} });    // 初始化数组插入(可以用二维一次插入多个元素，也可以用一维插入一个元素)
	myrecipe["coffee"] = 10.0;  //数组形式插入

	display(myrecipe, "myrecipe contains:");

	/****************查找*****************/
	unordered_map<string, double>::const_iterator got = myrecipe.find("coffee");

	if (got == myrecipe.end())
		cout << "not found";
	else
		cout << "found " << got->first << " is " << got->second << "\n\n";
	/****************修改*****************/
	myrecipe.at("coffee") = 9.0;
	myrecipe["milk"] = 3.0;
	display(myrecipe, "After modify myrecipe contains:");


	/****************擦除*****************/
	myrecipe.erase(myrecipe.begin());  //通过位置
	myrecipe.erase("milk");    //通过key
	display(myrecipe, "After erase myrecipe contains:");

	/****************交换*****************/
	myrecipe.swap(mypantry);
	display(myrecipe, "After swap with mypantry, myrecipe contains:");

	/****************清空*****************/
	myrecipe.clear();
	display(myrecipe, "After clear, myrecipe contains:");
	return 0;
}


