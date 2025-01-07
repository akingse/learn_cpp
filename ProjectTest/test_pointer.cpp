/*
因为指针的本身的值就是内存地址，它的占用字节数也就是该程序能够访问内存地址的空间大小，比如32位编译模式下，最大寻址为32位，2^32 B=4 GB，指针的值就是在 0x00000000 - 0xFFFFFFFF 范围内的值。因此指针本身占用的内存数和它指向的数据类型没有任何关系。
同理，64位编译模式下，理想的寻址位64位，也就是 2^64 B，这是个很大的值，而物理内存达不到这么大，CPU要实现64位的寻址能力只会增加系统复杂度和地址转换成本，因此Windows和Linux都做了限制，仅仅使用虚拟地址的48位，2^48 B=256TB。但是指针的占用内存字节数还是8 （只是Windows和Linux下，低48位有效而已）.
*/

#include"pch.h"
using namespace std;

int main_ptwo()
{

	//const 修饰指针 
//const int* ptr;
	int a = 1;
	int b = 2;
	int* p0 = &a;
	*p0 = 1;
	*p0 = 2;
	const int* p1 = &a;
	//*p1 = 2; //不能显式修改值
	a = 2;
	p1 = &b;
	int* const p2 = &a;
	//p2 = &b;//指向不能变
	*p2 = 3;
	a = 3;
	//p2 = &b;
	return 0;
}

//#include<stdio.h>
//int num = 5;
//void func()
//{
//	printf("fun in a.c");
//}

//指针传值
static void test_pointer_(byte* const data)
{
	//data += 4;
	*data = 'c';
	cout << *data << endl;
}

static void test_pointer(byte* data)
{
	data += 4;
	*data = 'c';
	cout << *data << endl;
}

//函数接收到的是指针 data 的一个副本。在函数内部，对 data 的偏移操作只会修改函数内部的副本，不会影响原始指针的值。
static void test_pointer_c(const byte* data)
{
	data += 4;
	//*data = 'c';
	cout << *data << endl;
}

//函数接收到的是指针 data 的引用。在函数内部，对 data 的偏移操作会直接修改原始指针的值，因为函数内部操作的是原始指针的引用。
static void test_pointer_r(byte*& data)
{
	data += 4;
	cout << *data << endl;
}

static void test_pointer_pp(byte** data)
{
	*data += 4;
	cout << *data << endl;
}

//测试指针改值
static void test1()
{
	string str = "12345";
	byte* data = new byte;
	memcpy(data, str.data(), str.size());

	//test_point_c(data);
	//cout << *data << endl;
	//test_point(data);
	//cout << *data << endl;
	//test_point_r(data);
	test_pointer_pp(&data);
	cout << *data << endl;
	return;
}

static void test2()
{
	Vec3* ptr = nullptr;
	Vec3** pp = &ptr; //null指针也可以取地址
    //*pp = &Vec3(1, 1);
	return;
}

//局部变量重名
static void test3()
{
	int a = 1;
	int c = 0;
	if (true)
	{
		int a = 2;//局部不改变外部（同名变量）
		c = a;
	}
	cout << a << endl;
}

static int enrol = []()->int
	{
		test1();
		test2();
		test3();
		cout << "test_pointer finished.\n" << endl;
		return 0;
	}();
