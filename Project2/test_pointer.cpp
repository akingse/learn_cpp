/*
��Ϊָ��ı����ֵ�����ڴ��ַ������ռ���ֽ���Ҳ���Ǹó����ܹ������ڴ��ַ�Ŀռ��С������32λ����ģʽ�£����ѰַΪ32λ��2^32 B=4 GB��ָ���ֵ������ 0x00000000 - 0xFFFFFFFF ��Χ�ڵ�ֵ�����ָ�뱾��ռ�õ��ڴ�������ָ�����������û���κι�ϵ��
ͬ��64λ����ģʽ�£������Ѱַλ64λ��Ҳ���� 2^64 B�����Ǹ��ܴ��ֵ���������ڴ�ﲻ����ô��CPUҪʵ��64λ��Ѱַ����ֻ������ϵͳ���ӶȺ͵�ַת���ɱ������Windows��Linux���������ƣ�����ʹ�������ַ��48λ��2^48 B=256TB������ָ���ռ���ڴ��ֽ�������8 ��ֻ��Windows��Linux�£���48λ��Ч���ѣ�.
*/

#include"pch.h"
using namespace std;

int main_ptwo()
{

	//const ����ָ�� 
//const int* ptr;
	int a = 1;
	int b = 2;
	int* p0 = &a;
	*p0 = 1;
	*p0 = 2;
	const int* p1 = &a;
	//*p1 = 2; //������ʽ�޸�ֵ
	a = 2;
	p1 = &b;
	int* const p2 = &a;
	//p2 = &b;//ָ���ܱ�
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

//ָ�봫ֵ
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

//�������յ�����ָ�� data ��һ���������ں����ڲ����� data ��ƫ�Ʋ���ֻ���޸ĺ����ڲ��ĸ���������Ӱ��ԭʼָ���ֵ��
static void test_pointer_c(const byte* data)
{
	data += 4;
	//*data = 'c';
	cout << *data << endl;
}

//�������յ�����ָ�� data �����á��ں����ڲ����� data ��ƫ�Ʋ�����ֱ���޸�ԭʼָ���ֵ����Ϊ�����ڲ���������ԭʼָ������á�
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

//����ָ���ֵ
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
	Vec3** pp = &ptr; //nullָ��Ҳ����ȡ��ַ
    //*pp = &Vec3(1, 1);
	return;
}

//�ֲ���������
static void test3()
{
	int a = 1;
	int c = 0;
	if (true)
	{
		int a = 2;//�ֲ����ı��ⲿ��ͬ��������
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
