int main1()
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

#include<stdio.h>
int num = 5;
void func()
{
	printf("fun in a.c");
}