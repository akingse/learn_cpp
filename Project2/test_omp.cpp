#include "pch.h"
//https://zhuanlan.zhihu.com/p/397670985

#include<omp.h>
#include<iostream>
using namespace std;
static int main1()
{
#pragma omp parallel
	{
		cout << "Hello, world!" << endl;
	}
}