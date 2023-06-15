#include "pch.h"
//#include <windows.h>
using namespace std;
// 数据结构和算法
// data structure and algorithm

/*
归并排序的原理 将待排序的数组分成前后两个部分，再递归的将前半部分数据和后半部分的数据各自归并排序，得到的两部分数据，
然后使用merge合并算法（算法见代码）将两部分算法合并到一起。
例如：如果N=1；那么只有一个数据要排序，N=2，只需要调用merge函数将前后合并，N=4，...........
也就是将一个很多数据的数组分成前后两部分，然后不断递归归并排序，再合并，最后返回有序的数组。
归并排序的时间复杂度  归并排序的最好、最坏和平均时间复杂度都是O(nlogn)，
而空间复杂度是O(n)，比较次数介于(nlogn)/2和(nlogn)-n+1，
赋值操作的次数是(2nlogn)。因此可以看出，归并排序算法比较占用内存，但却是效率高且稳定的排序算法。

*/
void merge(int array[], int tmp[], int leftPos, int rightPos, int rightEnd)
{
	// TODO Auto-generated method stub
	int leftEnd = rightPos - 1;
	int tmpPos = leftPos;
	int numElements = rightEnd - leftPos + 1;
	while (leftPos <= leftEnd && rightPos <= rightEnd) {
		if (array[leftPos] <= array[rightPos]) {
			tmp[tmpPos++] = array[leftPos++];
		}
		else {
			tmp[tmpPos++] = array[rightPos++];
		}
	}
	while (leftPos <= leftEnd) {
		tmp[tmpPos++] = array[leftPos++];
	}
	while (rightPos <= rightEnd) {
		tmp[tmpPos++] = array[rightPos++];
	}
	for (int i = 0; i < numElements; i++, rightEnd--) {
		array[rightEnd] = tmp[rightEnd];
	}
}

void mergeSort(int array[], int tmp[], int left, int right)
{
	if (left < right)
	{
		int center = (left + right) / 2;//取数组的中点
		mergeSort(array, tmp, left, center);//归并排序数组的前半部分
		mergeSort(array, tmp, center + 1, right);//归并排序数组的后半部分
		merge(array, tmp, left, center + 1, right);//将数组的前后半部分合并
	}
}
/*
 * 超简单的合并函数
 */

void mergeSort(int array[], int n)
{
	//int tmp[n];//声明一个用来合并的数组
	int* tmp = new int[n];
	//memcpy(tmp, array, n);
	mergeSort(array, tmp, 0, n - 1);//调用排序函数，传入数字的起点和终点
}


int sum(int A[], int n)
{
	return
		(n == 0) ?
		0 : sum(A, n - 1) + A[n - 1];
}

long long fib(int n)
{
	return (n < 2) ? n : fib(n - 1) + fib(n - 2); //CPU拉满
}

long long fib2(int n, long long first = 0, long long second = 1)
{
	if (n <= 2)
		return first + second;
	else
		return fib2(n - 1, second, first + second);
}

long long fib3(int n)
{
	int totol = 0;
	long long a = 0, b = 1;
	for (int i = 1; i < n; i++)
	{
		b = a + b;
		a = b - a;
	}
	return b;
}

//static int* temp;
void Merge(int arry[], int left, int middle, int right)
//合并arry数组中下标为left到middle,和下标为mi ddle+1到right的两个部分
{
	//int temp[right - left + 1];
	int* temp = new int[right - left + 1];
	for (int i = left; i <= right; i++) //copy to temp-array
		temp[i - left] = arry[i];
	int i = left, j = middle + 1;
	for (int k = left; k <= right; k++)
	{
		for (int i = left; i <= right; i++)
			cout << arry[i] << " ";
		cout << "" << endl;
		if (i > middle && j <= right)
		{
			arry[k] = temp[j - left]; j++;
		}
		else if (j > right && i <= middle)
		{
			arry[k] = temp[i - left]; i++;
		}
		else if (temp[i - left] > temp[j - left]) //comparer
		{
			arry[k] = temp[j - left]; j++;
		}
		else if (temp[i - left] <= temp[j - left])
		{
			arry[k] = temp[i - left]; i++;
		}
		for (int i = left; i <= right; i++)
			cout << arry[i] << " ";
		cout << endl << "---------" << endl;
	}
	delete[] temp;
}
void Mergesort(int arry[], int left, int right)//对arry数组中下标为left的元素到 下标为right的元素进行排序
{
	//形参中不存在数组的概念，即使中括号约定了数组的大小，也无效（传递是是一个地址，是数组的首地址）
	if (left >= right)
		return;
	int middle = (right + left) / 2;
	Mergesort(arry, left, middle);
	Mergesort(arry, middle + 1, right);
	Merge(arry, left, middle, right);
}


void shellSort(int array[], int n)
{
	int j;
	for (int gap = n / 2; gap > 0; gap /= 2) {
		//定义一个增长序列，即分割数组的增量,d1=N/2   dk=(d(k-1))/2
		for (int i = gap; i < n; i++) {
			int tmp = array[i];
			for (j = i; j >= gap && tmp < array[j - gap]; j -= gap)
				//将相距为Dk的元素进行排序
				array[j] = array[j - gap];
			array[j] = tmp;
		}
	}
}



int main_dsa()
{

	int A[] = { 2,1,4,3,5,3 ,9,7 };
	int siA = sizeof(A);
	int siAi = sizeof(A[0]);
	int n = sizeof(A) / sizeof(A[0]);

	int s = sum(A, n);
	//mergeSort(A,n);
	Mergesort(A, 0, n - 1);
	//shellSort(A, n);


	return 0;
}