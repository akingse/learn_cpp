#include "pch.h"
//#include <windows.h>
using namespace std;
// ���ݽṹ���㷨
// data structure and algorithm

/*
�鲢�����ԭ�� �������������ֳ�ǰ���������֣��ٵݹ�Ľ�ǰ�벿�����ݺͺ�벿�ֵ����ݸ��Թ鲢���򣬵õ������������ݣ�
Ȼ��ʹ��merge�ϲ��㷨���㷨�����룩���������㷨�ϲ���һ��
���磺���N=1����ôֻ��һ������Ҫ����N=2��ֻ��Ҫ����merge������ǰ��ϲ���N=4��...........
Ҳ���ǽ�һ���ܶ����ݵ�����ֳ�ǰ�������֣�Ȼ�󲻶ϵݹ�鲢�����ٺϲ�����󷵻���������顣
�鲢�����ʱ�临�Ӷ�  �鲢�������á����ƽ��ʱ�临�Ӷȶ���O(nlogn)��
���ռ临�Ӷ���O(n)���Ƚϴ�������(nlogn)/2��(nlogn)-n+1��
��ֵ�����Ĵ�����(2nlogn)����˿��Կ������鲢�����㷨�Ƚ�ռ���ڴ棬��ȴ��Ч�ʸ����ȶ��������㷨��

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

void mergeSort(int array[], int tmp[], int m_left, int m_right)
{
	if (m_left < m_right)
	{
		int center = (m_left + m_right) / 2;//ȡ������е�
		mergeSort(array, tmp, m_left, center);//�鲢���������ǰ�벿��
		mergeSort(array, tmp, center + 1, m_right);//�鲢��������ĺ�벿��
		merge(array, tmp, m_left, center + 1, m_right);//�������ǰ��벿�ֺϲ�
	}
}
/*
 * ���򵥵ĺϲ�����
 */

void mergeSort(int array[], int n)
{
	//int tmp[n];//����һ�������ϲ�������
	int* tmp = new int[n];
	//memcpy(tmp, array, n);
	mergeSort(array, tmp, 0, n - 1);//�������������������ֵ������յ�
}


int sum(int A[], int n)
{
	return
		(n == 0) ?
		0 : sum(A, n - 1) + A[n - 1];
}

long long fib(int n)
{
	return (n < 2) ? n : fib(n - 1) + fib(n - 2); //CPU����
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
void Merge(int arry[], int m_left, int middle, int m_right)
//�ϲ�arry�������±�Ϊleft��middle,���±�Ϊmi ddle+1��right����������
{
	//int temp[right - left + 1];
	int* temp = new int[m_right - m_left + 1];
	for (int i = m_left; i <= m_right; i++) //copy to temp-array
		temp[i - m_left] = arry[i];
	int i = m_left, j = middle + 1;
	for (int k = m_left; k <= m_right; k++)
	{
		for (int i = m_left; i <= m_right; i++)
			cout << arry[i] << " ";
		cout << "" << endl;
		if (i > middle && j <= m_right)
		{
			arry[k] = temp[j - m_left]; j++;
		}
		else if (j > m_right && i <= middle)
		{
			arry[k] = temp[i - m_left]; i++;
		}
		else if (temp[i - m_left] > temp[j - m_left]) //comparer
		{
			arry[k] = temp[j - m_left]; j++;
		}
		else if (temp[i - m_left] <= temp[j - m_left])
		{
			arry[k] = temp[i - m_left]; i++;
		}
		for (int i = m_left; i <= m_right; i++)
			cout << arry[i] << " ";
		cout << endl << "---------" << endl;
	}
	delete[] temp;
}
void Mergesort(int arry[], int m_left, int m_right)//��arry�������±�Ϊleft��Ԫ�ص� �±�Ϊright��Ԫ�ؽ�������
{
	//�β��в���������ĸ����ʹ������Լ��������Ĵ�С��Ҳ��Ч����������һ����ַ����������׵�ַ��
	if (m_left >= m_right)
		return;
	int middle = (m_right + m_left) / 2;
	Mergesort(arry, m_left, middle);
	Mergesort(arry, middle + 1, m_right);
	Merge(arry, m_left, middle, m_right);
}


void shellSort(int array[], int n)
{
	int j;
	for (int gap = n / 2; gap > 0; gap /= 2) {
		//����һ���������У����ָ����������,d1=N/2   dk=(d(k-1))/2
		for (int i = gap; i < n; i++) {
			int tmp = array[i];
			for (j = i; j >= gap && tmp < array[j - gap]; j -= gap)
				//�����ΪDk��Ԫ�ؽ�������
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