#include "pch.h"
using namespace std;
using namespace Eigen;
using namespace games;

//�ѣ�����ѣ� heap
//���Ǳ���һ����ȫ�������� C++������std::priority_queue
//����ʵ�Ǻܻ��������ݽṹ���������ǳ�ѧ��ѧϰ�ĵ�һ���������ݽṹ���������顢�������С�ջ���������ݽṹ֮��
//�ѵ�˼�������ˡ�����������棻�ѵ�ʵ�ָ�Ч���š���ֻ��Ҫά��һ�����飻�ѵĲ��롢ɾ�����޸ĵ�ʱ�临�Ӷȶ���һ���� nlogn
//�ѵ��������� ���ϵ�����Shift Up�������µ�����Shift Down��

//AVLƽ������ƽ������<=1��������������LR˫����RL˫��,


template<class K, class V>
struct AVLTreeNode
{
	AVLTreeNode<K, V>* _left;//��ڵ�
	AVLTreeNode<K, V>* _right;//�ҽڵ�
	AVLTreeNode<K, V>* _parent;//˫�׽ڵ�

	pair<K, V> _kv;//���������
	int _bf;//ƽ������

	//���캯��
	AVLTreeNode(const pair<K, V>& kv)
		:_left(nullptr)
		, _right(nullptr)
		, _parent(nullptr)
		, _kv(kv)
		, _bf(0)
	{}
};

//����� https://blog.csdn.net/leijie1123/article/details/127649234
/*
1. ÿ����㲻�Ǻ�ɫ���Ǻ�ɫ
2. ���ڵ��Ǻ�ɫ��
3. ���һ���ڵ��Ǻ�ɫ�ģ��������������ӽ���Ǻ�ɫ��
4. ����ÿ����㣬�Ӹý�㵽�����к��Ҷ���ļ�·���ϣ���������ͬ��Ŀ�ĺ�ɫ���
5. ÿ��Ҷ�ӽ�㶼�Ǻ�ɫ��(�˴���Ҷ�ӽ��ָ���ǿս��)


�������AVL�����Ǹ�Ч��ƽ�����������ɾ�Ĳ��ʱ�临�Ӷȶ�O(logN)���������׷
�����ƽ�⣬��ֻ�豣֤�·�����������·����2������Զ��ԣ������˲������ת�Ĵ�����
�����ھ���������ɾ�Ľṹ�����ܱ�AVL�����ţ����Һ����ʵ�ֱȽϼ򵥣�����ʵ�������к�
�������ࡣ

*/

//stl���ȶ��� https://blog.csdn.net/xingzi201/article/details/119884227
//priority_queue<Type, Container, Functional>

bool myCom(int a, int b) 
{
	return a % 10 > b % 10;
}

static void _test0()
{
	srand(time(NULL));
	//priority_queue<int> pq1; // Ĭ�������ѣ������
	priority_queue<int, vector<int>, greater<int>> pq1; //С����

	std::cout << "start..." << endl;
	for (int i = 0; i < 10; i++) 
	{
		int t = rand() % 100;
		//cout << t << ends;
		pq1.push(t);
	}
	std::cout << endl;
	while (!pq1.empty())
	{
		cout << pq1.top() << ends;
		pq1.pop();
	}
	cout << endl;

	// ʹ���Զ���cmp����
	priority_queue<int, vector<int>, function<bool(int, int)>> pq3(myCom);

}
static void _test1()
{
	QEMVertex v1 = { Vector3d(1,1,1),1 };
	QEMVertex v2 = { Vector3d(1,1,1),2 };
	QEMVertex v3 = { Vector3d(1,1,1),3 };
	QEMVertex v4 = { Vector3d(1,1,1),4 };
	std::priority_queue<QEMVertex> pq;
	pq.push(v2);
	pq.push(v1);
	pq.push(v4);
	pq.push(v3);

	pq.pop();

	return;
}

static int enrol = []()->int
	{
		//_test0();
		_test1();
		cout << "test_data_structure finished.\n" << endl;
		return 0;
	}();
