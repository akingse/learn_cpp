#include "pch.h"
using namespace std;
using namespace Eigen;
using namespace games;

//堆（二叉堆） heap
//堆是必须一个完全二叉树， C++内置了std::priority_queue
//堆其实是很基础的数据结构，堆往往是初学者学习的第一个树形数据结构，排在数组、链表、队列、栈等线形数据结构之后
//堆的思想简洁明了——大的在上面；堆的实现高效优雅——只需要维护一个数组；堆的插入、删除、修改的时间复杂度都是一样的 nlogn
//堆的两个操作 向上调整（Shift Up）和向下调整（Shift Down）

//AVL平衡树，平衡因子<=1，左旋、右旋、LR双旋、RL双旋,


template<class K, class V>
struct AVLTreeNode
{
	AVLTreeNode<K, V>* _left;//左节点
	AVLTreeNode<K, V>* _right;//右节点
	AVLTreeNode<K, V>* _parent;//双亲节点

	pair<K, V> _kv;//所存的内容
	int _bf;//平衡因子

	//构造函数
	AVLTreeNode(const pair<K, V>& kv)
		:_left(nullptr)
		, _right(nullptr)
		, _parent(nullptr)
		, _kv(kv)
		, _bf(0)
	{}
};

//红黑树 https://blog.csdn.net/leijie1123/article/details/127649234
/*
1. 每个结点不是红色就是黑色
2. 根节点是黑色的
3. 如果一个节点是红色的，则它的两个孩子结点是黑色的
4. 对于每个结点，从该结点到其所有后代叶结点的简单路径上，均包含相同数目的黑色结点
5. 每个叶子结点都是黑色的(此处的叶子结点指的是空结点)


红黑树和AVL树都是高效的平衡二叉树，增删改查的时间复杂度都O(logN)，红黑树不追
求绝对平衡，其只需保证最长路径不超过最短路径的2倍，相对而言，降低了插入和旋转的次数，
所以在经常进行增删的结构中性能比AVL树更优，而且红黑树实现比较简单，所以实际运用中红
黑树更多。

*/

//stl优先队列 https://blog.csdn.net/xingzi201/article/details/119884227
//priority_queue<Type, Container, Functional>

bool myCom(int a, int b) 
{
	return a % 10 > b % 10;
}

static void _test0()
{
	srand(time(NULL));
	//priority_queue<int> pq1; // 默认是最大堆，大根堆
	priority_queue<int, vector<int>, greater<int>> pq1; //小根堆

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

	// 使用自定义cmp函数
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
