#include"pch.h"
using namespace std;

enum class MyEnum :int
{
	V0 = 0,
	V1 = 1,
	V2 = 2,
	V3 = 3,
	V4 = 4,
	V5 = 5,
	V6 = 6,
};

vector<int> alist()
{
	//return { 1,2,3 };
	vector<int> a(6,0);
	return a;
	//return vector<int> a(6, 0);//not alllow
}

int test_fun(int a)
{
	assert(a ==10);
	return 0;
}

void print_number(int* myInt) {
	assert(myInt != NULL);  //如果myInt!=NULL,程序继续执行，反之程序打印报错信息，程序终止运行
	//assert(myInt!=NULL && 1==0) //检查多个条件
	cout << *myInt << endl;
}

typedef pair<string, int> pair_str;
bool cmp_less(pair_str a, pair_str b)
{
	return a.second < b.second;
}

//bool cmp_less(int a, int b)
//{
//	return a < b;
//}

struct MyStruct
{
	int m_i;
	double m_d;
	MyStruct()
	{
		m_i = 1;
		m_d = 0;
	}
};

bool fun(int a)
{
	int A;
	return A = a;
}

static int mainstl()
{
	test_fun(10);
	vector<int> mlist = alist();
	cout << mlist.at(0) << endl;

	auto name1 = cmp_less;
	auto name2 = typeid(name1).name();
	//auto bl = (*cmp_less)((int)3, (int)2); //函数地址

	//custom segment
	//Segment segm(Vec3(1, 1), Vec3(2, 2));
	//Vec3 vec = segm[0];
	//Vec3 vec2 = segm[1];
	puts("0");

	bool bl2 = fun(1);


	/*float a = 1.3;
	float b = 1.5;

	auto ar = round(a);
	auto br = round(b);*/

	vector<int> a(6, 0);
	a = { 0,1,2,3 };

	vector<int> b = { 1,2,3 };
	vector<int> c = { 4,5,6 };
	a = b;
	a = c;

	//两种访问(越界)
	//cout << b[3] << endl;
	cout << b.at(2) << endl;


	int sm = 1 < 2 ? 1 : 2;

	for (int i = 0; i < 10; i++)
	{
		sm++;
	}
	int n = 2;

	if (n == 2)
		cout << "success" << endl;
	if (!(n == 1))
		cout << "success" << endl;


	//-----------------------------------------------------

	map<string, int> mymap;
	mymap["b"] = 1;
	mymap["a"] = 2;
	mymap["c"] = 3;
	mymap["a"] = 4;

	cout << mymap.size() << endl;
	cout << mymap.max_size() << endl;
	//map<string, int>::iterator mit;
	vector<pair_str> sortVect;
	//vector<map_pair>::iterator vit;

	map<string, int> in_map;
	in_map["b"] = 11;
	in_map["a"] = 2222;
	in_map["c"] = 333;
	in_map["d"] = 4;
	in_map["e"] = 888888;
	for (auto iter = in_map.begin(); iter != in_map.end(); iter++)
	{
		sortVect.push_back(pair_str(iter->first, iter->second));
	}
	sort(sortVect.begin(), sortVect.end(), cmp_less);

	//for (mit = mp.begin(); mit != mp.end(); mit++) 
	//{
	//	vc.push_back(pii(mit->first, mit->second));//这一段是装进vector容器里面
	//	cout << mit->first << "  " << mit->second << endl;
	//}
	//puts("-------------------------------------");


	/*for (auto iter = sortVect.begin(); iter != sortVect.end(); iter++) 
	{
		cout << iter->first << "  " << iter->second << endl;
	}*/
	int num = 3;
	vector<string> minlist;
	for (int i = 0; i < num; i++)
	{
		minlist.push_back(sortVect[i].first);

	}


	string str = "C:\\Users\\Aking\\source\\repostest01//test01.py";
	string name;
	//vector<char> name;
	//for (string::iterator iter = str.end() - 1; iter >= str.begin(); iter--) //正序迭代
	for (string::reverse_iterator iter = str.rbegin(); iter != str.rend(); ++iter) //逆序迭代
	{
		char a = '\\';
		if (*iter != '\\' && *iter != '/')
			name.push_back(*iter);
		else
			break;
		//if (iter == str.begin())
		//	break;
	}
	std::reverse(name.begin(), name.end());


	vector<vector<int>> G2(10); //一种二维数组的写法
	G2[0].push_back(1);
	for (int i = 0; i < 10; i++)
		G2[1].push_back(i);


	vector<int> G[11]; //一种二维数组的写法
	G[1].push_back(2);
	G[1].push_back(3);
	for (int i=0;i<100;i++)
		G[1].push_back(i);

	vector<int> emp; //空vector不可以使用取值方法 front() begin()
	//emp.push_back(1);
	//cout << emp.back() << endl;
	//cout << *emp.begin()<<endl;
	bool ism = emp.empty(); //使用之前先判空

	//测试erase返回值
	map<int, string> mymap3;
	mymap3[1] = "1";
	bool bl1 = mymap3.erase(2);
	MyStruct a1;
	auto b1 = MyStruct();

	map<MyEnum, int> mymap5;
	mymap5[MyEnum(1)] = 1;
	mymap5[MyEnum(2)] = 2; //same key map will refresh
	mymap5[MyEnum(3)] = 3;
	mymap5[MyEnum(4)] = 4;

	//遍历删除 按value删除元素
	for (auto it = mymap5.begin(); it != mymap5.end();)
	{
		//if (it->first == MyEnum(2))
		if((it->second) == 2)
			it = mymap5.erase(it);
		else
			it++;
	}
	return 0;
}

//测试优先队列 priority_queue

static void _test_stl_0()
{
	priority_queue<int> pqueue;//大数在上
	pqueue.push(1);
	pqueue.push(3);
	pqueue.push(2);
	pqueue.push(4);

	int tp = pqueue.top();
	pqueue.pop(); //最上面的 弹出
	pqueue.pop();
	pqueue.pop();
	pqueue.pop();

}

//栈和队列
static void _test_stl_1()
{
	std::stack<int> myStack;
	// 压栈（push）
	myStack.push(1);
	myStack.push(2);
	myStack.push(3);
	// 查看栈顶元素（不删除）
	std::cout << "Top element: " << myStack.top() << std::endl;
	// 弹出栈顶元素（pop）
	myStack.pop();

	std::queue<int> myQueue;

	// 入队（push）
	myQueue.push(1);
	myQueue.push(2);
	myQueue.push(3);
	// 查看队首元素（不删除）
	std::cout << "Front element: " << myQueue.front() << std::endl;
	// 出队（pop）
	myQueue.pop();
	std::cout << "After pop, front element: " << myQueue.front() << std::endl;

}

static int _enrol = []()->int
	{
		_test_stl_0();
		cout << clash::get_filepath_filename(__FILE__) << " finished.\n" << endl;
		return 0;
	}();

