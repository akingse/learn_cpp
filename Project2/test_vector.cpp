#include "pch.h"
using namespace std;
using namespace Eigen;
using namespace para;

// 重载一下+号运算符
template <typename T>
std::vector<T> operator+(const std::vector<T>& vctA, const std::vector<T>& vctB)
{
	std::vector<T> res = vctA;
	res.insert(res.end(), vctB.begin(), vctB.end());
	return res;
}

template <typename T>
void operator+=(std::vector<T>& vctA, const std::vector<T>& vctB)
{
	vctA.insert(vctA.end(), vctB.begin(), vctB.end());
}

#pragma region MyRegion

//指针偏移
//指针偏移获取类的私有成员
//但违反了类的封装原则，在使用指针的类中也极不安全，所以不建议使用。
#pragma pack(1) // memory alignment

class Base
{
public:
	virtual void inheriFun1() {}
	virtual void inheriFun2() {}
	virtual void inheriFun3() {}
};
class A :public Base
{
public:
	virtual void inheriFun1() override {}
	virtual void inheriFun2() override {}
	virtual void inheriFun3() override {}

	void setter(int _a, Vec3 _vec, char _b, double _c) {
		this->a = _a;
		this->vec = _vec;
		this->b = _b;
		this->c = _c;
	}
private:
	int a;
	Vec3 vec;
	
	char b;
	double c;
	//单独设置char b，会导致字符串中的字符无效，必须将下一位字符设置为'\0'
	//字符串没有找到'\0'，这个是字符串的结束标记
};
#pragma pack() //end memory alignment

static int test_vector_1()
{
	auto ss1 = sizeof(void*); //8
	auto ss2 = sizeof(size_t);//8

	A insa;
	insa.setter(1112, Vec3(1,2,3), 'a', 3.14159);
	char* pI;
	pI = (char*)&insa + sizeof(void*); //only one pointor

	//int* p1 = (int*)&insa + 1;
	int* pInt = (int*)(pI);		pI += sizeof(int); //len(char*)=1byte, as a unit length
	Vec3* pVec = (Vec3*)(pI);	pI += sizeof(Vec3);
	char* pChar = (char*)(pI);	pI += sizeof(char); //&insa + sizeof(Vec3);
	double* pDouble = (double*)(pI); //&insa + sizeof(char);
	// (Vec3*)0x00000032f42fed98

	int a1[] = { 1,2,3 }, a2[] = { 4,5,6 };
	vector<int> v1(a1, a1 + 3);
	vector<int> v2(a2, a2 + 3);
	vector<int> v3 = { 7,8,9 };
	// 方法一：insert() 函数
	//v1.insert(v1.end(), v2.begin(), v2.end());
	//v1.insert(v1.end(), v3.begin(), v3.end());

	// 方法二：重载一下+号运算符
	v1 = v1 + v2 + v3;

	// 方法三：copy()函数
	int s=(int)v1.size();
	v1.resize(s+v2.size());
	std::copy(v2.begin(),v2.end(),v1.begin()+s);
	int s2=(int)v1.size();
	v1.resize(s2+v3.size());
	std::copy(v3.begin(),v3.end(),v1.begin()+s2);

		// 显示
	for (int i = 0; i < v1.size(); i++)
	{
		cout << "v1=" << v1[i] << endl;
	}

	std::multimap<int,int> umap;
	//umap.insert(pair<int, int>(1, 1));
	//umap.insert(pair<int, int>(1, 2));
	umap.insert({ 1, 1 });
	umap.insert({ 1, 2 });
	//umap.insert(1, 2); //不能这样插入
	for (auto& iter : umap)
	{
		cout << iter.first << ":" << iter.second << endl;
	}

	/*
	在向std::map/unordered_map中插入元素时，我们往往使用emplace，emplace的操作是如果元素key不存在，则插入该元素，否则不插入。
	但是在元素已存在时，emplace仍会构造一次待插入的元素，在判断不需要插入后，立即将该元素析构，因此进行了一次多余构造和析构操作。
	c++17加入了try_emplace，避免了这个问题。同时try_emplace在参数列表中将key和value分开，因此进行原地构造的语法比emplace更加简洁
	*/
	return 0;
}

static void test_vector_2()
{

	bool sign = true;
	double a = (2 * sign - 1) * 1.0;
	double b = (2 * (!sign)-1) * 1.0;

	//双向环形链表

	//std::list<double> alist;
	std::queue<double> alist;
	for (int i = 0; i < 5; i++)
		alist.push(i);

	double num = 0;
	double temp = 0;
	double i = alist.front();
	
	//for (auto iter = alist.begin(); iter != alist.end(); iter++)
	//{
	//	//temp = *iter + *(++iter) + *(++(++iter));
	//	temp = *iter + *(iter++) + *((iter++)++);
	//	num += *iter;

	//}

	for (int j = 2; j < 7; j++)
	{
		//num += alist
	}

	std::map<string, int> amap;
	auto res1 = amap.try_emplace("类型1", 1);
	auto res2 = amap.try_emplace("默认类型1", 2);
	auto res3 = amap.insert({ "类型2", 2 });

	cout << 1;
}

//测试map
static void test_vector_3()
{
	std::map<BPParaVec, int> amap;
	amap.insert({ BPParaVec(3, 6), 1 });
	amap.insert({ BPParaVec(3, 7), 1 });
	amap.insert({ BPParaVec(4, 6), 1 });
	amap.insert({ BPParaVec(5, 6), 1 });
	amap.insert({ BPParaVec(6, 4), 1 });
	amap.insert({ BPParaVec(-6, 4), 1 });
	amap.insert({ BPParaVec(-6, -4), 1 });
	amap.insert({ BPParaVec(10, 11), 1 });

	//amap.try_emplace(BPParaVec(3, 6), 1);
	//amap.try_emplace(BPParaVec(3, 7), 1);
	//amap.try_emplace(BPParaVec(4, 6), 1);
	//amap.try_emplace(BPParaVec(5, 6), 1);
	//amap.try_emplace(BPParaVec(6, 4), 1);
	//amap.try_emplace(BPParaVec(-6, 4), 1);
	//amap.try_emplace(BPParaVec(-6, -4), 1);
	//amap.try_emplace(BPParaVec(10, 11), 1);

	auto it = amap.find(BPParaVec(4, 6));

}

//wrote by chatgpt
static int test_vector_4()
{
	//							 |         |		
	std::vector<double> vec{ 1.2, 3.4, 3.5, 5.6 };
	if (std::find(vec.begin(), vec.end(), 1.2) != vec.end())
	{
		cout << "" << endl;
	}
	//double a1 = vec[-1]; //not support
	double a2 = vec.front();
	double a3 = vec.back();
	double a4 = vec[vec.size() - 1];
	double a5 = vec[vec.size() - 2];

	double target0 = 0;
	double target1 = 2;
	double target2 = 4;
	double target3 = 7;
	auto it0 = std::lower_bound(vec.begin(), vec.end(), target0);
	auto it1 = std::lower_bound(vec.begin(), vec.end(), target1);
	auto it2 = std::lower_bound(vec.begin(), vec.end(), target2);
	auto it3 = std::lower_bound(vec.begin(), vec.end(), target3);
	int dist0 = std::distance(vec.begin(), it0); //0
	int dist1 = std::distance(vec.begin(), it1); //1
	int dist2 = std::distance(vec.begin(), it2); //3
	int dist3 = std::distance(vec.begin(), it3); //4

	if (it0 == vec.end())
		cout << "out";
	if (it3 == vec.end())
		cout << "out";
	if (it1 != vec.end()) 
	{
		// 找到了大于等于target的元素
		int dist1 = std::distance(vec.begin(), it1);
		std::cout << "找到了目标值" << target1 << "，位置为：" << dist1 << std::endl;
	}
	else 
	{
		// 没有找到大于等于target的元素
		std::cout << "没有找到目标值" << target1 << std::endl;
	}

	vector<int> vec3 = { 1,2,3 };
	void* ptr1 = &vec3;
	void* ptr2 = vec3.data();
	vector<int> vec4 = *reinterpret_cast<vector<int>*>(ptr1);
	int i0 = *(int*)ptr2;
	int i1 = *(int*)ptr2 + 1;
	int i2 = *(int*)ptr2 + 2;
	return 0;

}
#pragma endregion

//测试删除
static void test_vector_5()
{
	std::vector<int> vec = { 1, 2, 3, 4, 5 };
	vec.erase(vec.begin() + 1);
	// vector索引的引用
	std::vector<int>& change = std::vector<int>();
	change = vec; //修改原始vec失败
	for (int i = 0; i < change.size(); i++)
		change[i] = 2 * change[i];
	std::vector<int>* change2; // = &std::vector<int>();
	change2 = &vec; //修改成功
	for (int i = 0; i < change2->size(); i++)
		(*change2)[i] = 2 * (*change2)[i];

	return;
}

//测试operator重载
static void test_vector_6()
{
	std::vector<int> vec1 = { 1, 2, 3, 4, 5 };
	std::vector<int> vec2 = { 6,7,8,9 };
	std::vector<int> vec3 = vec1 + vec2;
	vec3 += vec2;

	return;
}

//测试erase
static void test_vector_7()
{
	vector<int> vec = { 1,2,3 };
	std::for_each(vec.begin(), vec.end(), [](int& iter) {iter *= 2; });

	while (!vec.empty())
	{
		auto it = vec.begin();
		vec.erase(it);
	}

	//for (auto iter = vec.begin(); iter != vec.end();)
	//	vec.erase(iter);
	//vec.erase(std::remove(vec.begin(), vec.end(), 1), vec.end());

	//auto iter = vec.begin();
	//while (iter != vec.end())
	//	vec.erase(iter);


	for (int i = 0; i < vec.size();)
	{
		vec.erase(vec.begin());

	}

}


//<algorithm>函数
static void test_vector_8()
{
	vector<int> vec = { 1,2,3 };
	std::for_each(vec.begin(), vec.end(), [](int& iter) {iter *= 2; });

	//transform
	//它的主要功能是对输入范围中的每个元素应用一个指定的操作（操作通常是一个函数或函数对象），并将结果写入到一个输出范围。
	std::vector<int> numbers = { 1, 2, 3, 4, 5 };
	std::vector<int> squares(numbers.size());

	// 使用 std::transform 计算平方
	std::transform(numbers.begin(), numbers.end(), squares.begin(),
		[](int x) { return x * x; }); // Lambda 表达式


	std::vector<int> vec1 = { 1, 2, 3 };
	std::vector<int> vec2 = { 4, 5, 6 };
	std::vector<int> sums(vec1.size());

	// 使用 std::transform 计算对应元素的和
	std::transform(vec1.begin(), vec1.end(), vec2.begin(), sums.begin(),
		[](int a, int b) { return a + b; }); // Lambda 表达式

}

static int _enrol = []()->int 
	{
		test_vector_6();
		test_vector_7();
		test_vector_8();
		cout << "test_vector finished.\n" << endl;
		return 0;
	}();

