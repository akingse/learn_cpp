#include "pch.h"
using namespace std;
using namespace Eigen;
using namespace para;

// ����һ��+�������
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

//ָ��ƫ��
//ָ��ƫ�ƻ�ȡ���˽�г�Ա
//��Υ������ķ�װԭ����ʹ��ָ�������Ҳ������ȫ�����Բ�����ʹ�á�
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
	//��������char b���ᵼ���ַ����е��ַ���Ч�����뽫��һλ�ַ�����Ϊ'\0'
	//�ַ���û���ҵ�'\0'��������ַ����Ľ������
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
	// ����һ��insert() ����
	//v1.insert(v1.end(), v2.begin(), v2.end());
	//v1.insert(v1.end(), v3.begin(), v3.end());

	// ������������һ��+�������
	v1 = v1 + v2 + v3;

	// ��������copy()����
	int s=(int)v1.size();
	v1.resize(s+v2.size());
	std::copy(v2.begin(),v2.end(),v1.begin()+s);
	int s2=(int)v1.size();
	v1.resize(s2+v3.size());
	std::copy(v3.begin(),v3.end(),v1.begin()+s2);

		// ��ʾ
	for (int i = 0; i < v1.size(); i++)
	{
		cout << "v1=" << v1[i] << endl;
	}

	std::multimap<int,int> umap;
	//umap.insert(pair<int, int>(1, 1));
	//umap.insert(pair<int, int>(1, 2));
	umap.insert({ 1, 1 });
	umap.insert({ 1, 2 });
	//umap.insert(1, 2); //������������
	for (auto& iter : umap)
	{
		cout << iter.first << ":" << iter.second << endl;
	}

	/*
	����std::map/unordered_map�в���Ԫ��ʱ����������ʹ��emplace��emplace�Ĳ��������Ԫ��key�����ڣ�������Ԫ�أ����򲻲��롣
	������Ԫ���Ѵ���ʱ��emplace�Իṹ��һ�δ������Ԫ�أ����жϲ���Ҫ�������������Ԫ����������˽�����һ�ζ��๹�������������
	c++17������try_emplace��������������⡣ͬʱtry_emplace�ڲ����б��н�key��value�ֿ�����˽���ԭ�ع�����﷨��emplace���Ӽ��
	*/
	return 0;
}

static void test_vector_2()
{

	bool sign = true;
	double a = (2 * sign - 1) * 1.0;
	double b = (2 * (!sign)-1) * 1.0;

	//˫��������

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
	auto res1 = amap.try_emplace("����1", 1);
	auto res2 = amap.try_emplace("Ĭ������1", 2);
	auto res3 = amap.insert({ "����2", 2 });

	cout << 1;
}

//����map
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
		// �ҵ��˴��ڵ���target��Ԫ��
		int dist1 = std::distance(vec.begin(), it1);
		std::cout << "�ҵ���Ŀ��ֵ" << target1 << "��λ��Ϊ��" << dist1 << std::endl;
	}
	else 
	{
		// û���ҵ����ڵ���target��Ԫ��
		std::cout << "û���ҵ�Ŀ��ֵ" << target1 << std::endl;
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

//����ɾ��
static void test_vector_5()
{
	std::vector<int> vec = { 1, 2, 3, 4, 5 };
	vec.erase(vec.begin() + 1);
	// vector����������
	std::vector<int>& change = std::vector<int>();
	change = vec; //�޸�ԭʼvecʧ��
	for (int i = 0; i < change.size(); i++)
		change[i] = 2 * change[i];
	std::vector<int>* change2; // = &std::vector<int>();
	change2 = &vec; //�޸ĳɹ�
	for (int i = 0; i < change2->size(); i++)
		(*change2)[i] = 2 * (*change2)[i];

	return;
}

//����operator����
static void test_vector_6()
{
	std::vector<int> vec1 = { 1, 2, 3, 4, 5 };
	std::vector<int> vec2 = { 6,7,8,9 };
	std::vector<int> vec3 = vec1 + vec2;
	vec3 += vec2;

	return;
}

//����erase
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


//<algorithm>����
static void test_vector_8()
{
	vector<int> vec = { 1,2,3 };
	std::for_each(vec.begin(), vec.end(), [](int& iter) {iter *= 2; });

	//transform
	//������Ҫ�����Ƕ����뷶Χ�е�ÿ��Ԫ��Ӧ��һ��ָ���Ĳ���������ͨ����һ�������������󣩣��������д�뵽һ�������Χ��
	std::vector<int> numbers = { 1, 2, 3, 4, 5 };
	std::vector<int> squares(numbers.size());

	// ʹ�� std::transform ����ƽ��
	std::transform(numbers.begin(), numbers.end(), squares.begin(),
		[](int x) { return x * x; }); // Lambda ���ʽ


	std::vector<int> vec1 = { 1, 2, 3 };
	std::vector<int> vec2 = { 4, 5, 6 };
	std::vector<int> sums(vec1.size());

	// ʹ�� std::transform �����ӦԪ�صĺ�
	std::transform(vec1.begin(), vec1.end(), vec2.begin(), sums.begin(),
		[](int a, int b) { return a + b; }); // Lambda ���ʽ

}

static int _enrol = []()->int 
	{
		test_vector_6();
		test_vector_7();
		test_vector_8();
		cout << "test_vector finished.\n" << endl;
		return 0;
	}();

