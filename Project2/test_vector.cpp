#include "pch.h"
using namespace std;

// ����һ��+�������
template <typename T>
std::vector<T>& operator+(std::vector<T>& vct1, const std::vector<T>& vct2)
{
	vct1.insert(vct1.end(), vct2.begin(), vct2.end());
	return vct1;
}

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

	void setter(int a, Vec3 vec, char b, double c) {
		this->a = a;
		this->vec = vec;
		this->b = b;
		this->c = c;
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




int main_vector()
{
	auto ss1 = sizeof(void*); //8
	auto ss2 = sizeof(size_t);//8

	A insa;
	insa.setter(1112, Vec3(1,2,3), 'a', 3.14159);
	char* pI;
	pI = (char*)&insa + sizeof(void*); //only one pointor

	int* pInt = (int*)(pI);		pI += sizeof(int); //len(char*)=1byte, as a unit length
	//int* p1 = (int*)&insa + 1;
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

static void test_list()
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

	cout << 1;
}


void test_vec_compare()
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
static int _enrol = []()->int {
	test_list();
	return 0;
}();