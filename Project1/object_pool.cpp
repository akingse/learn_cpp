#include "pch.h"
using namespace std;

// �ڴ����������黹�ÿ�
// list ������ double��С�ڴ����

template<class Object>
class ObjectPool2
{

private:
	size_t m_size;
	//std::list<Object> m_pool;
	std::vector<Object> m_pool;
	//Gene gene;

public:
	ObjectPool2(size_t size) :m_size(size)
	{
		//_nSize = size;
		for (size_t n = 0; n < m_size; n++)
		{
			m_pool.push_back(move(Object()));
		}
	}
	virtual ~ObjectPool2()
	{
		auto iter = m_pool.begin();
		//auto name = typeid(iter).name();
		while (iter != m_pool.end())
		{
			//delete* iter;
			++iter;
		}
		m_size = 0;
	}

	Object requestObject()
	{
		Object pObj;// = NULL;
		if (m_size == 0)
		{
			pObj = Object(); //Object();
		}
		else
		{//Ϊ�˷�ֹ��ͻ��ÿ�λ�ȡ�����ʱ�򣬻�������ͷ���л�ȡ��Ȼ��ͷɾ��
			pObj = m_pool[m_size];
			//m_pool.pop_back();
			--m_size;
		}
		return pObj;
	}

	void returnObject(Object pObj)
	{
		//���ȡ������ͬ�����ն���ʱ����������뵽����ص�β���������Ͳ���ͻ�ȡ����������ͻ��
		//m_pool.push_back(pObj);
		++m_size;
	}

};


template<class Object>
class ObjectPool
{

private:
	size_t m_size;
	std::list<Object*> m_pool;
	//Gene gene;

public:
	ObjectPool(size_t size) :m_size(size)
	{
		//_nSize = size;
		for (size_t n = 0; n < m_size; n++)
		{
			m_pool.push_back(new Object());
		}
	}
	virtual ~ObjectPool()
	{
		auto iter = m_pool.begin();
		//auto name = typeid(iter).name();
		while (iter != m_pool.end())
		{
			delete* iter;
			++iter;
		}
		m_size = 0;
	}

	Object* requestObject()
	{
		Object* pObj = NULL;
		if (m_size == 0)
		{
			pObj = new Object(); //Object();
		}
		else
		{//Ϊ�˷�ֹ��ͻ��ÿ�λ�ȡ�����ʱ�򣬻�������ͷ���л�ȡ��Ȼ��ͷɾ��
			pObj = m_pool.front(); //���ص�һ��Ԫ��
			m_pool.pop_front(); //ɾ����һ��Ԫ��
			--m_size;
		}
		return pObj;
	}

	void returnObject(Object* pObj)
	{
		//���ȡ������ͬ�����ն���ʱ����������뵽����ص�β���������Ͳ���ͻ�ȡ����������ͻ��
		m_pool.push_back(pObj);
		++m_size;
	}

};




class Test
{
public:
	double m_a;
	Test(double a=1):m_a(a) {}
	~Test() {}
	void Print()
	{
		//cout << "Test" << endl;
	}
};


//�ڴ���Ƭ
//���������ڴ�

int mainpool()
{
	//Gene gnrc;
	//cout << sizeof(gnrc) << endl;
	//cout << sizeof(Gene(1)) << endl;
	//cout << sizeof(Gene(1.0)) << endl;
	//cout << sizeof(Vec2) << endl;
	//cout << sizeof(Vec3) << endl;
	////cout << sizeof(Gene(Vec2())) << endl;
	//Gene vec= Gene(Vec3());
	//cout << sizeof(vec) << endl;

	std::chrono::hours;   // ʱ
	std::chrono::minutes;   // ��
	std::chrono::seconds;   // ��
	std::chrono::milliseconds;   // ����
	std::chrono::microseconds;   // ΢��
	std::chrono::nanoseconds;   // ����

	//������ʱ
	//double a[long long(1e5)]; //vs������1e6
	//double* a = new double[1e6];
	vector<double> tVec;
	vector<double*> ptVec;
	vector<Vec3> vVec(1e6);
	vector<Vec3*> pvVec(1e6);
	Vec3* a = new Vec3[1e6];

	auto startTime0 = std::chrono::system_clock::now();
	//for (int j = 0; j < 1e1; j++)
		for (int i = 0; i < 1e6; i++)
		{
			//double a; //������� debug�� 1ms
			//double a = i; //���帳ֵ 2ms/million
			//a[i] = i; //������ֵ 2ms/million
			//a[i] = i; //new������ֵ 2ms/million
			// 
			//tVec.push_back(i); //70ms
			double* a = new double; //new���� 100ms/million
			//delete a; // �ܼ�150ms
			//a = nullptr; //�ÿ��ٶȺ��Բ���
			//double* a = new double(i); //�ٶȽӽ� 100ms/million
			//ptVec.push_back(a); //170ms
			//tVec.push_back(*a); //170ms
			//delete a; // �ܼ�150ms
			
			
			//Vec3 vec; //6ms
			//Vec3 vec(i,i,i); //6ms
			//vVec.push_back(Vec3(i, i, i)); //240ms/million
			//Vec3* vec = new Vec3; //130ms �����ʼ���޹�
			//delete vec; //�ܼ�170ms
			
			//Vec3* vec = new Vec3(i,i,i); //130ms
			//pvVec.push_back(vec); // 230ms/million
			//delete vec; //�ܼ�170ms
			
			//a[i] = Vec3(i, i, i); //������ֵ���� 20ms
			//pvVec[i] = &vec; //����+ȡַ��ֵ 10ms

			//pvVec[i] = new Vec3(i, i, i); //new�ܷ�ʱ 150ms

		}
	
		for (int i = 0; i < 1e5; i++)
			auto iter = a[i];
		//cout << i <<  << endl;


	auto endTime0 = std::chrono::system_clock::now();
	long long timeusing0 = std::chrono::duration_cast<std::chrono::microseconds>(endTime0 - startTime0).count();
	std::cout << "time= " << timeusing0 << "microseconds" << std::endl;


	

	ObjectPool2<Vec3> obj(1e6); //1e6
	//auto startTime = std::chrono::system_clock::now();
	for (int i = 0; i < 1e6; i++)
	{
		Vec3 pA;
		//Gene* pA;
		//Vec3 pA = obj.requestObject();
		pA.x = i;
		pA.y = i;
		pA.z = i;
		//pA->m_a=i;
		//pA->m_imp;
		//obj.returnObject(pA);
	}

	//auto name = typeid(startTime).name();
	//cout << name << endl;
	//for (int i = 0; i < 1e6; i++)
	//{
	//	Vec3* pA;
	//	//Gene* pA;
	//	pA = obj.requestObject();
	//	pA->x=i;
	//	pA->y=i;
	//	pA->z=i;
	//	//pA->m_a=i;
	//	//pA->m_imp;
	//	//obj.returnObject(pA);
	//}
	//��ʱ
	//auto endTime = std::chrono::system_clock::now();
	//std::cout << "time:" << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() << std::endl;


	int num = 1;
	float totaltime = 0.0;
	vector<int> randList;

	for (int j = 0; j < num; j++)
	{ 
		auto startTime = std::chrono::system_clock::now();
		for (int i = 0; i < 1e6; i++)
		{
			double* a = new double(i);
			//*a = i;
			delete a;
		}
		auto endTime = std::chrono::system_clock::now();
		float timeusing = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
		std::cout << "new+del time:" << timeusing << " ����" << std::endl;
		//���Խ����debug 150ms; release 50ms
		totaltime += timeusing;
	}
	//std::cout << "ave time:" << totaltime/ float(num) << " ����" << std::endl;

	//�ֱ���� new-delete
	vector<double*> numlist;
	vector<double> randlist;
	for (int i = 0; i < 1e6; i++)
	{
		//numlist.push_back(nullptr);
		randlist.push_back(rand());

	}
	auto startTime = std::chrono::system_clock::now();
	for (int i = 0; i < 1e6; i++)
	{
		double* a = new double(i);
		numlist.push_back(a);
		//numlist[i] = &(randlist[i]); //numlist[i]=a; //push_back(double) push_back(double*) ʱ��һ��
	}
	auto endTime = std::chrono::system_clock::now();
	float timeusing = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
	std::cout << "new time:" << timeusing << " ����" << std::endl;

	auto startTime2 = std::chrono::system_clock::now();
	for (int i = 0; i < 1e6; i++)
	{
		//double* a = numlist[i];
		delete numlist[i];
		numlist[i] = nullptr;
	}
	auto endTime2 = std::chrono::system_clock::now();
	float timeusing2 = std::chrono::duration_cast<std::chrono::milliseconds>(endTime2 - startTime2).count();
	std::cout << "delete time:" << timeusing2 << " ����" << std::endl;

	srand((unsigned)time(NULL)); //0-32767
	vector<int> count = {0,0,0,0,0,0,0,0,0,0};
	startTime2 = std::chrono::system_clock::now();
	for (int i = 0; i < 1e6; i++) //ʵ��֤����������ȶ�
	{
		int ran = int(10 * rand() / 32767.0);
		switch (ran)
		{
		case 0:
		{
			//count[0]++;
			double* a11 = new double(i);
			delete a11;
			break;
		}
		case 1:
		{
			//count[1]++;
			string* str = new string("hello world");
			delete str;
			break;
		}
		case 2:
		{
			//count[2]++;
			Vec3* v = new Vec3();
			delete v;
			break;
		}
		case 3:
		{
			//count[3]++;
			double* p = new double[100];
			delete[] p;
			break;
		}
		case 4:
		{
			//count[4]++;
			char* cs = new char[64];
			delete[] cs;
			break;
		}
		case 5:
		{
			//count[5]++;
			Gene gene1;
			break;
		}
		case 6:
		{
			//count[6]++;
			Gene* gnrc = new Gene;
			delete gnrc;
			break;
		}
		case 7:
		{
			int* ptr = new int[32];
			delete ptr;
			//count[7]++;
			break;
		}
		case 8:
		{
			typedef vector<int> VECINT;
			VECINT* vec = new VECINT;
			delete vec;
			//count[8]++;
			break;
		}
		case 9:
		{
			typedef map<char*, int> MAP;
			MAP* map = new MAP;
			delete map;
			//count[9]++;
			break;
		}
		}
	}
	endTime2 = std::chrono::system_clock::now();
	timeusing2 = std::chrono::duration_cast<std::chrono::milliseconds>(endTime2 - startTime2).count();
	std::cout << "rand time:" << timeusing2 << " ����" << std::endl;

	return 0;
}