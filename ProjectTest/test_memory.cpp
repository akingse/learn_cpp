#include "pch.h"
using namespace std;
/*
### �ڴ�й©�����

1��new without delete

2������ָ���໥����

3��������

4���ṹ��


*/


int& func()
{
	int* a = new int(1);
	return *a;
}

//������
class Manage
{
public:
    static Manage& get_instance();
    //static s_instance();
};

//������,ֻ��һ��ʵ������

//Manage* Manage::s_instance = nullptr;

//�ṹ��
struct C_struct
{
    double vd; //����©
    string ch = "cha"; //string�ɱ䳤

    ~C_struct()
    {

    }
};
//delete��Ҫָ������

//����Ҫ��


int main_me()
{
	int& ra = func(); //������ջ��Ұָ��
	ra = 2;

	char* pchar = new char;
	delete pchar; //�����������ͷ��ڴ棬�������ÿգ�
	pchar = nullptr;

	//�ǳ��
	//��ֵ��ֵ
	int a = 123;
	int b = a;//�
	int& c = a;//ǳ��
	int* p = &a;//ǳ��
    //Manager::get_instance().m_id2name[i++] = "name";

	return 0;
}

static void test_sharedptr_1()
{
    std::shared_ptr<int> ptr = make_shared<int>(1);
    ptr.reset(); //shared_ptr�ͷ��ڴ�
    return;
}


static int enrol = []()->int
    {
        test_sharedptr_1();
        cout << "test_memory_leak finished.\n" << endl;
        return 0;
    }();
