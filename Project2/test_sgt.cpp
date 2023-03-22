#include "pch.h"

using namespace std;
class Only
{
private:
    Only() {}   //���캯����˽�еģ������Ͳ����������ط�������ʵ��
    static Only* p;  //����һ��Ψһָ��ʵ���ľ�ָ̬�룬������˽�еġ�
    static int m_b;
    /*
    ��̬���ݱ������������ڵ�ȫ�ֱ��������ڷǾ�̬���ݳ�Ա��ÿ���������Լ��ĸ���������̬���ݳ�Ա����������ֻ��һ�ݣ�
    ���Ա�������ж�������ʡ���ȫ�ֱ�����ȣ���̬���ݳ�Ա�����������ŵ㣺
    ��̬���ݳ�Աû�н�������ȫ��������ֻ��������������ڣ�
    ����ʵ����Ϣ���أ���̬��Ա������private��Ա����ȫ�ֱ������ܣ�

    static���ݳ�Ա��������ĳ���ض�������������ڹ��캯���г�ʼ����
    static���ݳ�Ա���ඨ��֮���ʼ�����ڶ���ʱҪʹ���������޶���̬��Ա��������Ҫ�ظ�����static�ؼ��֡�

    ��̬���ݳ�Ա�������࣬�Ǿ�̬���ݳ�Ա�����ڶ���
    ��̬���ݳ�Ա����ھ�̬�洢�����ɱ�������ж����������ڲ������ڶ���
    �Ǿ�̬���ݳ�Ա��������ڸ��������У������������ڶ��������Ĵ��������ڣ����������ٶ�������

    */

public:
    static Only* GetInstance() //����һ�����к��������Ի�ȡ���Ψһ��ʵ������������Ҫʱ������ʵ����
    {
        if (p == NULL)  //�ж��Ƿ��һ�ε���  
            p = new Only;
        return p;
    }
    static void show()
    {
        cout << m_b << endl;
    }
    static int setB(int b)
    {
        m_b = b;
        return m_b;
    }


    class Garbo
    {
    public:
        ~Garbo()
        {
            if (Only::p)
            {
                delete Only::p;
                cout << "delete only success " << endl;
            }
        }
    };
    static Garbo carbo;
};


int Only::m_b = 66;  //static��������ݳ�Ա�����������ʼ������Ϊ�����������һ���֣�����������ĳ������
Only* Only::p = NULL;
Only::Garbo Only::carbo;


int main_singleton()
{


    Only* a1 = Only::GetInstance();
    cout << a1 << endl;
    a1->show();
    Only* a2 = Only::GetInstance();
    cout << a2 << endl;
    a2->show();

    a1->setB(11);
    cout << a1 << endl;
    a1->show();

    return 0;
}



