#include "pch.h"
using namespace std;

static int MyCompare(const void* e1, const void* e2)
{ //����qsort�ıȽϺ���
    return *((int*)e1) - *((int*)e2);
}


static int main_file()
{

    std::ifstream t("file.txt");
    std::stringstream buffer;
    //buffer << t.rdbuf();
    std::string contents(buffer.str());
    const int MAX_NUM = 1000;
    int a[MAX_NUM]; //����ļ��ж��������

    const char cmd[] = "C:/Users/Aking/AppData/Local/Programs/Python/Python38/python.exe \"c:/Users/Aking/Documents/WXWork/1688856575247594/Cache/File/2022-06/Release (1)/PythonScript/python-3.7.9-embed-amd64/Lib/site-packages/test_script0/Cube������.py\"";
    string cmds = "C:/Users/Aking/AppData/Local/Programs/Python/Python38/python.exe \"c:/Users/Aking/Documents/WXWork/1688856575247594/Cache/File/2022-06/Release (1)/PythonScript/python-3.7.9-embed-amd64/Lib/site-packages/test_script0/Cube������.py\"";
//    string cmds = "python";
    system(cmds.c_str());
    //system("taskkill /f /im cmd.exe");

	string mypath = "C:\\Users\\Aking\\Documents\\WXWork\\1688856575247594\\Cache\\File\\2022-06\\Release (1)\\PythonScript\\python-3.7.9-embed-amd64\\Lib\\site-packages\\test_script0\\Cube������.py";

	
    //һ��ȫ����ȡ
    ifstream inFile(mypath, ios::in);
    istreambuf_iterator<char> beg(inFile), end;
    string strdata(beg, end);
    inFile.close();

    //��ζ�ȡ
    ifstream srcFile(mypath, ios::in); //���ı�ģʽ��in.txt����
    string text, script;
    while (srcFile >> script)
    {
        text += script;
        text += "\n";
    }
    srcFile.close();



    //д��
	//ofstream destFile("out.txt", ios::out); //���ı�ģʽ��out.txt��д
 //   destFile << strdata;
 //   destFile.close();
	//if (!destFile) 
	//{
	//	srcFile.close(); //�������ǰ�������ǹر���ǰ�򿪹����ļ�
	//	cout << "error opening destination file." << endl;
	//	return 0;
	//}


    //int total = 0;//�������������
    //ifstream srcFile("in.txt", ios::in); //���ı�ģʽ��in.txt����
    //if (!srcFile) { //��ʧ��
    //    cout << "error opening source file." << endl;
    //    return 0;
    //}
    //ofstream destFile("out.txt", ios::out); //���ı�ģʽ��out.txt��д
    //if (!destFile) {
    //    srcFile.close(); //�������ǰ�������ǹر���ǰ�򿪹����ļ�
    //    cout << "error opening destination file." << endl;
    //    return 0;
    //}
    //int x;
    //while (srcFile >> x) //��������cin������ifstream����
    //    a[total++] = x;
    //qsort(a, total, sizeof(int), MyCompare); //����
    //for (int i = 0; i < total; ++i)
    //    destFile << a[i] << " "; //��������cout������ofstream����
    //destFile.close();
    //srcFile.close();
    return 0;
}

static void _test0()
{
    int x, y;
    cin >> x >> y;
    freopen("test01.txt", "w", stdout);
    if (y == 0)
        cerr << "error" << endl; //��ʾ������Ϣ
    else
        cout << float(x) / y; //�ն�û����ʾ���ض���д�����ļ�
}

static void _test1()
{
    double m, n;
    cin >> m >> n;
    try
    {
        cout << "before dividing." << endl;
        if (n == 0)
            throw - 1; //�׳�int�����쳣������ֹͣ����
        else
            cout << m / n << endl;
        cout << "after dividing." << endl;
    }

    //�쳣�������ϵ��£�����ƥ��
    catch (int e) 
    {
        cout << "catch(int) " << e << endl;
    }
    catch (double e)
    {
        cout << "catch(double) " << e << endl;
    }
    catch (...)//
    {
        cout << "catch(default) " << endl;
    }
    cout << "finished" << endl;
    //return 0;

}

static int enrol = []()->int
{
    //_test0();
    _test1();


    return 0;
}();

