#include "pch.h"
using namespace std;

int MyCompare(const void* e1, const void* e2)
{ //用于qsort的比较函数
    return *((int*)e1) - *((int*)e2);
}


int main_file()
{

    std::ifstream t("file.txt");
    std::stringstream buffer;
    //buffer << t.rdbuf();
    std::string contents(buffer.str());
    const int MAX_NUM = 1000;
    int a[MAX_NUM]; //存放文件中读入的整数

    const char cmd[] = "C:/Users/Aking/AppData/Local/Programs/Python/Python38/python.exe \"c:/Users/Aking/Documents/WXWork/1688856575247594/Cache/File/2022-06/Release (1)/PythonScript/python-3.7.9-embed-amd64/Lib/site-packages/test_script0/Cube参数化.py\"";
    string cmds = "C:/Users/Aking/AppData/Local/Programs/Python/Python38/python.exe \"c:/Users/Aking/Documents/WXWork/1688856575247594/Cache/File/2022-06/Release (1)/PythonScript/python-3.7.9-embed-amd64/Lib/site-packages/test_script0/Cube参数化.py\"";
//    string cmds = "python";
    system(cmds.c_str());
    //system("taskkill /f /im cmd.exe");

	string mypath = "C:\\Users\\Aking\\Documents\\WXWork\\1688856575247594\\Cache\\File\\2022-06\\Release (1)\\PythonScript\\python-3.7.9-embed-amd64\\Lib\\site-packages\\test_script0\\Cube参数化.py";

	
    //一次全部读取
    ifstream inFile(mypath, ios::in);
    istreambuf_iterator<char> beg(inFile), end;
    string strdata(beg, end);
    inFile.close();

    //逐段读取
    ifstream srcFile(mypath, ios::in); //以文本模式打开in.txt备读
    string text, script;
    while (srcFile >> script)
    {
        text += script;
        text += "\n";
    }
    srcFile.close();



    //写入
	//ofstream destFile("out.txt", ios::out); //以文本模式打开out.txt备写
 //   destFile << strdata;
 //   destFile.close();
	//if (!destFile) 
	//{
	//	srcFile.close(); //程序结束前不能忘记关闭以前打开过的文件
	//	cout << "error opening destination file." << endl;
	//	return 0;
	//}


    //int total = 0;//读入的整数个数
    //ifstream srcFile("in.txt", ios::in); //以文本模式打开in.txt备读
    //if (!srcFile) { //打开失败
    //    cout << "error opening source file." << endl;
    //    return 0;
    //}
    //ofstream destFile("out.txt", ios::out); //以文本模式打开out.txt备写
    //if (!destFile) {
    //    srcFile.close(); //程序结束前不能忘记关闭以前打开过的文件
    //    cout << "error opening destination file." << endl;
    //    return 0;
    //}
    //int x;
    //while (srcFile >> x) //可以像用cin那样用ifstream对象
    //    a[total++] = x;
    //qsort(a, total, sizeof(int), MyCompare); //排序
    //for (int i = 0; i < total; ++i)
    //    destFile << a[i] << " "; //可以像用cout那样用ofstream对象
    //destFile.close();
    //srcFile.close();
    return 0;
}


static void _test0()
{

    string mypath = "C:\\Users\\Aking\\Documents\\WXWork\\PythonScript\\python-3.7.9-embed-amd64\\Lib\\site-packages\\test_script0\\Cube参数化.py";
    string res;
    int count = 0;
	for (auto iter = mypath.rbegin(); iter != mypath.rend(); iter++)
    {
        if (*iter == '\\')
            count++;
		if (count < 2)
            res += *iter;
    }
    std::reverse(res.begin(), res.end());
	int id1 = mypath.find_last_of('\\');
	string sub1 = mypath.substr(id1, mypath.size());

    return;
}


static int enrol = []()->int
{
    //_test0();
    return 0;
}();
