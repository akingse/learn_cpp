#include "pch.h"
using namespace std;
using namespace clash;

static int main_file()
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
    int x, y;
    cin >> x >> y;
    freopen("test01.txt", "w", stdout);
    if (y == 0)
        cerr << "error" << endl; //显示调试信息
    else
        cout << float(x) / y; //终端没有显示，重定向，写入了文件
}

static void _test1()
{
    double m, n;
    cin >> m >> n;
    try
    {
        cout << "before dividing." << endl;
        if (n == 0)
            throw - 1; //抛出int类型异常，程序停止运行
        else
            cout << m / n << endl;
        cout << "after dividing." << endl;
    }

    //异常处理，从上到下，依次匹配
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

static void _test2()
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

std::vector<std::string> getListFiles(const std::string& directory) 
{
    std::vector<std::string> filenameRes;
    std::string searchPath = directory + "\\*";
    WIN32_FIND_DATA findFileData;
    HANDLE hFind = FindFirstFileW(string2wstring(searchPath.c_str()).c_str(), &findFileData);
    if (hFind == INVALID_HANDLE_VALUE) 
    {
        std::cerr << "FindFirstFile failed (" << GetLastError() << ").\n";
        return filenameRes;
    }
    do 
    {
        const std::string fileOrDir = wstring2string(findFileData.cFileName);
        // Skip current directory (.) and parent directory (..)
        if (fileOrDir != "." && fileOrDir != "..") 
        {
            // Construct the full path
            std::string fullPath = directory + "\\" + fileOrDir;
            if (findFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) 
            {
                // It's a directory, print it and recurse into it
                //std::cout << "Directory: " << fullPath << std::endl;
                std::vector<std::string> temp = getListFiles(fullPath); // Recursive call
                filenameRes.insert(filenameRes.end(), temp.begin(), temp.end());
            }
            else 
            {
                // It's a file, print it
                //std::cout << "File: " << fullPath << std::endl;
                filenameRes.push_back(fullPath);
            }
        }
    } 
    while (FindNextFileW(hFind, &findFileData) != 0);
    FindClose(hFind);
    return filenameRes;
}

#include <io.h>//corecrt_io
#include <direct.h>
int _test3() 
{
    //std::vector<std::string> filenameRes;
    //std::vector<std::string> temp;
    //filenameRes.insert(filenameRes.end(), temp.begin(), temp.end());

    std::string folderPath = "D:\\Download\\Quark";
    std::vector<std::string> filenameRes = getListFiles(folderPath);

    //returns 0 if the file has the given mode. 
    // The function returns –1 if the named file does not exist or is not accessible 
    int res = _access(folderPath.c_str(), 0);

    //判断目录是否存在

    fstream _file;
    _file.open(folderPath, ios::in);
    bool suc = _file.is_open();

    if (_access(folderPath.c_str(),0)==-1)
    {
        _mkdir(folderPath.c_str());
    }

    return 0;
}

static int enrol = []()->int
{
    //_test0();
    _test3();
    return 0;
}();
