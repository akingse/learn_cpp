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

static void _test2()
{

    string mypath = "C:\\Users\\Aking\\Documents\\WXWork\\PythonScript\\python-3.7.9-embed-amd64\\Lib\\site-packages\\test_script0\\Cube������.py";
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
