#include "pch.h"
#include <windows.h>
namespace para
{
}
using namespace para;
using namespace std;
class CClass
{
public:
	int m_imp=0;

	CClass() = default;
	CClass(int i) :m_imp(i) {}
	//或者尽量只定义const成员函数，对象都能用
	int getValue() const
	{
		return m_imp;
	//	return const_cast<CClass&>(*this).getValue();
	}
	//int getValue()
	//{
	//	return m_imp;
	//}
};

//class Base
//{
//private:
//	virtual void base_fun0() { cout << "private" << endl; }
//public:
//	virtual void base_fun1() { cout << "public" << endl; }
//	int m_imp;
//	void set(int a) { m_imp = a; }
//	virtual void _cout(int i)
//	{
//		cout << "base" << endl;
//	}
//};
//
//class Inhe: public Base
//{
//private:
//	void base_fun1() override { cout << "private" << endl; }
//
//public:
//	void base_fun0() override { cout << "public" << endl; }
//	virtual void _cout(int i) override
//	{
//		if (i == 1)
//		{
//			cout << "inhe" << endl;
//		}
//		else
//			Base::_cout(i);
//	}
//
//};

enum class Test :int
{
	A0 = 0,
	A1,
	A2,
	A3,
	A4,
	A5
};
void test_size(int a[])
{
	//printf("%d\n", sizeof(a));
	//printf("%d\n", sizeof(a + 0));
	//printf("%d\n", sizeof(*a));
	//printf("%d\n", sizeof(a + 1));
	//printf("%d\n", sizeof(a[1]));
	//printf("%d\n", sizeof(&a));
	//printf("%d\n", sizeof(*&a));
	//printf("%d\n", sizeof(&a + 1));
	//printf("%d\n", sizeof(&a[0]));
	//printf("%d\n", sizeof(&a[0] + 1));
}
struct MyStruct
{
	int m_i;
	double m_d;
	MyStruct(int i, double d) :m_i(i), m_d(d)
	{
	}
	MyStruct() :m_i(1), m_d(2)
	{
	}
};

class C1
{
	//测试类的 sizeof，会有内存对齐问题，小成员变量自动补为大成员的size
	int m_i;
	//void* m_p;
	//double m_d;
	//char m_c;
	MyStruct m_s;
	double* ptr=new double(1);
public:
	C1(int i):m_i(i)
	{
	}
	~C1()
	{
		delete ptr; //易知，memcpy复制了指针，析构时将重复delete
	}

	static int sm_i; //static 不算类内size
};

string stringToUTF8(const string& str)
{
	int nwLen = ::MultiByteToWideChar(CP_ACP, 0, str.c_str(), -1, NULL, 0);
	wchar_t* pwBuf = new wchar_t[nwLen + 1];
	ZeroMemory(pwBuf, nwLen * 2 + 2);
	::MultiByteToWideChar(CP_ACP, 0, str.c_str(), str.length(), pwBuf, nwLen);
	int nLen = ::WideCharToMultiByte(CP_UTF8, 0, pwBuf, -1, NULL, NULL, NULL, NULL);
	char* pBuf = new char[nLen + 1];
	ZeroMemory(pBuf, nLen + 1);
	::WideCharToMultiByte(CP_UTF8, 0, pwBuf, nwLen, pBuf, nLen, NULL, NULL);
	std::string retStr(pBuf);
	delete[]pwBuf;
	delete[]pBuf;
	pwBuf = NULL;
	pBuf = NULL;
	return retStr;
}

string UTF8Tostring(const string &strTemp)
{
	char buf[1024 * 60];
	snprintf(buf, sizeof(buf), "%s", strTemp.c_str());
	TCHAR wscBuffer[1024 * 10] = { 0 };
	MultiByteToWideChar(CP_UTF8, 0, buf, (int)strlen(buf) + 1, wscBuffer, sizeof(wscBuffer) / sizeof(wchar_t));
	memset(buf, 0, 1024 * 9);
	WideCharToMultiByte(CP_ACP, 0, wscBuffer, -1, buf, 1024 * 9, NULL, NULL);
	return buf;
}

void testReadFile()
{
	vector<string>labellist;
	ifstream ifs;

	ifs.open("D:\\Alluser\\Program Files (x86)\\BIMBase KIT 2022\\PythonScript\\python-3.7.9-embed-amd64\\Lib\\site-packages\\test_script0\\rels_python_exec.py");   //若打印为乱码，将txt文件打开另存为，编码方式选为ANSI
	string str;

	while (getline(ifs, str))
	{
		//cout << str << ";";
		labellist.push_back(str);
	}
	string a = UTF8Tostring(labellist[7]);
	ifs.close();
}

enum class MYEC :unsigned int
{
	E0,
	E1,
	E2,
	E3,
	E4,
	E5
};
class A
{
public:
	A(int a) :m_a(a) {}
	operator bool() //提供一个本类型到bool的隐式转换，不允许使用参数
	{
		cout << "operator bool" << endl;
		return m_a;
	}
	bool operator==(const bool& r) const
	{
		return true;
	}
		bool operator==(const A& _r) const
	{
		return m_a == _r.m_a;
	}


private:
	int m_a;
};

#define _USE_MATH_DEFINES
struct _Sphere {};

static std::map<std::string, int> testMap =
{
	{"1",1},
	{"2",2},
	{"3",3},
};

static std::map<std::string, set<int>> testSet =
{
	{"1",{1,2,3}},
	{"2",{11,22,33}},
	{"3",{111,222,333}},
};

bool delMap(std::map<std::string, int>& themap, string Handle)
{
	for (auto it = themap.begin(); it != themap.end();) //delete by value
	{
		if (it->first == Handle)
		{
			/*it = */testMap.erase(it);
			return true;
		}

		else
			it++;
	}
	return false;
}

int main_class()
{
	//string anum = "1234567890000000";
	string Handle = "1";
	std::map<std::string, int>::iterator resE;
	auto name = std::hash<std::string>()(typeid(_Sphere).name());


	//==========================================================================

	//GeTransform matG(1, 1, 1); //first
	//BPTransfrom matB(2, 2, 2); //second
	//matB = matG; //类型双向互转  
	//matG = matB;

	//testMap.clear();
	//auto iterM = testMap.begin();
	//for (; iterM != testMap.end(); iterM++)
	//{
	//	if (iterM->second == 4)
	//		break;
	//}
	bool tD = delMap(testMap, Handle);
	//if (iterM == testMap.end())
	//	return 0;

	if (testSet.find("1") == testSet.end())
		return false;
	set<int>& handle = testSet["1"];
	handle.insert(4);
	auto res1 = handle.erase(2);
	auto res2 = handle.erase(10);

	do
	{
		//do something
		if (true)
			break;
		//something else, if success execute

	} while (false);
	
	A a(10);
	A b(10);
	if (a) 
		cout << "a" << endl;
	if (a == b)
		cout << "operate==" << endl;
	double pi = M_PI;

	int a1 = 1 << 1;
	int a2 = 1 << 2;
	int a3 = 1 << 3;
	int a4 = 1 << 4;

	MYEC e1 = MYEC::E1;
	unsigned int ie1 = (unsigned int)e1;

	testReadFile();
	//统计中英文个数
	//using utf-8 变长编码
	string exp = "123中文啊English+*（）";
	string utf = stringToUTF8(exp);
	string str = UTF8Tostring(utf);
	//+ ：匹配一次或多次；
	cout << regex_match("123", regex("\\d")) << endl;		//结果为0
	cout << regex_match("123", regex("\\d+")) << endl;		//结果为1
	{
		static std::set<wchar_t> s_set = { L'（', L'）', L'！', L'，', L'。', L'？', L'【', L'】', L'—',L'《', L'》' };
		wchar_t it = *s_set.begin();
		
		std::wstring wstr = L"1,a:我是谁（）！，。？【】-——《》";
		for (wstring::iterator it = wstr.begin(); it != wstr.end(); it++)
		{
			bool flag = 0;
			if ((0x2E80 < (*it) && (*it) < 0x9fff) || (0xff21 < (*it) && (*it) < 0xff5a) || s_set.find(*it) != s_set.end())
				flag = 1;

			bool flag1 = regex_match(it, it + 1, regex("[0x4e00-0x9fa5]+"));
			//cout << *it << " "<< flag << flag1 << endl;
		}
	}

	//unique_ptr<int> ptr;
	//{
	//	int* a= new int(1);
	//	ptr= unique_ptr<int>(a);
	//	delete a;
	//	a = nullptr;
	//}
	//int* pa = new int(2);

	size_t st = 0;
	if (st)
	{
		std::cout << "";
	}

		//mem系列函数
		//void* ::memset(void* s, int c, size_t n);
		//void* ::memcpy(void* dest, const void* src, size_t n);
		//void* ::memmove(void* dest, const void* src, size_t n);
		//int ::memcmp(const void* str1, const void* str2, size_t n);
	{
		string s = "helloworld";
		//memset(&s, 48, 5); //'\x1' 字符型常量  0x1 整型常量 中间替换的是ASCII
		cout << s << endl;
	}
	{
		char s[] = "helloworld";
		char ori[] = "china";
		memcpy(s, ori, 5);

		//size字节数 char==256==1byte
		//int sz = ori.size();
		//int sz0 = sizeof(char);
		//int sz2 = sizeof(string);
		//int sz1 = sizeof(ori);
		//int sz3 = sizeof(C1);
		int sz4 = sizeof(ori) / sizeof(char);
		
		//using std::string
		string s1 = "helloworld";
		string ori1 = "china";
		//string may crash
		//memcpy(&s1, &ori1, ori1.size()); //target origin sizeof(ori)

		C1 c1(1);
		C1 c2(2);
		//memcpy(&c1, &c2, sizeof(C1)); //crash

		cout << s1 << endl;
		const char* a = "abcde";
		const char* b = "12345";

		vector<int> v1 = { 1,2,3,4 };
		vector<int> v2 = { 21,22,23,24 };

		//auto bl = v1.clear();
		//memcpy(&v1, &v2, v2.size());
	}

	{
		string s1 = "helloworld";
		string ori1 = "china";
		char s[] = "helloworld";
		char ori[] = "china";
		//void* memmove(void* dest, const void* src, size_t n);
		//memmove(&s1, &ori1, ori1.size());
		//memmove(&s1, &ori1, 10);
		//memmove(&ori1, &s1, 5);
	}

	{
		//int memcmp(const void* str1, const void* str2, size_t n);
		string s1 = "helloworld";
		string ori1 = "helloChina";
		char s[] = "helloworld";
		char ori[] = "hellochina";

		int res1 = memcmp(&s1, &ori1, 2);
		int res = memcmp(&s, &ori, 6);
	/*
	如果返回值 < 0，则表示 str1 小于 str2。
	如果返回值 > 0，则表示 str2 小于 str1。
	如果返回值 = 0，则表示 str1 等于 str2。*/
	}

	/*
	内存对齐
	第一个数据成员放在offset为0的地方，以后每个数据成员的起始位置要从自身大小的整数倍开始存储
	从其内部"最宽基本类型成员”的整数倍地址开始存储
	结构体的总大小为结构体的有效对齐值的整数倍，结构体的有效对齐值的确定
	都没有的空类，实际并不是空的，因为有默认的函数，大小是 1
	成员函数、静态成员函数、静态成员变量都是不占用类的内存的，这是因为这些东西都是类的，而不是每个对象分别存储。
	子类继承空类后，子类如果有自己的数据成员，而空基类的1个字节并不会加到子类中去。
	内存对齐和成员定义的顺序有关
	C++ 的类中如果有虚函数，类内就会有一个虚函数表的指针 _vptr，指向自己的虚函数表，指针大小都是 8
	效率原因：经过内存对齐之后，CPU的内存访问速度大大提升
	平台原因（移植原因）：不是所有的硬件平台都能访问任意地址上的任意数据
	比较两个结构体不能使用memcmp(void*, void*)
	*/

	//std::any g1 = std::any(int(1));
	//std::any g2 = std::any(double(1)); //any仅支持debug查看double变量
	//std::any g3 = std::any(string("1"));

	auto l1 = sizeof(int);
	auto l2 = sizeof(long);
	auto l3 = sizeof(long long);
	auto l4 = sizeof(double);
	auto l5 = sizeof(long double);
	unsigned long  anum = 0x0010;
	//unsigned long  anum4 = 0x1234;
	unsigned long  anum2 = 0x0123; //16进制，<<4位
	unsigned long  anum3 = 0x01230000; //F=16
	unsigned long  anum4 = anum2 * 16 * 16 * 16 * 16;
	unsigned long  anum5 = anum2 <<16;
	auto num2=anum << 4;
	anum += anum << 4;
	anum += anum << 4;

	unsigned long long res = 0;//8byte
	unsigned char iter=0x1; //1byte
	res = res + iter;
	res = res * 16 + iter;
	res = res * 16 + iter;
	res = res * 16 + iter;

	map<type_index, int> tiMap;
	tiMap[typeid(int)] = 1;
	tiMap[typeid(string)] = 2;
 

	//test MD5
	//string md0 = para::getMD5("helloworld");
	////MD5_32B md3 = para::getMD5_ULL("helloworld");
	//MD5 md("helloworld");
	//string mm2 = md.toStr(); //"fc5e038d 38a57032 085441e7 fe7010b0"// 32 byte
	//hex16 to dec	//unsigned long  hexA = 0xfe7010b0;
	//fc5e038d = 4234019725
	//38a57032 = 950366258
	//085441e7 = 139739623
	//fe7010b0 = 4268757168
	//string md2 = para::getMD5("ohmygod"); //md2 = "5802ae89 9e406453 f3ad93e3 7f9d26a8"
	//md2 = "1476570761 2655020115 4088239075 2141005480"
	//auto sz = sizeof(mm2);
	////MD5_32B it4 = MD5_32B();
	//string md1 = getMD5("你好啊");
	//string message = "hello";
	//cout << "md5(\"" << message << "\") = "<< MD5(message).toStr();

	unsigned long long a0 = 0xfc5e038d;
	auto ui = unsigned int(-1); //4294967295
	MyStruct sa1(1,2.0);// { 1, 2.0 };
	MyStruct sa2= MyStruct{ 1, 2.0 };
	vector<Test> myvec;
	int t = (int)Test::A5;

	for (int i=3;i<64;i++)
	{
		//cout << "fib(" << i << "):" << fib2(i) << endl;
	}
	CClass c1;
	const CClass c2;
	int r1 = c1.getValue();
	int r2 = c2.getValue();


	//int a[] = { 1,2,3,4 };

	return 0;
}

//test union
struct MyVector3d
{
	double x;
	double y;
	double z;
};

union MyUnion {
	int i;
	float f;
	double d;
	char* str;// [20] ;
	MyVector3d vec3;
	//vector<int> vec; //不可使用
};

static void _test1()
{
	int sz1 = sizeof(MyUnion); //最大成员；
	MyUnion u;
	u.i = 10; // 现在u存储的是int类型的数据  
	sz1 = sizeof(u);
	u.f = 220.5; 
	sz1 = sizeof(u);
	u.d = 123.1;
	sz1 = sizeof(u);
	string str = "hello world";
	u.str = const_cast<char*>(str.c_str());
	sz1 = sizeof(u);

	return;
}

class Base {
public:
	virtual ~Base() {} // 为了确保多态性，Base类应该有虚析构函数
};

class DerivedA : public Base {
public:
	void info() { std::cout << "I am DerivedA" << std::endl; }
};

class DerivedB : public Base {
public:
	void info() { std::cout << "I am DerivedB" << std::endl; }
};

void identify(Base* basePtr) {
	// 使用 dynamic_cast 进行类型判断
	if (DerivedA* derivedAPtr = dynamic_cast<DerivedA*>(basePtr)) {
		derivedAPtr->info();
	}
	else if (DerivedB* derivedBPtr = dynamic_cast<DerivedB*>(basePtr)) {
		derivedBPtr->info();
	}
	else {
		std::cout << "Unknown type" << std::endl;
	}
}

static void _test2()
{

	Base* ptr1 = new DerivedA();
	Base* ptr2 = new DerivedB();

	identify(ptr1); // 应输出 "I am DerivedA"
	identify(ptr2); // 应输出 "I am DerivedB"

	string name1 = typeid(*ptr1).name();
	string name2 = typeid(*ptr2).name();

	return;
}

static int enrol = []()->int
	{
		//_test1();
		//_test2();
		cout << clash::get_filepath_filename(__FILE__) << " finished.\n" << endl;
		return 0;
	}();
