#include "pch.h"
using namespace std;


/*
单元测试
int double longlong
class struct vector map
构造和析构
继承类

*/


void test1()
{
	const int& a = 0; //非常量引用的初始值必须为左值
	//左值是指表达式结束后依然存在的持久对象，右值是指表达式结束时就不再存在的临时对象。
	//一个区分左值与右值的便捷方法是：看能不能对表达式取地址（能不能再次赋值），如果能，则为左值，否则为右值。
	int b = 0;
	++b = 1;
	//b++ = 1; //右值
	Vec3 v;
}
Gene getAg(bool)
{
	Gene a(11);
	return a; //不能简化为Gene(11)
}


int maingnrc()
{
	std::any anya(1);
	std::any anyd(1.0);
	Vec3 vec=Vec3();
	std::any anyv(&vec);
	std::any anyb("abc");
	type_index mytype=anyb.type();
	if (anya.type() == typeid(int))
	{
		int a = std::any_cast<int>(anya);
		cout << a << endl;
	}

	if (anyv.type() == typeid(Vec3*))
	{
		Vec3* a = std::any_cast<Vec3*>(anyv);
		cout << a << endl;
	}


	Gene::enrol<bool>();
	Gene::enrol<int>();
	Gene::enrol<float>();
	Gene::enrol<double>();
	Gene::enrol<char>();
	Gene::enrol<string>();
	Gene::enrol<wchar_t>();

	//混合测试
	// value refTrue refFalse
	//string* str = new string("12mm");//禁止gnrc直接装载指针类型
	//int* ia0 = new int(1233);
	//double* ia1 = new double(1234);
	string str = string("12mm");//禁止gnrc直接装载指针类型
	string str1 = string("12mm");
	int ia0 = int(1233);
	double ia1 = double(1234);

	//for(int i=0;i<1e7;i++)	{
	Gene gnA0(int(120));
	Gene gnA1(double(131));
	Gene gnA2(string("string"));
	//}


	//for (int i = 0; i < 1e7; i++) {
	//	Gene gnB01(typeid(int), &ia0, false);
	//	Gene gnB02(typeid(string), &str, false);
	//	Gene gnC11(typeid(double), &ia1, true);
	//	Gene gnC12(typeid(string), &str, true);
	//}

	//bool bl1 = (gnB02 < gnC12);
	//bool bl2 = (gnB02 == gnC12);

	//is as _id _imp
	//bool bl1 = gnA0.is<int>();
	//bool bl2 = gnA0.is<int*>();
	//int res =  gnA0.as<int>();
	//auto idd = gnA0._id();
	//auto imp = gnA0._imp();

	//bool bl1 = gnB01.is<int>();
	//bool bl2 = gnB01.is<int*>();
	//int res  = gnB01.as<int>();
	//auto idd = gnB01._id();
	//auto imp = gnB01._imp();

	//bool bl1 = gnC12.is<string>();
	//bool bl2 = gnC12.is<string*>();
	//string res=gnC12.as<string>();
	//auto idd = gnC12._id();
	//auto imp = gnC12._imp(); //强转无效

	//生命周期测试
	Gene ltB01;
	Gene ltB02;
	Gene ltC11;
	Gene ltC12;
	{
		int v = 12;
		int* p1 = new int(v);
		int* p2 = p1;
		auto a = *(int*)p1;
		delete p1;
		delete p2;

		//ltB01(typeid(int), &ia0, false);
		//ltB02(typeid(string), &str, false);
		//ltC11(typeid(double), &ia1, true);
		//ltC12(typeid(string), &str1, true);
	}
	auto imp1 = ltB01;
	auto imp2 = ltB01;
	auto imp3 = ltC11;
	auto imp4 = ltC12;

	int* p1 = new int(1);
	//std::string* p1 = new std::string("3");
	//std::string* p2 = new std::string("3");

	{
		//Gene g1(typeid(string), p1, true);//false
		Gene g1(typeid(int), p1, false); 
		Gene g2 = g1;
		type_index a = typeid(int);
		Gene g3(a);
		int b = g3.is<int>();
	}

	//std::string p1 = std::string("3");
	//std::string p2 = std::string("3");

	{
		//Gene g1(typeid(string), p1, true); 
		//Gene g2(typeid(string), &p2, false);//自己用
	}
	cout << "" << endl;
	//delete p1;
	//delete p2;

	//auto aft = p1;
	//auto aft2 = p2;
	//int aa1 = 1;

	//测试相互 operator= copy construct	
	//gnA0 = gnA1; //value
	//gnA0 = gnB01;
	//gnA0 = gnB02;
	//gnA0 = gnC11;
	//gnA0 = gnC12;

	
	//gnB01 = gnA0; //ref false
	//gnB01 = gnB01;
	//gnB01 = gnB02;
	//gnB01 = gnC11;
	//gnB01 = gnC12;

	//gnC11 = gnA0; //ref true
	//gnC11 = gnB01;
	//gnC11 = gnB02;
	//gnC11 = gnC11;
	//gnC11 = gnC12;
	
	//-----------------------------------------------
	//测试Gnrc装ref引用
	//int* p = 0;// nullptr;
	//int* p1 = 0;// nullptr;
	//cout << *p << endl;
	//禁止gnrc装载指针类型
	//string* str = new string("1233");
	//int* ia0 = new int(1234);
	//int* ia1 = new int(1234);
	{
		//Gene gp(p);
		//Gene gs(str);
		//gp = 2.0; //Gnrc定义后可以随意改值的类型，走模板拷贝构造
		//gp = Vec3();
		//p1 = &aa1;
		/*Gnrc gc(typeid(str), str, false);
		Gnrc gci(typeid(ia), ia, false);*/
		//Gnrc gc(typeid(str), str, false);

	}
	//p = &aa1;
	//str = "00";
	//cout << *str << endl;
	//cout << *ia << endl;
	cout << "here" << endl;

	Gene num0; //没有合适的默认构造函数可用
	Gene num1(num0);
	Gene num2=num0;
	Gene num3;
	num3 = num0;
	num3 = num3;

	Gene nul1;
	Gene nul2;
	Gene a1(1);
	Gene b1(1);
	Gene a2(2);
	Gene b2(2.0);
	Gene c2(char('c'));
	type_index typ=a1.get_id();
	void* pt = (void*)a1.m_imp;


	/*int* pn1 = nullptr;
	int* pn2 = nullptr;
	if (pn1 == pn2)
	{
		std::cout << "==";
	}*/
	//小于
	if (nul1 < a1)
	{
		std::cout << "<";
	}
	if (a1 < nul1)
	{
		std::cout << "<";
	}
	if (a1 < a2)
	{
		std::cout << "<";
	}
	if (a2 < b2)
	{
		std::cout << "<";
	}
	if (a2 < c2)
	{
		std::cout << "<";
	}
	if (c2 < a2)
	{
		std::cout << "<";
	}
	if (Gene(int(1)) < Gene(int(2)))
	{
		std::cout << "<";
	}
	if (Gene(int(1)) < Gene(double(2)))
	{
		std::cout << "<";
	}
	if (Gene(double(2)) < Gene(int(1)))
	{
		std::cout << "<";
	}
	//小于等于
	if (Gene(int(1)) <= Gene())
	{
		std::cout << "<=";
	}
	if (Gene() <= Gene(int(1)))
	{
		std::cout << "<=";
	}
	if (Gene() <= Gene())
	{
		std::cout << "<=";
	}
	//大于
	if (Gene(int(1)) > Gene(2))
	{
		std::cout << "<=";
	}
	if (Gene(int(3)) > Gene(2))
	{
		std::cout << "<=";
	}
	if (Gene(int(3)) > Gene(3))
	{
		std::cout << "<=";
	}
	if (Gene(int(1)) >= Gene(2))
	{
		std::cout << "<=";
	}
	if (Gene(int(3)) >= Gene(2))
	{
		std::cout << "<=";
	}
	if (Gene(int(3)) >= Gene(3))
	{
		std::cout << "<=";
	}

	if (Gene(int(1)) <= Gene(int(2)))
	{
		std::cout << "<=";
	}
	if (Gene(int(1)) <= Gene(int(1)))
	{
		std::cout << "<=";
	}
	if (Gene(int(1)) <= Gene(int(0)))
	{
		std::cout << "<=";
	}


	//等于
	if (nul1 == nul1)
	{
		std::cout << "==";
	}
	if (nul1 == nul2) //两个默认构造nullptr是否相等
	{
		std::cout << "==";
	}
	if (nul1 == a1)
	{
		std::cout << "==";
	}

	if (a1 == b1)
	{
		std::cout << "==";
	}
	if (a1 == a2)
	{
		std::cout << "==";
	}
	if (a2 == b2) //Temp<double>转Temp<int>会失败，返回false
	{
		std::cout << "==";
	}
	//不等于
	if (nul1 != nul1)
	{
		std::cout << "==";
	}
	if (nul1 != nul2)  
	{
		std::cout << "==";
	}
	if (nul1 != a1)
	{
		std::cout << "==";
	}

	if (a1 != b1)
	{
		std::cout << "==";
	}
	if (a1 != a2)
	{
		std::cout << "==";
	}
	if (a2 != b2) 
	{
		std::cout << "==";
	}
	GeneDict mymap;
	mymap[Gene(1)] = 1;
	mymap[Gene(2)] = 2;
	mymap[Gene(3.0)] = 3;
	//mymap[Gene("cha")] = 4; //不允许存 char*
	mymap[Gene(5)] = 5;
	

	//if (1 == 1)
	if (a1 == b1)
		cout << "yes" << endl;
	int yes=1;
	//测试基本函数
	Gene numt0(0);
	//Gene gnrc_c0(num0); //拷贝构造 
	//Gene gnrc_c1 = num0; //拷贝构造 
	//num0 = gnrc_c1; //拷贝赋值

	//Gene geta = getA(); //移动构造
	//Gene gnrc_m = move(geta); //移动构造 
	//Gene gnrc_m2(2);
	//Gene gnrc_m1(std::move(gnrc_m2));//移动构造
	//Gene gnrc_m3(3);
	//Gene &&gnrc_m4= move(gnrc_m3); //右值赋值
	//Gene gnrc_m5(gnrc_m4);//直接赋值无效，还是拷贝构造


	//gnrc_m2 = getA();//移动赋值
	//gnrc_m2 = move(gnrc_m); //移动赋值

	GeneList list;
	vector<Vec3> vecs;
	vector<int> vecint;
	//Vec3* v = new Vec3[10];
	//Gene num0 = Gene(0);
	//Gene gnrcn(num0);



	auto siz = sizeof(list); //vector 统一32byte
	auto siz0 = sizeof(int); //4
	auto siz1 = sizeof(vecs); //32
	auto siz2 = sizeof(vecint); //32
	auto siz3 = sizeof(Gene(0)); //8
	auto siz4 = sizeof(map<int,int>); //24
	auto siz5 = sizeof(Gene); //8
	auto siz6 = sizeof(Vec3); //32

	for (int i = 0; i < 10; i++)
	{
		Vec3 v(i, i, i);
		//vecs.push_back(v);
		vecs.push_back(move(v)); //右值引用成功
	}

	//if (1) // true
	for (int i = 0; i < 1e1; i++) //1e1
	{
		//size_t a=list.capacity();
		Gene num0(0); // num0(nullptr);// = Gene(); //int(1)
		Gene num1(1); // num1(nullptr);// = Gene();
		Gene num2(2); // num2(nullptr);// = Gene();
		//Gene gnrc0(num0);
		//Gene gnrc1(num1);
		//Gene gnrc2(num2);
		list.push_back(num0);
		list.push_back(num1);
		list.push_back(num2);

		vecint.push_back(0);
		vecint.push_back(1);
		vecint.push_back(2);

		//a = list.capacity();
		/*list.push_back(std::move(num0));
		list.push_back(std::move(num1));
		list.push_back(std::move(num2));*/

		//Vec3 v0 = Vec3(0, 0, 0);
		//Vec3 v1 = Vec3(1, 1, 1);
		//Vec3 v2 = Vec3(2, 2, 2);
		//vecs.push_back(v0);
		//vecs.push_back(v1);
		//vecs.push_back(v2);
	}


	//list.push_back(num);
	////list.push_back(Gene(1));
	//Gene num1 = Gene(1);
	//list.push_back(num1);
	//return 0;
	//vecs.clear();


	//for (int i = 0; i < 1e4; i++)
	//{
	//	Gene gnrcint(int(0)); //测试独立Gene对象
	//	Gene gnrcint1(int(1));

	//	Gene num = Gene(0);
	//	//list.push_back(num);  //测试vector<Gene>对象
	//	//list.push_back(gnrcint1);
	//	//list.push_back(Gene(0));
	//	//list.push_back(Gene(1));

	//	//GeneList aint = { int(1),double(2),string("3"), Vec3() };
	//	GeneList aint = { int(0), int(1) };// , double(2), string("3"), Vec3()
	//}


	//-------------------------------------------------
	//类型支持验证
	//map<string,int> mp={ {"A",1} }; 验证支持map
	////mp["A"] = 1;
	//Gene gmap(mp);
	//auto gmapas = gmap.as<map<string, int>>();

	Gene gnrcint(int(4884));
	if (gnrcint.is<double>()) //可判断
	{
		std::cout << "" << endl;
	}

	if (gnrcint.is<int>())
	{
		auto res2 = gnrcint.as<int>();
		int a=1;
	}
	Gene gnrcdouble(double(1.23));
	//auto res3 = gnrcdouble.is<double>();
	if (gnrcdouble.is<double>())
	{
		auto res3 = gnrcdouble.as<double>();
		cout << res3 << endl;
	}
	Gene gnrcstring(std::string("sss"));
	if (gnrcstring.is<string>())
	{
		auto res4 = gnrcstring.as<string>();
		int a = 1;
	}


	//char*
	//char chas[] = "chars";
	//char* chas2 = chas;
	////const char* chas = "chars";
	//cout << chas << endl;
	//Gene gnrcchar(chas);
	//if (gnrcchar.is<char*>())
	//{
	//	auto res4 = gnrcchar.as<char*>();
	//	cout << res4 << endl;
	//	int a = 1;
	//}

	int n = 1;
	int& nref = n;

	Gene gnrcref(nref);


	Vec3 a = Vec3(1, 2, 3);
	Vec3 b;
	Gene gnrc(a);
	if (gnrc.is<Vec3>())
	{
		b = gnrc.as<Vec3>();
		int a = 0;
	}
	bool jud = (a == b); //重载 ==
	auto dd = (double)a;
	auto bb = bool(a);
	auto ddd = double(a);

	long long l = 112;
	Gene gnrc2(l);
	if (gnrc2.is<long long>())
	{
		auto b = gnrc2.as<long long>();
		string name = typeid(b).name();
		type_index namei = typeid(b);
		int a = 0;

	}


	//大于 小于 等于

	cout << "test gnrc" << endl;

	return 0;
}