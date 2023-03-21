#include "pch.h"
using namespace std; //https://www.codenong.com/cs106026685/

//// dynamic_cast_1.cpp
//// compile with: /c
//class B 
//{ };
//class C : public B 
//{ };
//class D : public C 
//{ };
//
//void f(D* pd) 
//{
//	C* pc = dynamic_cast<C*>(pd); // ok: C is a direct base class
//	// pc points to C subobject of pd
//	B* pb = dynamic_cast<B*>(pd); // ok: B is an indirect base class
//	// pb points to B subobject of pd
//
//	return;
//}
//
//// dynamic_cast_2.cpp
//// compile with: /c /GR
//class A { virtual void f() {}; };
//class B { virtual void f() {}; };
//
//void f() 
//{
//	A* pa = new A;
//	B* pb = new B;
//	void* pv = dynamic_cast<void*>(pa);
//	// pv now points to an object of type A
//
//	pv = dynamic_cast<void*>(pb);
//	// pv now points to an object of type B
//	return;
//}


// dynamic_cast_3.cpp
// compile with: /c /GR
class B { virtual void f() {}; };
class D : public B { virtual void f() {}; };

void f() {
	B* pb = new D; // unclear but ok
	B* pb2 = new B;

	D* pd = dynamic_cast<D*>(pb); // ok: pb actually points to a D
	D* pd2 = dynamic_cast<D*>(pb2); // pb2 points to a B not a D
}

//--------------------------------------------------------
class CLA
{
public:
	CLA() {};
	CLA(int a) :m_a(a) {};
	virtual ~CLA() {}
	//virtual int getNum() { return 0; }
	int m_a = 1;
};

class CLB :public CLA
{
public:
	virtual ~CLB() {};
	int m_b = 2;
};

class CLC : public CLB
{
public:
	CLC() {};
	CLC(int c) :m_c(c) {};
	int getNum() { return m_a; }
	~CLC() {}
	int m_c = 3;
};

class CLD : public CLA
{
public:
	int getNum() { return m_a; }
	~CLD() override {}
};



bool func_input(int& in)
{
	int i=1;
	return in = i;
}

//bool func_input1()
//{
//	return func_input(1);
//}

int main_cast()
{
	int j;
	bool b0=func_input(j);

	const int* a = new int(0);
	//int* b = a; //error
	int* b = const_cast<int*>(a); //ok
	//int* b = new int(0);
	//const int* d = b;
	

	string expre = "( '无缝钢管,Φ' + STR ( ATTRIB PARA[2 ] ) + 'x' + STR ( ATTRIB SDTH[1 ] OF CMPR OF SPRE  ) + ',GB/T 5310-2017,P4.2T455A11SO,' )";
	regex reQuo(string("'.+?'"));
	vector<std::string> quoList;
	vector<std::string> strList;
	for (sregex_iterator iter(expre.begin(), expre.end(), reQuo); iter != sregex_iterator(); iter++)
	{
		string finder = iter->str();
		string finderSub = regex_replace(finder, regex(string("'")), "");
		quoList.push_back(finderSub);
	}
	regex reSTR(string("STR\\s\\(.+?\\)"));
	for (sregex_iterator iter(expre.begin(), expre.end(), reSTR); iter != sregex_iterator(); iter++)
	{
		string finder = iter->str();
		string finderSub = regex_replace(finder, regex(string("STR\\s")), "");
		finderSub = regex_replace(finderSub, regex(string("\\(\\s")), "");
		finderSub = regex_replace(finderSub, regex(string("\\s\\)")), "");
		strList.push_back(finderSub);
	}





	//mymap.erase(MyEnum(3));


	//D* d = new D;
	//f(d);
	f();

	//constexpr string name = "NAME";

	//up
	CLD* d1 = new CLD();
	int d2 = d1->getNum();
	CLA* dp1 = dynamic_cast<CLA*>(d1);



	CLC c;
	cout << typeid(CLA).name() << endl;
	cout << typeid(CLA*).name() << endl;
	cout << typeid(CLC).name() << endl;
	cout << typeid(CLC*).name() << endl;

	//CLA a5(5);
	CLA* a6 = new CLC();
	CLA* a7 = new CLD();
	CLC* c5 = dynamic_cast<CLC*>(a6);
	CLD* d3 = dynamic_cast<CLD*>(a7);


	auto res0 = std::is_base_of<CLA, CLC>::value;
	if (std::is_base_of<CLA, CLC>::value)
	{
	}
	if (typeid(CLC*) == typeid(CLA*))
	{
		cout << "";
	}
	vector<CLA*> vecA;
	vecA.push_back(&c);
	CLA* pa = vecA.at(0);
	CLC* pc = dynamic_cast<CLC*>(pa);

	CLC* c1 = new CLC();
	CLA* a1 = dynamic_cast<CLA*>(c1);
	int res = a1->m_a;

	vector<shared_ptr<CLA>> vecPA;
	CLA* a2 = new CLA();
	//vecPA.push_back(shared_ptr<CLA>(a2));
	vecPA.push_back(make_shared<CLA>(CLA()));
	//using unique ptr
	vector<unique_ptr<CLA>> vecUA;
	unique_ptr<CLA> a3(new CLC());
	unique_ptr<CLA> a4(new CLC());
	unique_ptr<CLA> a5(new CLC());
	vecUA.push_back(move(a3));

	//vecUA.push_back(make_unique<CLC>(4));
	//unique_ptr<CLA> ptr = move(vecUA[0]);
	//int ma=ptr->m_a;


	map<string, unique_ptr<CLA>> Amap;
	Amap.insert(std::pair<string, unique_ptr<CLA> >("1", move(a4)));
	unique_ptr<CLA> cla = move(Amap.begin()->second);
	for (auto iter = Amap.begin(); iter != Amap.end(); iter++)
	{
		int i = 0;
	}
	int i = cla->m_a;
	auto cmp = move(Amap);

	unique_ptr<CLA> ptr(new CLC(1));
	ptr.get()->m_a;
	ptr->m_a;
	dynamic_cast<CLC*>(ptr.get());

	map<int, int> Bmap;
	Bmap.emplace(int(1), int(1));//原位构造

	return 0;
}



