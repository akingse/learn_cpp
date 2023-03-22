#include "pch.h"
using namespace std;

int main_geo()
{
	Cone cone;
	Cone cone1(1, 2);
	Cone cone2(0, 0);
	Cone cone3(3, 3);
	Cone cone4(0, 4);
	cone4.set_h(0);
	double h = cone3.get_h();
	ToGeCone::sm_coneMap;
	ToGeCone::sm_totalCount;
	Vec3 vec;

	//test 局部变量析构



	//test stl

	map<int, string> mymap{ {1,"1"} ,{2,"2"} ,{3,"3"} };
	auto d = mymap.begin()->first;
	auto e = mymap.begin()->second;

	map<int, string>::iterator iter = mymap.end();
	iter--;
	auto f = iter->first;
	auto g = iter->second;


	static constexpr double pla = 1;

	//string rule = "[[:graph:]]+@163[.]com";
//string rule = ".*@163[.]com";
//regex re(rule);
//string expre = "acvds.cpp@163.com";
//cout << regex_match(expre, re) << endl;    //1
//cout << regex_replace(expre, re, "ok") << endl;    //ok
//bool bb = isFloatZero(0.00001,0.001);

	for (int i = 0; i < 3; i++)
	{
		if (mymap.find(2) != mymap.end())
		{
			mymap.insert({ 4,"4" });
		}
	}




	//auto a = std::vector<int>{1};
	auto b = std::vector<int>{ 0,1,2,3,4,5,6,7 };
	auto c = set<int>{ 1,3,2 };
	c.insert(4);
	c.insert(0);
	c.insert(1);

	return 0;
}