#pragma once
using namespace std;
template <typename A, typename B>
//A add(const A& a, const B& b)
auto add(const A& a, const B& b)
{
	auto c= a + b;
	cout << typeid(c).name() << endl;
	return c;
	//cout << a + b << endl;
}



