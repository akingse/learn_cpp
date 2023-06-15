#include "pch.h"
using namespace std;

class A
{
public:
	A() = default;
	virtual ~A() = default;

};

class B :public A
{
public:

	B() = default;

};

static bool _is_base_of(const type_info& typeBase, const type_info& typeChild)
{
	return typeBase.before(typeChild);
}

static void _type_cast()
{

	bool res = _is_base_of(typeid(A), typeid(B));
	bool resIn = _is_base_of(typeid(B), typeid(A));

	//const type_info&;
	auto& infoA = typeid(A);
	auto& infoB = typeid(B);
	type_index index = infoB;
	bool isIn1 = infoB.before(typeid(A));
	bool isIn2 = infoB.before(typeid(B));
	bool isIn3 = infoA.before(typeid(A));
	bool isIn4 = infoA.before(typeid(B)); //true
}

static int enrol = []()->int
{
	//_type_cast();
	return 0;
}();

