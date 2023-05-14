#include"pch.h"

using namespace std;


//宏
#define ENROL_INTERFACE_MACRO(CLASS_NAME, FUN_NAME) \
enrolInterfaceThirdparty(typeid(CLASS_NAME), BPPropertyID(), [](BIMBase::Data::BPObject* ptr){\
    BIMBase_Common::CLASS_NAME* sub_ptr = dynamic_cast<BIMBase_Common::CLASS_NAME*>(ptr);\
    if (!(ptr && sub_ptr))\
        return BIMBase::ParaComponent::Gnrc();\
    return sub_ptr->FUN_NAME();\
    }, true)


static int _enrol0 = []()->int
{
	//ENROL_FUN(BCModCube, getHigh);
	//enrolInterfaceThirdparty(typeid(BCModCube), BPPropertyID(), std::bind(enrol_fun<BCModCube>, std::placeholders::_1, BCModCube::setWidth));
	return 0;
}();

//变长模板
template<typename T>
struct _Get_Number;
template<typename R, typename... Args>
struct _Get_Number<R(*)(Args...)> 
{
	static size_t const value = sizeof...(Args);
};

template<typename T>
size_t getInputVariablenumber(T) 
{
	return _Get_Number<T>::value;
}

void function1(int, int, double) {}
void function0(void) {}

template <class R, class... Args>
R getRetValue(R(*)(Args...));
template <class R, class... Args>
size_t getNumArgs(R(*)(Args...))
{
	return sizeof...(Args);
}
//using ret_t = decltype(getRetValue(f));


class Foo 
{
public:
	Foo(int d):data(d)
	{
	}
	void display_greeting() 
	{
		std::cout << "Hello, world.\n";
	}
	void display_number(int i) 
	{
		std::cout << "number: " << i << '\n';
	}
	int data = 7;
};


template <class _Rx, class	_Ty>
void enrol(long long id, _Rx _Ty::* _fun)
{
	auto get_fun = [&](BPObject* ptr)->int {
		_Ty* sub_prt = dynamic_cast<_Ty*>(ptr);
		if (!(sub_prt && ptr))
			return {};
		auto mf = std::mem_fn(_fun);
		return mf(*sub_prt);
	};
	//enrolInterfaceThirdparty(typeid(_Ty), BPPropertyID(), get_fun),true);
}

template <class _Callable, class _Ty1, class... _Types2>
int my_invoke(_Callable&& _Obj, _Ty1&& _Arg1, _Types2&&... _Args2)
{
	string name2 = typeid(_Obj).name();
	string name3 = typeid(_Arg1).name();
	size_t size = sizeof...(_Args2);
	return size;
	//if constexpr (_Invoker1<_Callable, _Ty1>::_Strategy == _Invoker_strategy::_Pmf_object)
	//{
	//	return (static_cast<_Ty1&&>(_Arg1).*_Obj)(static_cast<_Types2&&>(_Args2)...);
	//}
}

template <class T>
class Mem_fn : public _Weak_types<T> {
private:
	T _Pm;
	bool withPara;
public:
	size_t m_size;
	explicit Mem_fn(T _Val) : 
		_Pm(_Val) 
	{
		type_index name = typeid(_Val); // Cube::getLength
		const type_info& nInfo = typeid(_Val);
		size_t hash = typeid(_Val).hash_code();
		string name1 = typeid(_Val).name(); //"int (__cdecl Cube::*)(void)const __ptr64"
		auto a = name1.find("(void)");
		auto a1 = name1.find("(void)") == string::npos;
		withPara= name1.find("(void)") == string::npos;
		//auto a2 = name1.find("(void*)")!= string::npos;
		m_size = getNum(_Val);

	}
	template <class... _Types>
	int getNum(_Types&&... _Args)
	{
		size_t size = sizeof...(_Args);
		return sizeof...(_Types);
		//return 0;
	}

	template <class... _Types>
	int getNumber(_Types&&... _Args) const
	{
		return my_invoke(_Pm, std::forward<_Types>(_Args)...);
	}
	template <class... _Types>
	inline auto operator()(_Types&&... _Args) const
	{
		return std::invoke(_Pm, std::forward<_Types>(_Args)...);
	}
	// std::invoke std::invoke 接受一些可调用的东西，以及调用它的参数，然后进行调用。 std::invoke( f, args...) 是键入 f(args...) 的一个小泛化，它还处理一些其他情况。
		//可调用对象包括函数指针或引用、成员函数指针、带有 operator() 的对象或指向成员数据的指针。
		//在成员情况下，第一个参数被解释为 this。然后将剩余的参数传递给()(指向成员数据的指针除外)，其中 std::reference_wrapper 展开。
	// std::forward 通常是用于完美转发的，它会将输入的参数原封不动地传递到下一个函数中
};


template <class _Rx, class _Ty>
Mem_fn<_Rx _Ty::*> mem_fnc(_Rx _Ty::* _Pm) noexcept
//Mem_fn<void (Cube::*)(int length)> mf9
{
	return Mem_fn<_Rx _Ty::*>(_Pm);
}

//默认
//template<class ...Args>
//size_t ShowList(Args... args)
//{
//	cout << sizeof...(args) << endl; //获取参数包中参数的个数
//	return sizeof...(args);
//}

////递归终止函数
void ShowList()
{
	//cout << endl;
}
//展开函数
template<class T, class ...Args>
void ShowList(T value, Args... args)
{
	cout << value << " "; //打印分离出的第一个参数
	ShowList(args...);    //递归调用，将参数包继续向下传
}
//供外部调用的函数
template<class ...Args>
void ShowList(Args... args)
{
	ShowListArg(args...);
}


void doF1(int t1)
{
	cout << "arg1 type:" << typeid(t1).name() << " value:" << t1 << endl;
}

void doF2(int t1, const char* t2)
{
	cout << "arg1 type:" << typeid(t1).name() << " value:" << t1 << endl;
	cout << "arg2 type:" << typeid(t2).name() << " value:" << t2 << endl;
}

template<class Fn, class... Args>    //通过class...声明模板参数包Args
void f(Fn f, Args... args)           //args为参数包
{
	cout << "arg number: " << sizeof...(args) << endl; //通过sizeof...可以获得参数包内参数的数量
	f(args...);                                  //可通过args...对参数包进行解包，传入最终函数doF
}


class A
{
public:
	static void staticmember() { cout << "static" << endl; }   //static member
	void nonstatic() { cout << "nonstatic" << endl; }          //nonstatic member
	virtual void virtualmember() { cout << "virtual" << endl; };//virtual member
};

std::tuple<int, int> foo_tuple()
{
	auto info = std::make_pair(3.8, "Lisa Simpson");
	double score = 0.0;
	std::string name;
	std::tie(score, name) = info;


	//foo在C语言中经常作为方法名或者类名，英文全称为function object Oriented，即面向对象函数。
	return { 1, -1 }; // Error until N4387
	//return std::tuple<int, int>{1, -1}; // Always works
	//return std::make_tuple(1, -1); // Always works
}



int main_template () {

	A a;
	//static成员函数,取得的是该函数在内存中的实际地址，而且因为static成员是全局的，所以不能用A::限定符
	void (*ptrstatic)() = &A::staticmember;
	//nonstatic成员函数 取得的是该函数在内存中的实际地址     
	void (A:: * ptrnonstatic)() = &A::nonstatic;
	//虚函数取得的是虚函数表中的偏移值，这样可以保证能过指针调用时同样的多态效果
	void (A:: * ptrvirtual)() = &A::virtualmember;
	//函数指针的使用方式
	std::function<void(A*)> ptrnonstatic1 = &A::nonstatic;
	ptrstatic();
	(a.*ptrnonstatic)();
	(a.*ptrvirtual)();

	f(doF1, 0);
	f(doF2, 1, "hello");
	Cube cube(100, 100, 200);


	ShowList(1, 'A', string("hello"));
	size_t n1 = getInputVariablenumber(cube_setLength);
	size_t n2 = getInputVariablenumber(function1);
	size_t n4 = getNumArgs(function1);
	//ShowList(function1);
	//ShowList(function0);

	//auto mf1 = std::mem_fn(&BPGeometricPrimitive::initIndex);
	//auto mf2 = std::mem_fun(&BPGeometricPrimitive::initIndex);
	//auto mf3 = std::mem_fn(&BPGeometricPrimitive::setPropertyValue);
	//auto mf5 = mem_fnc(&BPGeometricPrimitive::initIndex);
	//size_t n5 = getInputVariablenumber(triple);
	auto mf6 = mem_fnc(&Cube::setLength); //custom

	//void * ptr=

	//std::map < std::function<void(int)>, int > test = { {obj,1} };

	//auto mf6 = mem_fnc(&Cube::getLength); //custom
	size_t size=mf6.m_size;
	mf6.getNumber(cube,1,1,1);

	// const_mem_fun_t<int, Cube> mf8
	//auto mf8 = mem_fun(&Cube::getLength); //cant found in C++17
	// mem_fun1_t<void, Cube, int> mf10
	//auto mf10 = mem_fun(&Cube::setLength);
	//auto mf12 = mem_fun(&Cube::setArea); //不支持多于1个参数

	// _Mem_fn<int (Cube::*)() const> mf7
	auto mf7 = mem_fn(&Cube::getLength);
	// _Mem_fn<void (Cube::*)(int length)> mf9
	auto mf9 = mem_fn(&Cube::setLength);
	// _Mem_fn<bool (Cube::*)(int length, int width)> 
	auto mf11 = mem_fn(&Cube::setArea);


	Foo f(8);
	auto memf1 = std::mem_fn(&Foo::display_greeting);
	memf1(f);
	auto memf2 = std::mem_fn(&Foo::display_number);
	memf2(f, 42);
	auto memf3 = std::mem_fn(&Foo::data);
	int resf = memf3(f);



	auto fpa = &BPGeometricPrimitive::initIndex;
	auto fpa2 = &BPGeometricPrimitive::setPropertyValue;
	type_index t1 = typeid(fpa);
	string t2 = typeid(fpa).name();
	type_index t3 = typeid(fpa2);
	string t4 = typeid(fpa2).name();

	std::function<void(BPGeometricPrimitive*)> fp = &BPGeometricPrimitive::initIndex;
	//size_t n5 = getNumArgs(&BPGeometricPrimitive::initIndex);
	//size_t n3 = getInputVariablenumber(fp);
	//enrol(1, &Cube::getLength);


	return 0;
}


static int _enrol = []()->int {

	return 0;
}();