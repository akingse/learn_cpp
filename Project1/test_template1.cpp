#include "pch.h"
using namespace std;

template <typename T, typename = void>
static constexpr bool __has_transform = false;
template <typename T>
static constexpr bool __has_transform<T, std::void_t<decltype(std::declval<T>().fun(std::declval<double>(), std::declval<bool>()))>> = true;

// std::void_t<>
// std::decltype()
// std::declval<>



template<typename T>
class __Primitive
{
    T m_data;
    __Primitive(const T& data) :
        m_data(data)
    {
    }
    template <bool B = __has_transform<T>>
    void __transform(const BPTransfrom& transform, bool local) const;
    template <>
    void __transform<true>(const BPTransfrom& transform, bool local) const
    {
        
        cout << "true" << endl;
    }
    template <>
    void __transform<false>(const BPTransfrom& transform, bool local) const
    {
        cout << "false" << endl;

    }

};


class TClass1
{
public:
    int test()
    {
        int a = 1;
        return a;
    }
};

//==================================================================================================================
//void_t 判断 类型是否存在

class T
{
public:
    int test() { int a = 1; }
};
//std::declval :将任何一个类型T转换成引用类型， 在不创建对象的情况下能达到创建对像的效果，
//可以使 decltype 表达式中不必经过构造函数就能使用成员函数
//eg : decltype(std::declval<T>.test())   //int 这里没有创建T的对象， 也没有执行a = 1;

//给定一个变量或表达式，decltype能够推导出他的类型。最重要的是能够不需要计算表达式就可以推导出表达式所得值的类型。
struct Default 
{
	int foo(/*double d*/) const
	{
		return 1;
	}
};
struct NonDefault
{
    NonDefault(const NonDefault&) 
    { }
    int foo() const 
	{
		return 1;
	}
};

decltype(Default().foo()) n1 = 1;

//==================================================================================================================

#include <iostream> 
namespace 
{   
	struct base_t 
	{
		virtual ~base_t() {}
	};

    template<class T>     
    struct Base : public base_t 
	{
		virtual T t() = 0;
	};

    template<class T>     
    struct A : public Base<T> 
    { 
        ~A() {}      
		virtual T t() override 
		{
			std::cout << "A" << '\n'; 
            return T{};
		}
	};
} 

class A1 {
public:
    A1(int i) {
        printf("A::A()函数执行了，this = %p\n", this);
    }

    double myfunc() {
        printf("A::myfunc()函数执行了，this = %p\n", this);
        return 12.1;
    }
};

//==================================================================================================================
// https://www.jianshu.com/p/24b1a7045bea
#define HAS_MEMBER(XXX) \
template<typename T, typename... Args>\
struct has_member_##XXX \
{ \
private:  \
  template<typename U> static auto Check(int) -> decltype(std::declval<U>().XXX(std::declval<Args>()...), std::true_type());  \
  template<typename U> static std::false_type Check(...); \
public: \
  static constexpr auto value = decltype(Check<T>(0))::value; \
}

HAS_MEMBER(foo);
//class A2 {};
class A2
{
public:
    void foo(int, double) {}
};
bool ret = has_member_foo<A2, int, double>::value;
//==================================================================================================================

template <class T, class = void>
struct has_foo : std::false_type {};

template <class T>
struct has_foo<T, std::void_t<decltype(std::declval<T&>().foo())>> : std::true_type {};

template <class T, class = void>
struct has_foo_int : std::false_type {};

template <class T>
struct has_foo_int<T, std::void_t<decltype(std::declval<T&>().foo(std::declval<int>()))>> : std::true_type {};

class A3 {};

class B { public:void foo() {} };

class C { void foo() {} };

class D { public:int foo() { return 1; } };

class E { public:int foo(int) { return 2; } };


int main_p1()
{
    //序列化
    std::vector<unsigned char> buf(sizeof(Vec3));
    Vec3 vecIn(1,1,1);
	memcpy(buf.data(), &vecIn, sizeof(Vec3));
    //反序列化
    Vec3 vecOut;
    memcpy(&vecOut, buf.data(), sizeof(Vec3));



    //bool exp0 = std::void_t<decltype(std::declval<Default>().foo(std::declval<double>())) >>;
    //bool exp1= std::void_t<decltype(std::declval())>
    //bool exp = __has_transform<TClass1>();

    int i = 33;  
    decltype(i) j = i * 2;   //类型表达和使用
    std::cout << typeid(j).name() << " : " << j << endl;


    decltype(std::declval<A<int>>().t()) a{}; // = int a;  
    decltype(std::declval<Base<int>>().t()) b{};     // = int b;   
    std::cout << a << ',' << b << '\n';


    using YT = decltype(std::declval<A1>());//不要丢到declval<A>() 后的括号，因为是函数嘛，否则代码含义发生变化


    //using boost::typeindex::type_id_with_cvr;
    //cout << "YT = " << type_id_with_cvr<YT>().pretty_name() << endl;//显式YT类型

    std::cout << "A" <<bool(has_foo<A3>::value) << std::endl;//false
    std::cout << "B" <<bool(has_foo<B>::value) << std::endl;//true
    std::cout << "C" <<bool(has_foo<C>::value) << std::endl;//false
    std::cout << "D" <<bool(has_foo<D>::value) << std::endl;//true
    std::cout << "E" <<bool(has_foo<E>::value) << std::endl;//false
    std::cout << "E" <<bool(has_foo_int<E>::value) << std::endl;//true
    std::cout << "D" <<bool(has_foo_int<D>::value) << std::endl;//false

    vector<int> vecSize;
    vector<int> vecCapacity;

    //在vector中size()和capacity()属性分别对应着resize(size_type)和reserve(size_type)这两个方法。
    //size: 表示实际容器中保存元素的个数
    //capaciy : 表示在发生重新分配之前允许存放多少元素


    vecSize.resize(2); // 2 2
    cout << "vecSize.size() = " << vecSize.size() << ", vecSize.capacity() = " << vecSize.capacity() << endl;
    vecCapacity.reserve(3); // 0 3
    cout << "vecCapacity.size() = " << vecCapacity.size() << ", vecCapacity.capacity() = " << vecCapacity.capacity() << endl;

    return 0;
}

