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
//void_t �ж� �����Ƿ����

class T
{
public:
    int test() { int a = 1; }
};
//std::declval :���κ�һ������Tת�����������ͣ� �ڲ����������������ܴﵽ���������Ч����
//����ʹ decltype ���ʽ�в��ؾ������캯������ʹ�ó�Ա����
//eg : decltype(std::declval<T>.test())   //int ����û�д���T�Ķ��� Ҳû��ִ��a = 1;

//����һ����������ʽ��decltype�ܹ��Ƶ����������͡�����Ҫ�����ܹ�����Ҫ������ʽ�Ϳ����Ƶ������ʽ����ֵ�����͡�
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
        printf("A::A()����ִ���ˣ�this = %p\n", this);
    }

    double myfunc() {
        printf("A::myfunc()����ִ���ˣ�this = %p\n", this);
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
    //���л�
    std::vector<unsigned char> buf(sizeof(Vec3));
    Vec3 vecIn(1,1,1);
	memcpy(buf.data(), &vecIn, sizeof(Vec3));
    //�����л�
    Vec3 vecOut;
    memcpy(&vecOut, buf.data(), sizeof(Vec3));



    //bool exp0 = std::void_t<decltype(std::declval<Default>().foo(std::declval<double>())) >>;
    //bool exp1= std::void_t<decltype(std::declval())>
    //bool exp = __has_transform<TClass1>();

    int i = 33;  
    decltype(i) j = i * 2;   //���ͱ���ʹ��
    std::cout << typeid(j).name() << " : " << j << endl;


    decltype(std::declval<A<int>>().t()) a{}; // = int a;  
    decltype(std::declval<Base<int>>().t()) b{};     // = int b;   
    std::cout << a << ',' << b << '\n';


    using YT = decltype(std::declval<A1>());//��Ҫ����declval<A>() ������ţ���Ϊ�Ǻ����������뺬�巢���仯


    //using boost::typeindex::type_id_with_cvr;
    //cout << "YT = " << type_id_with_cvr<YT>().pretty_name() << endl;//��ʽYT����

    std::cout << "A" <<bool(has_foo<A3>::value) << std::endl;//false
    std::cout << "B" <<bool(has_foo<B>::value) << std::endl;//true
    std::cout << "C" <<bool(has_foo<C>::value) << std::endl;//false
    std::cout << "D" <<bool(has_foo<D>::value) << std::endl;//true
    std::cout << "E" <<bool(has_foo<E>::value) << std::endl;//false
    std::cout << "E" <<bool(has_foo_int<E>::value) << std::endl;//true
    std::cout << "D" <<bool(has_foo_int<D>::value) << std::endl;//false

    vector<int> vecSize;
    vector<int> vecCapacity;

    //��vector��size()��capacity()���Էֱ��Ӧ��resize(size_type)��reserve(size_type)������������
    //size: ��ʾʵ�������б���Ԫ�صĸ���
    //capaciy : ��ʾ�ڷ������·���֮ǰ�����Ŷ���Ԫ��


    vecSize.resize(2); // 2 2
    cout << "vecSize.size() = " << vecSize.size() << ", vecSize.capacity() = " << vecSize.capacity() << endl;
    vecCapacity.reserve(3); // 0 3
    cout << "vecCapacity.size() = " << vecCapacity.size() << ", vecCapacity.capacity() = " << vecCapacity.capacity() << endl;

    return 0;
}

