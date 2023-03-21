// is_copy_constructible example
#include "pch.h"
#include <iostream>
#include <type_traits>
using namespace std;

namespace CHECK
{
    struct No {};
    template<typename T, typename Arg> 
    No operator== (const T&, const Arg&);

    template<typename T, typename Arg = T>
    struct EqualExists
    {
        enum 
        { 
            value = !std::is_same<decltype(std::declval<T>() == std::declval<Arg>)(), No > ::value 
        };
    };
}


template <typename T>
struct has_equal_operator {
    template<typename U> static void test(...);

    template<typename U>
    static auto test(int)->decltype(std::declval<U>() == std::declval<U>());
    enum { value = std::is_same<decltype(test<T>(0)), bool>::value };
};


template <typename T>
struct has_asterisk_operator {
    template<typename U> static auto test(int)->decltype(*std::declval<U>());
    template<typename U> static void test(...);
    enum { value = std::is_reference<decltype(test<T>(0))>::value };
};


template <typename T>
struct has_gt_operator {
    template<typename U> static void test(...) {};
    template<typename U> static auto test(int)->decltype(std::declval<U>() > std::declval<U>()) {};
    static constexpr bool value = std::is_same<decltype(test<T>(0)), bool>::value;
};


template<typename T, bool>
class F
{
public:
    static void copy_assignable(T& l, const T& r);
};

template<typename T>
class F<T, true>
{
public:
    static void copy_assignable(T& l, const T& r)
    {
        std::cout << "can be ="<<endl;
        l = r;
    }
};
template<typename T>
class F<T, false>
{
public:
    static void copy_assignable(T& l, const T& r)
    {
#ifdef _DEBUG
        std::cout << "cannot be =" << endl;

        //throw std::logic_error("T no = error");
#endif // DEBUG
    }
};


//是否等于
template<typename T, bool>
class AbleEqual
{
public:
    static bool equal_operator(const T& l, const T& r);
};

template<typename T>
class AbleEqual<T, true>
{
public:
    static bool equal_operator(const T& l, const T& r)
    {
        std::cout << "can be =" << endl;
        return l == r;
    }
};
template<typename T>
class AbleEqual<T, false>
{
public:
    static bool equal_operator(const T& l, const T& r)
    {
#ifdef _DEBUG
        std::cout << "logic_error" << endl;

        //throw std::logic_error("T no = error");
#endif // DEBUG
        return false;
    }
};

template<typename T>
class Gnrc
{
    T imp;
public:
    Gnrc() {};
    Gnrc(const Gnrc& r)
    {
    }
    void operator=(const Gnrc& r)
    {
        //imp = r.imp;
        F<T, std::is_copy_assignable<T>::value>::copy_assignable(imp, r.imp);
    }
};

class C
{
public:
    void operator=(const C&) = delete;
};

using namespace rel_ops;//自动处理比较运算符


template<typename T >
class CC
{
public:
    T imp;
    bool operator < (const CC& other)
    {
        return imp < other.imp;
    }
    bool operator == (const CC& other)
    {
        return imp == other.imp;
    }
};


class D
{
public:
    D() {};
    D(int src) :imp(src)
    {}
    int fun() {}

    int imp;
    //bool operator < (const D& other) = delete;
    bool operator== (const D& other) ;
    //bool operator== (const D& other) = delete;

};


int main_ope() 
{

    {

        Gnrc<float> c1;
        Gnrc<float> c2;
        //c2 = c1;

    }
    {
        Gnrc<C> c1;
        Gnrc<C> c2;
        c2 = c1;
    }
    D d1;
    D d2;
    //d1 < d2;
    Gnrc<D> gd;
    map<D, int> mp;
    map<Gnrc<D>, int> mp1;
    //mp1[Gnrc(d1)] = 1;
    {
        Gnrc<D> d1;
        Gnrc<D> d2;
        //d2 < d1;
    }
    __if_exists(D::fun)
    {}

    std::cout << " has operator :" << has_equal_operator<int>::value << std::endl;
    std::cout << " has operator :" << has_equal_operator<std::string>::value << std::endl;
    std::cout << " has operator :" << has_equal_operator<D>::value << std::endl;


    return 0;
}