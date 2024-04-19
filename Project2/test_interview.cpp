#include"pch.h"
#include<iostream>
#include<string>
#include <unordered_map>

using namespace std;
#include "stdio.h"
#include "string.h"

class Father
{
public:
    void name() { printf("father name\n"); };
    virtual void call() { printf("father call\n"); };
};
class Son : public Father
{
public:
    void name() { printf("Son name\n"); };
    virtual void call() { printf("Son call\n"); };
    Son() = default;
    Son(char c)
    {
        printf("x");
    }
};


class CTest
{
public:
    int m = 0;
    CTest(int _m)
    {
        m = _m;
    }
    CTest()
    {
        m = 0;
        cout << 1;
    }
    CTest(const CTest&)
    {
        cout << 2;
    }

};

CTest fun(const CTest& b)
{
    return b;
}

static int _test2()
{
    CTest ct(1);
    CTest cb(2);
    ct = fun(cb);
    

    int a = 1 + 2;
    cout << "hello occ" << endl;
    Son* ptr;

    Son* Son1 = new Son();
    Father* father1 = (Father*)Son1;
    father1->call();
    father1->name();
    ((Son*)(father1))->call();
    ((Son*)(father1))->name();
    Father* f2 = new Father();
    Son* s2 = (Son*)f2;
    s2->call();
    s2->name();
    ((Father*)(s2))->call();
    ((Father*)(s2))->name();


    std::unordered_map<std::string, int> scores;
    auto res = scores.find(" ") == scores.end();

    return 0;
}


static int enrol = []()->int
    {
        _test2();

        cout << "test_interview finished.\n" << endl;
        return 0;
    }();

