#include "pch.h"

using namespace std;


int counter = 1;

int fac(int n)
{
    cout << "第" << counter << "次调用：" << n << " * " << "fac(" << n - 1 << ")" << endl;
    if (n == 1) {
        return 1;
    }
    else
    {
        counter++;
        return n * fac(n - 1); // 断点1
    }
}

int main_recur()
{

    auto str = "thisishowishow";
    auto type = typeid(str).name();

    string str1 = "thisishowishow";
    auto type1 = typeid(str1).name();


    int n = 5;

    cout << "结果：fac(" << n << ")=" << endl << fac(n) << endl;
    return 0;
}