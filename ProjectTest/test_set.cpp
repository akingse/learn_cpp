#include "pch.h"

#define _USE_MATH_DEFINES
using namespace std;


bool pnpoly(int nvert, float* vertx, float* verty, float testx, float testy)
{
    /*
    Argument	Meaning
    nvert	Number of vertices in the polygon. Whether to repeat the first vertex at the end is discussed below.
    vertx, verty	Arrays containing the x- and y-coordinates of the polygon's vertices.
    testx, testy	X- and y-coordinate of the test point.
    */
    bool c = false;
    int i, j;
    for (i = 0, j = nvert - 1; i < nvert; j = i++)
    {
        if (
            ((verty[i] > testy) != (verty[j] > testy)) &&
            (testx - vertx[i] < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) )
            )
            c = !c;
    }
    return c;
}

//inline 是加在头文件内的，用于声明「这个函数虽然定义在头文件内，但多个编译实体中只保留一份」。
//static 是加在源文件里边的，用于声明「这个函数仅用于当前源文件，其它编译实体不可用」。
static int mainstl()
{
    std::cout << "Hello World! test_set\n";
    float xList[5] = { 0, 10, 10, 0 };
    float yList[5] = { 0, 0, 10, 10 };
    int lenP = 4;
    //float x = 1, y = 1;
    float x = -1, y = 0;
    bool bl= pnpoly(lenP, xList, yList, x, y);

    //for (int i = 0; i <= 5; i++)
    //for (int i = 0; i <= 5; ++i) //效果一样，++i效率高
    //    std::cout << i << endl;
    set<string> myset = { "poly","cube" };
    myset.insert("point");
    myset.insert("line");
    myset.insert("arc");
    myset.insert("arc");
    auto a=myset.find("arc");
    auto b=myset.count("arc");
    auto c = myset.erase("line");
    std::cout << myset.count("arc") << endl;
    return 0;
}

static void _test_set_0()
{
    set<int> mset{ 1,2,3 };
    auto iter = mset.begin();
    //auto iter1 = iter++;
    auto iter1 = ++iter;
    auto iter2 = ++iter;

    set<int> myset;
    myset.insert(1);
    myset.insert(3);
    myset.insert(2);
    bool suc;
    auto it = myset.insert(4);
    suc = it.second;
    auto it2 = myset.erase(5);
    auto it3 = myset.find(2);

    set<int> mset1{ 1,2,3,4 };
    set<int> mset2{ 4,5,6 };
    //set里面有set_intersection（取集合交集）、
    //set_union（取集合并集）、set_difference（取集合差集）、set_symmetric_difference（取集合对称差集）等函数。
    set<int> intersection;
    set<int> convergence;
    set<int> difference;
    set_intersection(mset1.begin(), mset1.end(), mset2.begin(), mset2.end(), inserter(intersection, intersection.begin()));
    set_union(mset1.begin(), mset1.end(), mset2.begin(), mset2.end(), inserter(convergence, convergence.begin()));
    set_difference(mset1.begin(), mset1.end(), mset2.begin(), mset2.end(), inserter(difference, difference.begin()));


    return;
}

static void _test_set_2()
{
    set<int> set0{ 1,2,3 };
    set<int> set1{ 3,2,1 };

    set<int> set2{ 1,2,4 };
    set<int> set3{ 1,2,3,4 };
    //set可以使用==判断
    bool iseq = set0 == set1;
    bool iseq1 = set0 == set2;
    return;
}

static int enrol = []()->int
{
    //mainstl();
    //_test_set_1();
    _test_set_2();
    return 0;
}();
