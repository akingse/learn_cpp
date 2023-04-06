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

static int enrol = []()->int
{
    mainstl();
    return 0;
}();


