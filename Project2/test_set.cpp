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

int mainset()
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
    auto a=myset.find("arc");
    auto b=myset.count("arc");
    std::cout << myset.count("arc") << endl;

    return 0;

}

