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

//inline �Ǽ���ͷ�ļ��ڵģ��������������������Ȼ������ͷ�ļ��ڣ����������ʵ����ֻ����һ�ݡ���
//static �Ǽ���Դ�ļ���ߵģ�����������������������ڵ�ǰԴ�ļ�����������ʵ�岻���á���
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
    //for (int i = 0; i <= 5; ++i) //Ч��һ����++iЧ�ʸ�
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

static void _test_set()
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
    //set������set_intersection��ȡ���Ͻ�������
    //set_union��ȡ���ϲ�������set_difference��ȡ���ϲ����set_symmetric_difference��ȡ���϶ԳƲ���Ⱥ�����
    set<int> intersection;
    set<int> convergence;
    set<int> difference;
    set_intersection(mset1.begin(), mset1.end(), mset2.begin(), mset2.end(), inserter(intersection, intersection.begin()));
    set_union(mset1.begin(), mset1.end(), mset2.begin(), mset2.end(), inserter(convergence, convergence.begin()));
    set_difference(mset1.begin(), mset1.end(), mset2.begin(), mset2.end(), inserter(difference, difference.begin()));


    return;
}

static int enrol = []()->int
{
    //_test_set();
    //mainstl();
    return 0;
}();
