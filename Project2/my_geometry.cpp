#include "pch.h"
using namespace std;

Vec2 _get_circumcircle_center(const std::array<Vec2, 3>& trigon)
{
    {
        // _inverse_2x2
        double x1 = trigon[0].x;
        double y1 = trigon[0].y;
        double x2 = trigon[1].x;
        double y2 = trigon[1].y;
        double x3 = trigon[2].x;
        double y3 = trigon[2].y;

        double m1 = 2 * (x2 - x1);
        double n1 = 2 * (y2 - y1);
        double k1 = x2 * x2 - x1 * x1 + y2 * y2 - y1 * y1;
        double m2 = 2 * (x3 - x1);
        double n2 = 2 * (y3 - y1);
        double k2 = x3 * x3 - x1 * x1 + y3 * y3 - y1 * y1;
        double k = (m1 * n2 - n1 * m2);
        if (k == 0)  // is_float_zero(k)
            return {};
        return Vec2((n2 * k1 - n1 * k2) / k, (-m2 * k1 + m1 * k2) / k);
    }
}

//int    polySides;//=  how many cornersthe polygon has
//float  polyX[];//=  horizontalcoordinates of corners
//float  polyY[];//=  verticalcoordinates of corners
//float  x, y;//=  point to be tested

bool pointInPolygon(int    polySides, float  polyX[], float  polyY[],float x, float y)
{
    int   i, j = polySides - 1;
    bool  oddNodes = false;
    for (i = 0; i < polySides; i++)
    {
        //if ((polyY[i] < y && polyY[j] >= y || polyY[j] < y && polyY[i] >= y) && (polyX[i] <= x || polyX[j] <= x))
        if (polyY[i] < y && polyY[j] >= y || polyY[j] < y && polyY[i] >= y)
        {
            if (polyX[i] + (y - polyY[i]) / (polyY[j] - polyY[i]) * (polyX[j] - polyX[i]) < x)
            {
                oddNodes = !oddNodes;
            }
        }
        j = i;
    }
    return oddNodes;
}



// Randolph Franklin的算法
int pnpoly(int npol, float* xp, float* yp, float x, float y)
{
    int i, j, c = 0;
    for (i = 0, j = npol - 1; i < npol; j = i++)
    {//<== 和< 奥妙蕴含其中
        if ((((yp[i] <= y) && (y < yp[j])) || ((yp[j] <= y) && (y < yp[i]))) && (x < (xp[j] - xp[i]) * (y - yp[i]) / (yp[j] - yp[i]) + xp[i]))
            c = !c;
    }
    return c;
}


static void _test_geometry()
{
	Cube cube(100, 100, 200);

	auto type = &Cube::setHigh; //(0x00007ff6c3826a64)(0x000000c87852ebb8,100)-		&cube	0x000000c87852ebb8 {m_l=100 m_w=100 m_h=200 }	Cube *
	//constexpr  long long size = long long(&Cube::setHigh);
	std::function<int(Cube*)> fp = &Cube::getHigh;
	//显示指定类型
	std::function<void(Cube*, int)> fp1 = &Cube::setHigh;
	fp1(&cube, 300);

	const std::map<std::string, std::map<std::string, DpIn*>>* pMap = &g_funMap;
	//注册函数，统一改参
	interface_enrol("CUBE", "长", cube_setLength);
	interface_enrol("CUBE", "宽", cube_setWidth);
	interface_enrol("CUBE", "mf高", &Cube::setHigh);
	//enrol_fun("CUBE", "高", cube_setHigh);

	BPGeometricPrimitive primitive(new Cube(100, 100, 200), false);
	//BPGeometricPrimitive primitive(nullptr, false);
	Cube* ptr = new Cube(100, 100, 200);
	const Cube cubec(100, 100, 200);
	int lengthc = cube.getLength();
	int length = cubec.getLength();
	auto mf4 = std::mem_fn(&Cube::setLength);
	BPObject* ptrO = ptr;
	Cube* sub_prt = dynamic_cast<Cube*>(ptrO);
	/*mf6(cube, 300); *///注意参数匹配
	type_index a1 = typeid(ptr);
	type_index a2 = typeid(Cube);
	BPObject ob = cube;
	type_index a3 = typeid(&ob);
	primitive.setPropertyValue("长", 5000);
	//primitive.setPropertyValue("mf高", 6000);
	bool res = cube_setLength(nullptr, 300);
}

static int _enrol2=[]()->int 
{

	return 0;
}();