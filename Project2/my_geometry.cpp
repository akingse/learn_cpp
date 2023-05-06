#include "pch.h"



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



// Randolph Franklin���㷨
int pnpoly(int npol, float* xp, float* yp, float x, float y)
{
    int i, j, c = 0;
    for (i = 0, j = npol - 1; i < npol; j = i++)
    {//<== ��< �����̺�����
        if ((((yp[i] <= y) && (y < yp[j])) || ((yp[j] <= y) && (y < yp[i]))) && (x < (xp[j] - xp[i]) * (y - yp[i]) / (yp[j] - yp[i]) + xp[i]))
            c = !c;
    }
    return c;
}







static int _enrol2=[]()->int {
	Cube cube(100, 100, 200);

	auto type = &Cube::setHigh; //(0x00007ff6c3826a64)(0x000000c87852ebb8,100)-		&cube	0x000000c87852ebb8 {m_l=100 m_w=100 m_h=200 }	Cube *
	//constexpr  long long size = long long(&Cube::setHigh);
	std::function<int(Cube*)> fp = &Cube::getHigh;
	//��ʾָ������
	std::function<void(Cube*,int)> fp1 = &Cube::setHigh;
	fp1(&cube, 300);

	const std::map<std::string, std::map<std::string, DpIn*>>* pMap = &g_funMap;
	//ע�ắ����ͳһ�Ĳ�
	interface_enrol("CUBE", "��", cube_setLength);
	interface_enrol("CUBE", "��", cube_setWidth);
	interface_enrol("CUBE", "mf��", &Cube::setHigh);
	//enrol("CUBE", "��", cube_setHigh);

	BPGeometricPrimitive primitive(new Cube(100, 100, 200), false);
	//BPGeometricPrimitive primitive(nullptr, false);
	Cube* ptr = new Cube(100, 100, 200);
	const Cube cubec(100, 100, 200);
	int lengthc = cube.getLength();
	int length = cubec.getLength();
	auto mf4 = std::mem_fn(&Cube::setLength);
	BPObject* ptrO = ptr;
	Cube* sub_prt = dynamic_cast<Cube*>(ptrO);
	/*mf6(cube, 300); *///ע�����ƥ��
	type_index a1 = typeid(ptr);
	type_index a2 = typeid(Cube);
	BPObject ob = cube;
	type_index a3 = typeid(&ob);
	primitive.setPropertyValue("��", 5000);
	//primitive.setPropertyValue("mf��", 6000);
	bool res = cube_setLength(nullptr, 300);

	return 0;
}();