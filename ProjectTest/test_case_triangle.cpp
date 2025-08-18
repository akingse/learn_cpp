#include "pch.h"
using namespace std;
using namespace Eigen;
using namespace clash;
//三角面片求交，测试案例

static void EXPECT_TRUE(bool fun)
{
	if (fun)
		cout << "TRUE" << endl;
	else
		cout << "FALSE" << endl;
}

static void _test1()
{
	double a = 0.0;
	auto b = abs(a);
	std::array<Vector3f, 3>  triA;
	std::array<Vector3f, 3>  triB;
	//triA = { Vertex(0,0), Vertex(100,0), Vertex(0,100) };
	//triB = { Vertex(0,0), Vertex(100,0), Vertex(0,100) };
	//EXPECT_TRUE(TriangularIntersectionTest(triA, triB));
	//EXPECT_TRUE(isTwoTrianglesIntersection1(triA, triB));
	//点接触
	triA = triA;
	//triB = translate(100, 0) * triB;
	//EXPECT_TRUE(TriangularIntersectionTest(triA, triB));
	//EXPECT_TRUE(isTwoTrianglesIntersection1(triA, triB));
}

static int enrol = []()->int
{
	//_test1();
	return 0;
}();


