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

//三角形夹角
static void _test2()
{
	array<Eigen::Vector3d, 3> trigon = {
		//Vector3d(0,0,0),
		//Vector3d(10,0,0),
		//Vector3d(-10,1,0),
		//Vector3d(608.02018563984893,742.78732411097735,283.178),
		//Vector3d(587.89506118441932,771.19127296749502,298.03699999999998),
		//Vector3d(587.75920581282116,742.135129773058,282.202),
        Vector3d(10270,558,0),
        Vector3d(10330,566,0),
        Vector3d(10290,563,0),
	};
	Vector3d edge0 = trigon[1] - trigon[0];
	Vector3d edge1 = trigon[2] - trigon[1];
	Vector3d edge2 = trigon[0] - trigon[2];
	double side0 = edge0.norm();
	double side1 = edge1.norm();
	double side2 = edge2.norm();
	//angle
	double cos01 = edge0.dot(edge1) / (side0 * side1);
	double cos12 = edge1.dot(edge2) / (side1 * side2);
	double cos20 = edge2.dot(edge0) / (side2 * side0);
	cos01 = fabs(fabs(cos01) - 1.0);
	cos12 = fabs(fabs(cos12) - 1.0);
	cos20 = fabs(fabs(cos20) - 1.0);
	return;
}

static int enrol = []()->int
    {
        //_test1();
        _test2();
		cout << get_filepath_filename(__FILE__) << " finished.\n" << endl;
		return 0;
    }();


