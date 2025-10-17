#include "pch.h"
using namespace eigen;
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

double getDepthZ(const Triangle3d trigon, const Vector2d& p)
{
	Triangle2d trigon2d = { to_vec2(trigon[0]), to_vec2(trigon[1]), to_vec2(trigon[2]) };
	Vector3d fno = (trigon[1] - trigon[0]).cross(trigon[2] - trigon[1]).normalized();

	if (!isPointInTriangle(p, trigon2d))
		return -1;
	double deno = Vector3d(0, 0, 1).dot(fno);
	if (deno == 0)
	{
		for (int j = 0; j < 3; j++)
		{
            if (0 < (p - trigon2d[j]).dot(p - trigon2d[(j + 1) % 3]))
				continue;
			Vector3d dir = trigon[(j + 1) % 3] - trigon[j];
			Vector3d normal = Vector3d(0, 0, 1).cross(dir);
			if (normal.isZero())
				return trigon[j].z();
			double k = (trigon[j] - to_vec3(p)).cross(dir).dot(normal) / (normal.dot(normal));
			//return k;
			continue;
		}
	}
	double k = (trigon[0] - Vector3d(p[0], p[1], 0)).dot(fno) / deno;
	return k;
}

static void _test3()
{
	Triangle3d trigon =
	{
		//Vector3d(0,0,1),
		//Vector3d(0,10,1),
		//Vector3d(0,5,11),

		Vector3d(0,0,1),
		Vector3d(0,10,1),
		Vector3d(0,0,11),
	};
	Vector2d p(0, 0);
	double z = getDepthZ(trigon, p);
	return;
}

static int enrol = []()->int
    {
        //_test1();
        //_test2();
        _test3();
		cout << get_filepath_filename(__FILE__) << " finished.\n" << endl;
		return 0;
    }();


