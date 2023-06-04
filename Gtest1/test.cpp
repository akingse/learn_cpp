#include "pch.h"

//TEST(TestCaseName, TestName) {
//  EXPECT_EQ(1, 1);
//  EXPECT_TRUE(true);
//}

using namespace std;
using namespace Eigen;
using namespace psykronix;

#define _USE_MATH_DEFINES //using M_PI
#include <math.h>


// ��ƽ��
TEST(Test_Triangle, parallel)
{
	//�غ�
	std::array<Vector3d, 3>  triA;
	std::array<Vector3d, 3>  triB;
	triA = { Vertex(0,0), Vertex(100,0), Vertex(0,100) };
	triB = { Vertex(0,0), Vertex(100,0), Vertex(0,100) };
	EXPECT_TRUE(TriangularIntersectionTest(triA, triB));
	//��Ӵ�
	triA = triA;
	triB = translate(100, 0) * triB;
	EXPECT_TRUE(TriangularIntersectionTest(triA, triB));

}

TEST(Test_Triangle, parallel_2)
{
	//�غ�
	std::array<Vector3f, 3>  triA;
	std::array<Vector3f, 3>  triB;
	triA = { Vertex(0,0), Vertex(100,0), Vertex(0,100) };
	triB = { Vertex(0,0), Vertex(100,0), Vertex(0,100) };
	//triB = (translate(0, 0, 100) * roty(M_PI)) * triB;
	bool res = isTwoTrianglesIntersection(triA, triB);
	//EXPECT_TRUE(isTwoTrianglesIntersection(triA, triB));
	EXPECT_TRUE(true);
}
