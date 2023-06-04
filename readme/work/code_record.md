







### gtest项目崩溃

可能是项目配置不一致导致的

```cpp
#include "gtest/gtest.h"

#include <wchar.h>
#include <tchar.h>
#include "test_triangular.h"

using namespace std;
using namespace Eigen;
using namespace psykronix;

int _tmain(int argc,_TCHAR* argv[])
{
    testing::GTEST_FLAG(filter) = "hello";
    testing::InitGoogleTest(&argc, argv);
    RUN_ALL_TESTS();
    std::cout << "Hello World!\n";

    std::array<Vector3d, 3>  triA;
    std::array<Vector3d, 3>  triB;
    triA = { Vertex(0,0), Vertex(100,0), Vertex(0,100) };
    triB = { Vertex(-100,0), Vertex(-100,100) , Vertex(10,10,110) };
    //triB = translate(0, 0, -100) * triB;

    TriangularIntersectionTest(triA, triB);
    isTwoTrianglesIntersection(triA, triB);

}
```



### 三角形算法



```cpp




//static int a = 1; //test global var
//bool _isPointInTriangular(const BPParaVec& point, const std::array<BPParaVec, 3>& trigon)
//{
//	BPParaVec pA = trigon[0];
//	BPParaVec pB = trigon[1];
//	BPParaVec pC = trigon[2];
//	BPParaVec sdA = (point - pA) ^ (pB - pA);
//	BPParaVec sdB = (point - pB) ^ (pC - pB);
//	BPParaVec sdC = (point - pC) ^ (pA - pC);
//	return abs(norm(sdA) * norm(sdB) - (sdA * sdB)) < PL_Surface && abs(norm(sdA) * norm(sdC) - (sdA * sdC)) < PL_Surface;
//}


/*
快速互斥
即线段的外接矩形相交，线段才可能会相交
if(min(a.x,b.x)<=max(c.x,d.x) && min(c.y,d.y)<=max(a.y,b.y)&&min(c.x,d.x)<=max(a.x,b.x) && min(a.y,b.y)<=max(c.y,d.y))
　　return true;

*/

//bool _isPointInTriangular(const BPParaVec& point, const std::array<BPParaVec, 3>& trigon)
//{
//    BPParaVec sdA = (point - trigon[0]) ^ (trigon[1] - trigon[0]);
//    BPParaVec sdB = (point - trigon[1]) ^ (trigon[2] - trigon[1]);
//    BPParaVec sdC = (point - trigon[2]) ^ (trigon[0] - trigon[2]);
//    //isParallel(sdA , sdB); //double 
//    return abs(norm(sdA) * norm(sdB) - (sdA * sdB)) < PL_Surface && abs(norm(sdA) * norm(sdC) - (sdA * sdC)) < PL_Surface;
//}

//bool _isPointInTriangular(const BPParaVec& point, const std::array<BPParaVec, 3>& trigon)
//{
//	//if (abs(((trigon[1] - trigon[0]) ^ (trigon[2] - trigon[0])) * (point - trigon[0])) > PL_Surface) //not coplanar
// //       return false;
//	return ((trigon[1] - trigon[0]) ^ (point - trigon[0])) * ((trigon[2] - trigon[0]) ^ (point - trigon[0])) < PL_Surface &&
//        ((trigon[0] - trigon[1]) ^ (point - trigon[1])) * ((trigon[2] - trigon[1]) ^ (point - trigon[1])) < PL_Surface; // (x)*(x)<0 || abs()<PLA
//}

```



点线面计算

```cpp
BPParaTransform _getMatrixFromThreePoints(const std::array<BPParaVec, 3>& points)
{
    BPParaVec vecX = unitize(points[1]- points[0]);
    BPParaVec vecY = points[2]- points[0];
    BPParaVec vecZ = unitize(vecX ^ vecY); // been collinear judge
    vecY = (vecZ ^ vecX);
    return setMatrixByColumnVectors(vecX, vecY, vecZ, points[0]);
}

BPParaVec _getIntersectPointOfSegmentPlane(const BPParaVec& pA, const BPParaVec& pB, const BPParaVec& pOri, const BPParaVec& normal)
{
    double div = (normal * (pB - pA)); //
    if (abs(div) < PL_Surface)
    {
        return pA;
    }
    double k = (normal * (pA - pOri)) / div;
    return pA + k * (pA - pB);
}


```



```cpp
共面处理有问题

bool isTwoTrianglesIntersection(const std::array<Vector3f, 3>& triL, const std::array<Vector3f, 3>& triR)
{
    // using for many coplanar
    //if (abs(veczL.dot(triR[0] - triL[0])) < PL_Surface) //is point on triangular plane
    //{
    //    if (_isPointInTriangular(triR[0], triL))
    //        return true;
    //}
    //if (abs(veczL.dot(triR[1] - triL[0])) < PL_Surface) //is point on triangular plane
    //{
    //    if (_isPointInTriangular(triR[1], triL))
    //        return true;
    //}
    //if (abs(veczL.dot(triR[2] - triL[0])) < PL_Surface) //is point on triangular plane
    //{
    //    if (_isPointInTriangular(triR[2], triL))
    //        return true;
    //}

    // right through left plane
    Vector3f veczL = (triL[1] - triL[0]).cross(triL[2] - triL[0]);
    bool acrossR2L_A = (veczL.dot(triR[0] - triL[0])) * (veczL.dot(triR[1] - triL[0])) < eps; //include point-on-plane
	bool acrossR2L_B = (veczL.dot(triR[1] - triL[0])) * (veczL.dot(triR[2] - triL[0])) < eps;
	bool acrossR2L_C = (veczL.dot(triR[2] - triL[0])) * (veczL.dot(triR[0] - triL[0])) < eps;
	if (!acrossR2L_A && !acrossR2L_B && !acrossR2L_C)
		return false;

    // left through right plane
    //double dotL2R_A = (veczR.dot(triR[0] - triL[0])) * (veczR.dot(triR[1] - triL[0]));
	//double dotL2R_B = (veczR.dot(triR[1] - triL[0])) * (veczR.dot(triR[2] - triL[0]));
	//double dotL2R_C = (veczR.dot(triR[2] - triL[0])) * (veczR.dot(triR[0] - triL[0]));
    Vector3f veczR = (triR[1] - triR[0]).cross(triR[2] - triR[0]);
    bool acrossL2R_A = (veczR.dot(triR[0] - triL[0])) * (veczR.dot(triR[1] - triL[0])) < eps;
	bool acrossL2R_B = (veczR.dot(triR[1] - triL[0])) * (veczR.dot(triR[2] - triL[0])) < eps;
	bool acrossL2R_C = (veczR.dot(triR[2] - triL[0])) * (veczR.dot(triR[0] - triL[0])) < eps;
	if (!acrossL2R_A && !acrossL2R_B && !acrossL2R_C)
		return false;

	if (acrossR2L_A) // first filter
	{
		float deno = veczL.dot(triR[0] - triR[1]);
		if (fabs(deno) < eps) // perpendi
		{
			if (_isPointInTriangle(triR[0], triL) || _isPointInTriangle(triR[1], triL))
				return true;
		}
		else
		{
            Vector3f point = triR[0] + (veczL.dot(triR[0] - triL[0]) / deno) * (triR[1] - triR[0]);
			if (_isPointInTriangle(point, triL))
				return true;
		}
	}
	if (acrossR2L_B) // first filter
	{
        float deno = (veczL.dot(triR[1] - triR[2]));
		if (fabs(deno) < eps) // perpendi
		{
			if (_isPointInTriangle(triR[1], triL) || _isPointInTriangle(triR[2], triL))
				return true;
		}
		else
		{
            Vector3f point = triR[1] + (veczL.dot(triR[1] - triL[0]) / deno) * (triR[2] - triR[1]);
			if (_isPointInTriangle(point, triL))
				return true;
		}
	}
	if (acrossR2L_C) // first filter
	{
        float deno = (veczL.dot(triR[2] - triR[0]));
		if (fabs(deno) < eps) // perpendi
		{
			if (_isPointInTriangle(triR[2], triL) || _isPointInTriangle(triR[0], triL))
				return true;
		}
		else
		{
            Vector3f point = triR[2] + (veczL.dot(triR[2] - triL[0]) / deno) * (triR[0] - triR[2]);
			if (_isPointInTriangle(point, triL))
				return true;
		}
	}
    // reversal
    if (acrossL2R_A) // first filter
    {
        float deno = veczR.dot(triL[0] - triL[1]);
        if (fabs(deno) < eps) // perpendi
        {
            if (_isPointInTriangle(triL[0], triR) || _isPointInTriangle(triL[1], triR))
                return true;
        }
        else
        {
            Vector3f point = triL[0] + (veczL.dot(triL[0] - triR[0]) / deno) * (triL[1] - triL[0]);
            if (_isPointInTriangle(point, triR))
                return true;
        }
    }
    if (acrossL2R_B) // first filter
    {
        float deno = veczR.dot(triL[1] - triL[2]);
        if (fabs(deno) < eps) // perpendi
        {
            if (_isPointInTriangle(triL[1], triR) || _isPointInTriangle(triL[2], triR))
                return true;
        }
        else
        {
            Vector3f point = triL[0] + (veczL.dot(triL[0] - triR[0]) / deno) * (triL[1] - triL[0]);
            if (_isPointInTriangle(point, triR))
                return true;
        }
    }
    if (acrossL2R_B) // first filter
    {
        float deno = veczR.dot(triL[2] - triL[0]);
        if (fabs(deno) < eps) // perpendi
        {
            if (_isPointInTriangle(triL[2], triR) || _isPointInTriangle(triL[0], triR))
                return true;
        }
        else
        {
            Vector3f point = triL[2] + (veczL.dot(triL[2] - triR[0]) / deno) * (triL[0] - triL[2]);
            if (_isPointInTriangle(point, triR))
                return true;
        }
    }
	return false;
}



bool isTwoTrianglesIntersection(const std::array<Eigen::Vector3d, 3>& triL, const std::array<Eigen::Vector3d, 3>& triR)
{
    std::array<Vector3f, 3>  triA = { triL[0].cast<float>(), triL[1].cast<float>(), triL[2].cast<float>() };
    std::array<Vector3f, 3>  triB = { triR[0].cast<float>(), triR[1].cast<float>(), triR[2].cast<float>() };
    return isTwoTrianglesIntersection(triA, triB);
}

```

