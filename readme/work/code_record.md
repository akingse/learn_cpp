### gtest项目崩溃

可能是项目配置不一致导致的

破案，被测项目不是lib，是可执行exe

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





### 距离计算 Tri

```
double TriDist(double p[3], double q[3], const double s[3][3], const double t[3][3]);

计算两个三角形上最近的点，并返回它们之间的距离。
如果三角形重叠，则p和q是三角形中的一对随机点，而不是三角形交点上的重合点

For each edge pair, the vector connecting the closest points of the edges defines a slab (parallel planes at head and tail enclose the slab). If we can show that the off-edge vertex of each triangle is outside of the slab, then the closest points of the edges are the closest points for the triangles.Even if these tests fail, it may be helpful to know the closest points found, and whether the triangles were shown disjoint.

对于每个边对，连接边的最近点的向量定义了一个板（头部和尾部的平行平面包围了板）。如果我们可以证明每个三角形的离边顶点都在板外，那么边的最近点就是三角形的最接近点。即使这些测试失败，了解找到的最接近的点以及三角形是否显示为不相交也可能很有帮助。


No edge pairs contained the closest points.  
either:
1. one of the closest points is a vertex, and the   other point is interior to a face.
2. the triangles are overlapping.
3. an edge of one triangle is parallel to the other's face. If cases 1 and 2 are not true, then the closest points from the 9 edge pairs checks above can be taken as closest points for the triangles.
4. possibly, the triangles were degenerate.  When the triangle points are nearly colinear or coincident, one 
   of above tests might fail even though the edges tested contain the closest point
   
没有包含最近点的边对。
或者：
1.最近的点之一是顶点，而另一个点是面的内部。
2.三角形是重叠的。
3.一个三角形的边与另一个的面平行。如果情况1和2不成立，则可以将上述9个边对检查中的最近点作为三角形的最近点。
4.三角形可能是退化的。当三角形点几乎共线或重合时
即使测试的边包含最接近的点，上述测试中的一个也可能失败



```



```

	//bug测试
	Vector3d triA0 = Vector3d(4928967.0000119563, -378740.42555394967, 5318.0000000002756);
	Vector3d triA1 = Vector3d(4928973.9922683481, -378747.41781034082, 5319.5661914788307);
	Vector3d triA2 = Vector3d(4928989.6274289545, -378763.05297094764, 5350.0000000002756);
	Vector3d triB0 = Vector3d(4928967.1581119057, -378706.64252840198, 5357.7254248593727);
	Vector3d triB1 = Vector3d(4928961.6339804111, -378710.43624643586, 5350.0000000000000);
	Vector3d triB2 = Vector3d(4928962.4991871417, -378711.30145316647, 5357.7254248593645);

	Vector3d triA_0 = Vector3d(4924500.816042, -384858.485537, 5350.000000);
	Vector3d triA_1 = Vector3d(4924500.816042, -384856.898474, 5364.085575);
	Vector3d triA_2 = Vector3d(4924500.816042, -384852.216866, 5377.464841);
	Vector3d triB_0 = Vector3d(4924394.810992, -384838.186268, 5384.291939);
	Vector3d triB_1 = Vector3d(4924500.816042, -384838.186268, 5384.291939);
	Vector3d triB_2 = Vector3d(4924500.816042, -384829.477476, 5393.000732);
```



### Project2 项目配置

包含

```

```

库LibraryPath

```


//origin
$(SolutionDir)..\TPL\boost-1.81.0\stage\lib;
$(SolutionDir)..\TPL\boost-1.81.0\libs;
$(SolutionDir)..\TPL\CGAL-5.5.2\auxiliary\gmp\lib;

$(SolutionDir)build\Project1\;
$(SolutionDir)build\Project1\$(PlatformTarget)\link;
$(SolutionDir)$(Platform)\$(Configuration)\

```

```
Project2.exe - System Error

The code execution cannot proceed because Project_cal.dlI was not found. Reinstalling the program may fix this problem.

```

