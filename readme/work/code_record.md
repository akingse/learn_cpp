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

包含 IncludePath

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

依赖路径不正确
```



### 循环缩小包围盒里的三角面

// return index-vector of two mesh

```c
std::array<std::vector<size_t>, 2> _getPotentialIntersectTrianglesOfMesh(ModelMesh& mesh_a, ModelMesh& mesh_b, const Eigen::AlignedBox3d& box, double tolerance)
{
    //Eigen::Affine3d relative_matrix = mesh_b.pose_.inverse() * mesh_a.pose_; // model_a * a / b
    //// machine error process
    //for (size_t i = 0; i < 3; i++)
    //{
    //    for (size_t j = 0; j < 3; j++)
    //    {
    //        relative_matrix(i, j) = abs(relative_matrix(i, j)) < 1e-14 ? 0.0 : relative_matrix(i, j);
    //        relative_matrix(i, j) = abs(relative_matrix(i, j) - 1.0) < 1e-14 ? 1.0 : relative_matrix(i, j);
    //    }
    //    relative_matrix(i, 3) = std::round(relative_matrix(i, 3) * 1e9) / 1e9;
    //}
    //Eigen::AlignedBox3d boxMag(box.min() - 0.5 * Vector3d(tolerance, tolerance, tolerance), box.max() + 0.5 * Vector3d(tolerance, tolerance, tolerance));
    Eigen::AlignedBox3d boxMag(box.min() - Vector3d(tolerance, tolerance, tolerance), box.max() + Vector3d(tolerance, tolerance, tolerance));
    std::vector<size_t> triA_Index; // using index of mesh IBO
    Eigen::AlignedBox3d boxA_Min; // iterate to lessen box
    for (size_t i = 0; i < mesh_a.ibo_.size(); ++i)
    {
        std::array<Eigen::Vector3d, 3> triIter = {
                mesh_a.vbo_[mesh_a.ibo_[i][0]],
                mesh_a.vbo_[mesh_a.ibo_[i][1]],
                mesh_a.vbo_[mesh_a.ibo_[i][2]] };
        //enlarge box while has tolerance
        if (isTriangleAndBoundingBoxIntersect(triIter, boxMag))
        {
            triA_Index.push_back(i);
            boxA_Min.extend(triIter[0]);
            boxA_Min.extend(triIter[1]);
            boxA_Min.extend(triIter[2]);
        }
    }
    if (triA_Index.empty())
        return {};
    std::vector<size_t> triB_Index;
    Eigen::AlignedBox3d boxB_Min;
    for (size_t j = 0; j < mesh_b.ibo_.size(); ++j)
    {
        std::array<Eigen::Vector3d, 3> triIter = {
                mesh_b.vbo_[mesh_b.ibo_[j][0]],
                mesh_b.vbo_[mesh_b.ibo_[j][1]],
                mesh_b.vbo_[mesh_b.ibo_[j][2]] };
        if (isTriangleAndBoundingBoxIntersect(triIter, boxMag))
        {
            triB_Index.push_back(j);
            boxB_Min.extend(triIter[0]);
            boxB_Min.extend(triIter[1]);
            boxB_Min.extend(triIter[2]);
        }
    }
    if (triB_Index.empty())
        return {};
    // second bounding-box intersect
    if (!boxA_Min.intersects(boxB_Min))
        return {};
#ifdef STATISTIC_DATA_COUNT
    count_triA_inter += triA_Index.size();
    count_triB_inter += triB_Index.size();
#endif
    bool isMinA = false;
    bool isMinB = false;
    while (!isMinA && !isMinB)
    {
        Eigen::AlignedBox3d box_Inter = boxA_Min.intersection(boxB_Min);
        box_Inter.min() = box_Inter.min() - Vector3d(tolerance, tolerance, tolerance);
        box_Inter.max() = box_Inter.max() + Vector3d(tolerance, tolerance, tolerance);
        Eigen::AlignedBox3d boxA_Temp;
        std::vector<size_t> triA_IndexTemp;
        if (!isMinA)
        {
            for (auto& iA : triA_Index)
            {
                std::array<Eigen::Vector3d, 3> triA_Temp = {
                        mesh_a.vbo_[mesh_a.ibo_[iA][0]],
                        mesh_a.vbo_[mesh_a.ibo_[iA][1]],
                        mesh_a.vbo_[mesh_a.ibo_[iA][2]] };
                if (isTriangleAndBoundingBoxIntersect(triA_Temp, box_Inter))
                {
                    triA_IndexTemp.push_back(iA);
                    boxA_Temp.extend(triA_Temp[0]);
                    boxA_Temp.extend(triA_Temp[1]);
                    boxA_Temp.extend(triA_Temp[2]);
                }
            }
            if (triA_IndexTemp.empty())
                return {};
            if (boxA_Temp.contains(boxA_Min.min()) && boxA_Temp.contains(boxA_Min.max())) // boxA_Temp==boxA_Min
            {
                isMinA = true;//end iterate
            }
            else //iterate min box
            {
                boxA_Min = boxA_Temp;
                triA_Index = triA_IndexTemp;
            }
        }
        if (!isMinB)
        {
            Eigen::AlignedBox3d boxB_Temp;
            std::vector<size_t> triB_IndexTemp;
            for (auto& iB : triB_Index)
            {
                std::array<Eigen::Vector3d, 3> triB_Temp = {
                    mesh_b.vbo_[mesh_b.ibo_[iB][0]],
                    mesh_b.vbo_[mesh_b.ibo_[iB][1]],
                    mesh_b.vbo_[mesh_b.ibo_[iB][2]] };
                if (isTriangleAndBoundingBoxIntersect(triB_Temp, box_Inter))
                {
                    triB_IndexTemp.push_back(iB);
                    boxB_Temp.extend(triB_Temp[0]);
                    boxB_Temp.extend(triB_Temp[1]);
                    boxB_Temp.extend(triB_Temp[2]);
                }
            }
            if (triB_IndexTemp.empty())
                return {};
            if (boxB_Temp.contains(boxB_Min.min()) && boxB_Temp.contains(boxB_Min.max())) // boxB_Temp==boxB_Min
            {
                isMinB = true; //end iterate
            }
            else //iterate min box
            {
                boxB_Min = boxB_Temp;
                triB_Index = triB_IndexTemp;
            }
        }
        if (!boxA_Min.intersects(boxB_Min))
            return {};
    }
	return { triA_Index, triB_Index };
}

```



```c
//对比开关优化，

#ifdef DETECTION_BUG_HARD
	for (size_t i = 0; i < mesh_a.ibo_.size(); i++)
	{
		for (size_t j = 0; j < mesh_b.ibo_.size(); j++)
		{
			std::array<Eigen::Vector3d, 3> triA = {
				relative_matrix * mesh_a.vbo_[mesh_a.ibo_[i][0]],
				relative_matrix * mesh_a.vbo_[mesh_a.ibo_[i][1]],
				relative_matrix * mesh_a.vbo_[mesh_a.ibo_[i][2]] };
			std::array<Eigen::Vector3d, 3> triB = {
				mesh_b.vbo_[mesh_b.ibo_[j][0]],
				mesh_b.vbo_[mesh_b.ibo_[j][1]],
				mesh_b.vbo_[mesh_b.ibo_[j][2]] };
			//if (isTwoTrianglesIntersection(triA, triB) != isTwoTrianglesIntersection2(triA, triB))
			//{
			//    triInterListExtra.push_back({ triA, triB });
			//}
			if (isTwoTrianglesIntersectionSAT(triA, triB))
			{
				bool findFlag = false;
				for (const auto& iter : triInterList) //findTri
				{
					if (iter[0][0].x() == triA[0].x() && iter[0][0].y() == triA[0].y() && iter[0][0].z() == triA[0].z() &&
						iter[0][1].x() == triA[1].x() && iter[0][1].y() == triA[1].y() && iter[0][1].z() == triA[1].z() &&
						iter[0][2].x() == triA[2].x() && iter[0][2].y() == triA[2].y() && iter[0][2].z() == triA[2].z() &&
						iter[1][0].x() == triB[0].x() && iter[1][0].y() == triB[0].y() && iter[1][0].z() == triB[0].z() &&
						iter[1][1].x() == triB[1].x() && iter[1][1].y() == triB[1].y() && iter[1][1].z() == triB[1].z() &&
						iter[1][2].x() == triB[2].x() && iter[1][2].y() == triB[2].y() && iter[1][2].z() == triB[2].z())
					{
						findFlag = true;
						break;
					}
				}
				if (!findFlag) //record the trigons that detect by function, but exclude by pre-box
					triInterListExtra.push_back({ triA, triB }); //多余的相交三角形
				return true;
			}
		}
	}
#endif
```





#### 排除退化的三角形

```c
LoadModel流程
edgesA[0].cross(edgesA[1]).isZero()

before
<linkage count_entity="5233" count_triangle="1982865"/>
after
<linkage count_entity="5233" count_triangle="1982041"/>



				if (face_temp.size() == 3)
				{
					// exclude degenerated triangle
					if (!(mesh.vbo_[face_temp.at(1)] - mesh.vbo_[face_temp.at(0)]).cross(
						mesh.vbo_[face_temp.at(2)] - mesh.vbo_[face_temp.at(0)]).isZero())
						//count_err_degen_tri++;
						mesh.ibo_.push_back({ face_temp.at(0), face_temp.at(1), face_temp.at(2) });
				}
				face_temp.clear();
				
				
			std::array<int, 3> face_temp;
			int idx = 0;
			for (size_t i_index = 0; i_index < indexes_raw.size(); i_index++)
			{
				int index = indexes_raw.at(i_index);
				if (index != 0)
				{
					face_temp[idx] = abs(index) - 1;
					idx++;
				}
				if (idx % 3 == 0)
				{
					if (!(mesh.vbo_[face_temp[1]] - mesh.vbo_[face_temp[0]]).cross(
						mesh.vbo_[face_temp[2]] - mesh.vbo_[face_temp[1]]).isZero())
						mesh.ibo_.push_back(face_temp);
					idx = 0;
				}
			}
```





```C++
//box judge
((bounding.min() - bounding.max() - tolerance_longlong).array() > 0).any()
//判出isSepa
	box12.min() = box12.min() - tolerance_double;
	box12.max() = box12.max() + tolerance_double;
	bool isSepa0 = !box12.intersects(box22);

cwiseQuotient //对矩阵或数组的每个元素执行除法操作
// intersects判定
(a.min.array()<=(b.max)().array()).all() && ((b.min)().array()<=a.max.array()).all();
//box intersection规则
min=MaxBox.min()
min=MinBox.max()
    
bool is_soft = tolerance > 0 ? ((bounding.min() - bounding.max()).array() > 0).any() : false;

if (tolerance > 0) //tolerance > 0 软碰撞
    return !box1.intersects(box2); 
	isSape == true // boxSepa, enlarge boxInter ==> getDist, if d>tole exclude
	isInter==false // boxInter ==>if isInter, true: d=0, false: getDist
else
    return false; //tolerance==0 硬碰撞，boxInter ==> only judge isInter



tolerance == 0
	MeshIntrusionTesting

tolerance > 0
    isInter==false //boxInter
    	if MeshIntrusionTesting
            d=0
        else
            MeshStandoffDistance
    isSape == true //boxSepa
    	MeshStandoffDistance
```





```
                        //for (size_t i = 0; i < triInterList.size(); i++)
                        //{
                        //    {
                        //        tinyxml2::XMLElement* clashpoint = doc.NewElement("clashpoint");
                        //        {
                        //            tinyxml2::XMLElement* pos3f = doc.NewElement("pos3d");
                        //            std::array<std::array<Eigen::Vector3d, 3>, 2> triPair = triInterList[i];
                        //            for (int i = 0; i < 3; ++i)
                        //            {
                        //                std::string point; 
                        //                std::stringstream ssX;
                        //                std::stringstream ssY;
                        //                std::stringstream ssZ;
                        //                //A
                        //                ssX << setprecision(15) << triPair[0][i].x();
                        //                ssY << setprecision(15) << triPair[0][i].y();
                        //                ssZ << setprecision(15) << triPair[0][i].z();
                        //                point = ssX.str() + "," + ssY.str() + "," + ssZ.str();
                        //                ssX.clear();
                        //                ssY.clear();
                        //                ssZ.clear();
                        //                std::string name = "triA_" + to_string(i);
                        //                pos3f->SetAttribute(name.c_str(), point.c_str());
                        //            }
                        //            for (int i = 0; i < 3; ++i)
                        //            {
                        //                std::string point;
                        //                std::stringstream ssX;
                        //                std::stringstream ssY;
                        //                std::stringstream ssZ;
                        //                //A
                        //                ssX << setprecision(15) << triPair[1][i].x();
                        //                ssY << setprecision(15) << triPair[1][i].y();
                        //                ssZ << setprecision(15) << triPair[1][i].z();
                        //                point = ssX.str() + "," + ssY.str() + "," + ssZ.str();
                        //                ssX.clear();
                        //                ssY.clear();
                        //                ssZ.clear();
                        //                std::string name = "triB_" + to_string(i);
                        //                pos3f->SetAttribute(name.c_str(), point.c_str());
                        //            }
                        //            clashpoint->InsertEndChild(pos3f);
                        //        }
                        //        clashtest->InsertEndChild(clashpoint);
                        //    }
                        //}
                        //triInterList.clear();
```





```c
std::tuple<Eigen::Vector3d, double> _getRelationOfTwoSegments(const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB, bool isSquared = true)
{
	//enum RelationTwoSegment
	//{
	//	NearestMiddleMiddle, // edgeA.corss(edgeB)
	//	NearestMiddleVertex, // (vertexA-vertexB).cross(edgesB).cross(edgesB)
	//	NearestVertexVertex, // vertexA-vertexB
	//};
	auto getAxisFromFourPoints = [&]()->std::tuple<Vector3d, double>
	{
		double dmax = DBL_MAX;
		Vector3d direction;
		for (const auto& iterA : segmA)
		{
			for (const auto& iterB : segmB)
			{
				double dtemp = (iterB - iterA).squaredNorm();
				if (dtemp < dmax)
				{
					direction = iterB - iterA;
					dmax = dtemp;
				}
			}
		}
		return { direction, dmax };
	};

	auto getDistanceOfPointAndSegment = [&](const Vector3d& point, const std::array<Vector3d, 2>& segm)->double 
	{
		double dmin = DBL_MAX;
		Eigen::Vector3d direction = (segm[1] - segm[0]);//.normalized(); //unnecessary
		double projection = direction.dot(point);
		if (direction.dot(segm[1]) < projection || projection < direction.dot(segm[0]))
			return dmin; // (point - segm[1]).squaredNorm();
		double k = direction.dot(point - segm[0]) / direction.dot(direction);
		return (segm[0] - point + k * direction).squaredNorm();
	};

	Vector3d vectA = segmA[1] - segmA[0];
	Vector3d vectB = segmB[1] - segmB[0];
	double delta1 = (segmB[0] - segmA[0]).dot(vectA);
	double delta2 = (segmB[0] - segmA[0]).dot(vectB);
	// 2*2 inverse matrix, 1/|M|*(exchange main diagonal and -1 counter-diagonal)
	double deno = -vectA.dot(vectA) * vectB.dot(vectB) + vectA.dot(vectB) * vectB.dot(vectA);//a*d-b*c
	if (deno == 0.0)
	{
		perror("divided by zero.");// cout << "error" << endl;
		return { Vector3d(0,0,0), DBL_MAX };
	}
	double kA = 1 / deno * (-vectB.dot(vectB) * delta1 + vectB.dot(vectA) * delta2);
	double kB = 1 / deno * (-vectA.dot(vectB) * delta1 + vectA.dot(vectA) * delta2);
	// based on position
	double ptA0_segmB_p = std::min((segmA[0] - segmB[0]).squaredNorm(), (segmA[0] - segmB[1]).squaredNorm());
	double ptA1_segmB_p = std::min((segmA[1] - segmB[0]).squaredNorm(), (segmA[1] - segmB[1]).squaredNorm());
	double ptB0_segmA_p = std::min((segmB[0] - segmA[0]).squaredNorm(), (segmB[0] - segmA[1]).squaredNorm());
	double ptB1_segmA_p = std::min((segmB[1] - segmA[0]).squaredNorm(), (segmB[1] - segmA[1]).squaredNorm());
	double ptA0_segmB = getDistanceOfPointAndSegment(segmA[0], segmB);
	double ptA1_segmB = getDistanceOfPointAndSegment(segmA[1], segmB);
	double ptB0_segmA = getDistanceOfPointAndSegment(segmB[0], segmA);
	double ptB1_segmA = getDistanceOfPointAndSegment(segmB[1], segmA);
	Vector3d pointA = segmA[0] + kA * vectA;
	Vector3d pointB = segmB[0] + kB * vectB;
	if (0 <= kA && kA <= 1 && 0 <= kB && kB <= 1)
	{
		//Vector3d pointA = segmA[0] + kA * vectA;
		//Vector3d pointB = segmB[0] + kB * vectB;
		double d = isSquared ? (segmA[0] + kA * vectA - segmB[0] - kB * vectB).squaredNorm() : 
			(segmA[0] + kA * vectA - segmB[0] - kB * vectB).norm();
		return { vectA.cross(vectB), d };
	}

	if ((ptA0_segmB == DBL_MAX && ptA1_segmB == DBL_MAX) && (ptB0_segmA == DBL_MAX && ptB1_segmA == DBL_MAX))
		return getAxisFromFourPoints();
	if (std::min(ptA0_segmB, ptA1_segmB) < std::min(ptB0_segmA, ptB1_segmA))
		return (ptA0_segmB < ptA1_segmB) ?
		tuple<Eigen::Vector3d, double>{ (segmA[0] - segmB[0]).cross(vectB).cross(vectB), ptA0_segmB }:
		tuple<Eigen::Vector3d, double>{ (segmA[1] - segmB[0]).cross(vectB).cross(vectB), ptA1_segmB };
	else
		return (ptB0_segmA < ptB1_segmA) ?
		tuple<Eigen::Vector3d, double>{ (segmB[0] - segmA[0]).cross(vectA).cross(vectA), ptB0_segmA }:
		tuple<Eigen::Vector3d, double>{ (segmB[1] - segmA[0]).cross(vectA).cross(vectA), ptB1_segmA };


	//if (0 <= kA && kA <= 1 )//&& 0 > kB || kB > 1 )
	//{
	//	double dS = _getDistanceOfPointAndSegment(segmB[0], segmA);
	//	double dE = _getDistanceOfPointAndSegment(segmB[1], segmA);
	//	if (dS == DBL_MAX && dE == DBL_MAX)
	//	{
	//		return getAxisFromFourPoints();
	//	}
	//	if (dS < dE)
	//		return { (segmB[0] - segmA[0]).cross(vectA).cross(vectA) , dS };
	//	else
	//		return { (segmB[1] - segmA[0]).cross(vectA).cross(vectA) , dE };
	//}
	//if (0 <= kB && kB <= 1 )//&& 0 > kA || kA > 1)
	//{
	//	double dS = _getDistanceOfPointAndSegment(segmA[0], segmB);
	//	double dE = _getDistanceOfPointAndSegment(segmA[1], segmB);
	//	if (dS == DBL_MAX && dE == DBL_MAX)
	//	{
	//		return getAxisFromFourPoints();
	//	}
	//	if (dS < dE)
	//		return { (segmA[0] - segmB[0]).cross(vectB).cross(vectB) , dS };
	//	else
	//		return { (segmA[1] - segmB[0]).cross(vectB).cross(vectB) , dE };
	//}
	//return getAxisFromFourPoints();
}

//此方案行不通，移动的距离是两直线间的最小距离，点到直线的距离通常会更大
	Vector3d vectZ = vectA.cross(vectB).normalized();
	double d_P2L = (segmA[0] - segmB[0] + vectA.dot(segmB[0] - segmA[0]) / vectA.dot(vectA) * vectA).norm();
	if (vectZ.dot(segmA[0]) < vectZ.dot(segmB[0])) //segmB front
		d_P2L = -d_P2L;
	std::array<Vector3d, 2> segmB_move = { segmB[0] + d_P2L * vectZ, segmB[1] + d_P2L * vectZ };
	//double coplanar = vectA.cross(segmB_move[0] - segmA[0]).dot(segmB_move[1] - segmA[0]);
	if (isTwoSegmentsIntersect(segmA, segmB_move))
        
        

		std::get<0>(_getRelationOfTwoSegments({triA[0], triA[1]}, {triB[0], triB[1]})),
		std::get<0>(_getRelationOfTwoSegments({triA[0], triA[1]}, {triB[1], triB[2]})),
		std::get<0>(_getRelationOfTwoSegments({triA[0], triA[1]}, {triB[2], triB[0]})),
		std::get<0>(_getRelationOfTwoSegments({triA[1], triA[2]}, {triB[0], triB[1]})),
		std::get<0>(_getRelationOfTwoSegments({triA[1], triA[2]}, {triB[1], triB[2]})),
		std::get<0>(_getRelationOfTwoSegments({triA[1], triA[2]}, {triB[2], triB[0]})),
		std::get<0>(_getRelationOfTwoSegments({triA[2], triA[0]}, {triB[0], triB[1]})),
		std::get<0>(_getRelationOfTwoSegments({triA[2], triA[0]}, {triB[1], triB[2]})),
		std::get<0>(_getRelationOfTwoSegments({triA[2], triA[0]}, {triB[2], triB[0]})),
```





```
std::array<Eigen::Vector3d, 2> getTwoTrianglesIntersectPoints(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB)
{
	std::array<Vector3d, 2> res = { gVecNaN , gVecNaN }; // avoid separate
	if (!isTwoTrianglesIntersectSAT(triA, triB))
		return res;
	Vector3d vecSeg;

	std::array<array<Vector3d, 2>, 3> edgesA = { { {triA[0], triA[1]},
												{triA[1], triA[2]},
												{triA[2], triA[0] } } };
	std::array<array<Vector3d, 2>, 3> edgesB = { { {triB[0], triB[1]},
												{ triB[1], triB[2] },
												{ triB[2], triB[0] } } };
	int count = 0;
	for (const auto& edgeA : edgesA)
	{
		double k = _getIntersectOfSegmentAndPlaneINF(edgeA, triB); // refer revise direction of edgeA
		if (DBL_MAX == k) //coplanar
		{
			if (isPointInTriangle(edgeA[0], triB) && isPointInTriangle(edgeA[1], triB))
				return edgeA;
			for (const auto& edgeB : edgesB)
			{
				if (!isTwoSegmentsIntersect(edgeA, edgeB))
					continue;
				double k2 = _getPointOfTwoIntersectSegments(edgeA, edgeB);
				if (DBL_MAX == k2) //collinear
				{
					res[0] = ((edgeA[0] - edgeB[0]).dot(edgeA[0] - edgeB[1])) <= 0.0 ? edgeA[0] : edgeA[1];
					res[1] = ((edgeB[0] - edgeA[0]).dot(edgeB[0] - edgeA[1])) <= 0.0 ? edgeB[0] : edgeB[1];
					return res;
				}
				res[count++] = edgeA[0] + k2 * (vecSeg);
			}
			if (count == 2)
				return res;
		}
		else if (0 <= k && k <= 1)
		{
			Vector3d local = edgeA[0] + k * vecSeg;
			if (!isPointInTriangle(local, triB))
				continue;
			res[count++] = local;
		}
		if (count == 2)
			return res;
	}
	for (const auto& edgeB : edgesB)
	{
		double k = _getIntersectOfSegmentAndPlaneINF(edgeB, triA);
		if (DBL_MAX == k) //coplanar
		{
			if (isPointInTriangle(edgeB[0], triA) && isPointInTriangle(edgeB[1], triA))
				return edgeB;
			for (const auto& edgeA : edgesA)
			{
				if (!isTwoSegmentsIntersect(edgeB, edgeA))
					continue;
				double k2 = _getPointOfTwoIntersectSegments(edgeB, edgeA);
				if (DBL_MAX == k2) //collinear
				{
					res[0] = ((edgeA[0] - edgeB[0]).dot(edgeA[0] - edgeB[1])) <= 0.0 ? edgeA[0] : edgeA[1];
					res[1] = ((edgeB[0] - edgeA[0]).dot(edgeB[0] - edgeA[1])) <= 0.0 ? edgeB[0] : edgeB[1];
					return res;
				}
				res[count++] = edgeB[0] + k * (vecSeg);
			}
			if (count == 2)
				return res;
		}
		else if (0 <= k && k <= 1)
		{
			Vector3d local = edgeB[0] + k * vecSeg;
			if (!isPointInTriangle(local, triA))
				continue;
			res[count++] = local;
		}
		if (count == 2)
			return res;
	}
	return res;
}


```





轨道线性

```py
import sys
import os
mypath = r'C:\Users\Aking\source\repos\bimbase\Bin\Release\PythonScript\python-3.7.9-embed-amd64\Lib\site-packages'  # fixed path
sys.path.append(os.path.join(os.path.dirname(__file__), mypath))

txtname=r'C:\Users\Aking\source\repos\bimbase\Bin\Release\PythonScript\python-3.7.9-embed-amd64\Lib\site-packages\铁设城轨一期\线位里程坐标.txt'
a = np.loadtxt(txtname, skiprows=1, usecols=(1, 2, 3))
length = len(a) #204
length=2

#扣件
        sec = rotate(Vec3(0,1,0),pi/2) * Section(scale(R_button) * Arc(pi),scale(R_button) * rotz(pi)*Arc(pi))
        # Button = sweep_stere(sec,Line(ChooseLine)).color(122/255,255/255,212/255,1)
        # Button.smooth = True
        # create_geometry(sec)
        # create_geometry(Line(ChooseLine))
        sec=trans(ChooseLine[0])*sec
        # Button = trans(-ChooseLine[0])*Sweep(sec,Line(SplineCurve(ChooseLine,0,2)))
        Button = trans(-ChooseLine[0])*Sweep(sec,Line(ChooseLine)).color(122/255,255/255,212/255,1)

        # # 扣件管 FilletPipe
        # Button = FilletPipe(ChooseLine, [
        #                     R_button]*len(ChooseLine), R_button).color(122/255, 255/255, 212/255, 1)
        # Button = FilletPipe_Sweep(ChooseLine, [
        #                     R_button]*len(ChooseLine), R_button).color(122/255, 255/255, 212/255, 1)
        # create_geometry(Button)
        # # 扣件管位置调整
        # Button = translate(0, -50, self['螺扭高程']+37) * Button.color(screwcolor)


```



阵列参数

10*20，100000-50000
