#include "pch.h"
using namespace std;

class TriangleVec
{
public:
	Vec3 m_p0;
	Vec3 m_p1;
	Vec3 m_p2;
};

/*
一般来说，可以用两个方法来检测三角形是否相交:
投影算法:
利用投影理论，可以将三维空间中的三角形，投影到任意一个平
面上，观察其投影之后，是否出现重叠，从而推断是否有相交;
三角剖分算法:
将两个三角形进行三角剖分，比较两个三角形的边、顶点是否有
交点，如有交点，则两个三角形可以认定为相交;

*/

enum class COLLISION :int
{
	T_SEPA,
	T_INTER,
	T_TANG,

};

COLLISION is_two_triangle_intersec(const TriangleVec& triA, const TriangleVec& triB)
{
	return {};
}