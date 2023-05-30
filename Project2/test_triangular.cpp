#include "pch.h"
using namespace para;
using namespace psykronix;
using Eigen::Matrix3d;
using Eigen::Vector3d;

Matrix4d psykronix::rotx(double theta)
{
    Matrix4d R;
    R << 1, 0, 0, 0,
        0, cos(theta), -sin(theta), 0,
        0, sin(theta), cos(theta), 0,
		0, 0, 0, 1;
    return R;
}

Matrix4d psykronix::roty(double theta)
{
    Matrix4d R;
    R << cos(theta), 0, sin(theta), 0,
        0, 1, 0, 0,
        -sin(theta), 0, cos(theta), 0,
        0, 0, 0, 1;
    return R;
}

Matrix4d psykronix::rotz(double theta)
{
    Matrix4d R;
    R << cos(theta), -sin(theta), 0, 0,
        sin(theta), cos(theta), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return R;
}

Matrix4d psykronix::translate(double x, double y, double z /*= 0.0*/)
{
    Matrix4d T;
    T << 1, 0, 0, x,
        0, 1, 0, y,
        0, 0, 1, z,
        0, 0, 0, 1;
    return T;
}

Matrix4d psykronix::translate(const Eigen::Vector3d& vec)
{
    return psykronix::translate(vec.x(), vec.y(), vec.z());
}

Matrix4d psykronix::scale(double x, double y, double z /*= 0.0*/)
{
    Matrix4d T;
    T << x, 0, 0, 0,
        0, y, 0, 0,
        0, 0, z, 0,
        0, 0, 0, 1;
    return T;
}

std::array<Eigen::Vector3d, 3> psykronix::operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector3d, 3>& tri)
{
    std::array<Eigen::Vector3d, 3> res;
	for (int i = 0; i < 3; i++)
    {
        Vector4d vec4 = mat * (tri[i].homogeneous()); //Vector4d(tri[i].x(), tri[i].y(), tri[i].z(), 1);
        res[i] = vec4.hnormalized(); // Vector3d(vec4.x(), vec4.y(), vec4.z());
    }
    return res;
}


Matrix4d psykronix::rotate(const Eigen::Vector3d& axis /*= { 0, 0, 1 }*/, double theta /*= 0.0*/)
{
    Quaterniond q = Quaterniond(AngleAxisd(theta, axis));
    Matrix3d R = q.toRotationMatrix();
    Matrix4d mat4d = Matrix4d::Identity();
    mat4d.block<3, 3>(0, 0) = R; //block<rows,cols>(row_index,col_index) <>is child size, () is begin index
    //mat4d.block<1, 3>(3, 0) << 0, 0, 0; 
    //mat4d(3, 3) = 1; 
    return mat4d;
}

//--------------------------------------------------------------------------------------------------
//  triangle
//--------------------------------------------------------------------------------------------------




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


bool _isPointInTriangular(const Vector3d& point, const std::array<Vector3d, 3>& trigon)
{
	//if (abs(((trigon[1] - trigon[0]) ^ (trigon[2] - trigon[0])) * (point - trigon[0])) > PL_Surface) //not coplanar
 //       return false;
	return ((trigon[1] - trigon[0]).cross(point - trigon[0])).dot((trigon[2] - trigon[0]).cross(point - trigon[0])) < PL_Surface &&
		((trigon[0] - trigon[1]).cross(point - trigon[1])).dot((trigon[2] - trigon[1]).cross(point - trigon[1])) < PL_Surface; // (x)*(x)<0 || abs()<PLA
}

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

//
//bool _isTwoTriangularIntersection1(const std::array<BPParaVec, 3>& triL, const std::array<BPParaVec, 3>& triR)
//{
//	BPParaVec pL0 = triL[0];
//	BPParaVec pL1 = triL[1];
//	BPParaVec pL2 = triL[2];
//	BPParaVec pR0 = triR[0];
//	BPParaVec pR1 = triR[1];
//	BPParaVec pR2 = triR[2];
//
//	//cout << "pL0: " << pL0.x() << "," << pL0.y() << "," << pL0.z() << endl;
//	//cout << "pR0: " << pR0.x() << "," << pR0.y() << "," << pR0.z() << endl;
//
//	// 1, check legal triangular
//
//
//	//BPParaVec veczL = (triL[1] - triL[0]) ^ (triL[2] - triL[0]);
//	BPParaVec veczL = (pL1 - pL0) ^ (pL2 - pL0);
//	BPParaVec veczR = (pR1 - pR0) ^ (pR2 - pR0);
//
//	// plane parallel
//	//if (isParallel(veczL, veczR))
//	//{
//
//	//}
//	// 
//	// 
//	//through the triangular plane
//	// edge of triR cross plane triL
//	double dotR2L_A = (veczL * (pL0 - pR0)) * (veczL * (pL1 - pR0));
//	double dotR2L_B = (veczL * (pL1 - pR0)) * (veczL * (pL2 - pR0));
//	double dotR2L_C = (veczL * (pL2 - pR0)) * (veczL * (pL0 - pR0));
//	bool acrossR2L_A = (dotR2L_A < 0.0 || abs(dotR2L_A) < PL_Surface);
//	bool acrossR2L_B = (dotR2L_B < 0.0 || abs(dotR2L_B) < PL_Surface);
//	bool acrossR2L_C = (dotR2L_C < 0.0 || abs(dotR2L_C) < PL_Surface);
//	if (!acrossR2L_A && !acrossR2L_B && !acrossR2L_C)
//		return false;
//
//	double dotL2R_A = (veczR * (pR0 - pL0)) * (veczR * (pR1 - pL0));
//	double dotL2R_B = (veczR * (pR1 - pL0)) * (veczR * (pR2 - pL0));
//	double dotL2R_C = (veczR * (pR2 - pL0)) * (veczR * (pR0 - pL0));
//	bool acrossL2R_A = (dotL2R_A < 0.0 || abs(dotL2R_A) < PL_Surface);
//	bool acrossL2R_B = (dotL2R_B < 0.0 || abs(dotL2R_B) < PL_Surface);
//	bool acrossL2R_C = (dotL2R_C < 0.0 || abs(dotL2R_C) < PL_Surface);
//	if (!acrossL2R_A && !acrossL2R_B && !acrossL2R_C)
//		return false;
//
//	// special coplanar
//
//
//	if (acrossR2L_A) // first filter
//	{
//		double div = (veczL * (pL0 - pL1));
//		if (abs(div) < PL_Surface)
//		{
//			if (_isPointInTriangular(pL0, triL) || _isPointInTriangular(pL1, triL))
//				return true;
//		}
//		double k = (veczL * (pL0 - pR0)) / div;
//		BPParaVec locate = pL0 + k * (pL1 - pL0);  // paramater formula vector
//		if (_isPointInTriangular(locate, triL))
//			return true;
//	}
//	if (acrossR2L_B) // first filter
//	{
//		double div = (veczL * (pL1 - pL2));
//		if (abs(div) < PL_Surface)
//		{
//			if (_isPointInTriangular(pL1, triL) || _isPointInTriangular(pL2, triL))
//				return true;
//		}
//		double k = (veczL * (pL1 - pR0)) / div;
//		BPParaVec locate = pL1 + k * (pL2 - pL1);  // paramater formula vector
//		if (_isPointInTriangular(locate, triL))
//			return true;
//	}
//	if (acrossR2L_C) // first filter
//	{
//		double div = (veczL * (pL2 - pL0));
//		if (abs(div) < PL_Surface)
//		{
//			if (_isPointInTriangular(pL2, triL) || _isPointInTriangular(pL0, triL))
//				return true;
//		}
//		double k = (veczL * (pL2 - pR0)) / div;
//		BPParaVec locate = pL2 + k * (pL0 - pL2);  // paramater formula vector
//		if (_isPointInTriangular(locate, triL))
//			return true;
//	}
//	return false;
//}
//


bool isTwoTrianglesIntersection(const std::array<Vector3d, 3>& triL, const std::array<Vector3d, 3>& triR)
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
    Vector3d veczL = (triL[1] - triL[0]).cross(triL[2] - triL[0]);
    bool acrossR2L_A = (veczL.dot(triR[0] - triL[0])) * (veczL.dot(triR[1] - triL[0])) < PL_Surface; //include point-on-plane
	bool acrossR2L_B = (veczL.dot(triR[1] - triL[0])) * (veczL.dot(triR[2] - triL[0])) < PL_Surface;
	bool acrossR2L_C = (veczL.dot(triR[2] - triL[0])) * (veczL.dot(triR[0] - triL[0])) < PL_Surface;
	if (!acrossR2L_A && !acrossR2L_B && !acrossR2L_C)
		return false;

    // left through right plane
 //   double dotL2R_A = (veczR.dot(triR[0] - triL[0])) * (veczR.dot(triR[1] - triL[0]));
	//double dotL2R_B = (veczR.dot(triR[1] - triL[0])) * (veczR.dot(triR[2] - triL[0]));
	//double dotL2R_C = (veczR.dot(triR[2] - triL[0])) * (veczR.dot(triR[0] - triL[0]));
    Vector3d veczR = (triR[1] - triR[0]).cross(triR[2] - triR[0]);
    bool acrossL2R_A = (veczR.dot(triR[0] - triL[0])) * (veczR.dot(triR[1] - triL[0])) < PL_Surface;
	bool acrossL2R_B = (veczR.dot(triR[1] - triL[0])) * (veczR.dot(triR[2] - triL[0])) < PL_Surface;
	bool acrossL2R_C = (veczR.dot(triR[2] - triL[0])) * (veczR.dot(triR[0] - triL[0])) < PL_Surface;
	if (!acrossL2R_A && !acrossL2R_B && !acrossL2R_C)
		return false;

	if (acrossR2L_A) // first filter
	{
        // k = dot(vecZ, p0-pO)/dot(vecZ, p0-p1); p = p0 + k * (p1 - p0);
		//double deno = (veczL.dot(triR[0] - triR[1]));
		//if (abs(deno) < PL_Surface)
		//{
		//	if (_isPointInTriangular(triR[0], triL) || _isPointInTriangular(triR[1], triL))
		//		return true;
		//}
		//else
		//{
			Vector3d locate = triR[0] + ((veczL.dot(triR[0] - triL[0])) / (veczL.dot(triR[0] - triR[1]))) * (triR[1] - triR[0]); 
			if (_isPointInTriangular(locate, triL))
				return true;
		//}
	}
	if (acrossR2L_B) // first filter
	{
		//double deno = (veczL.dot(triR[1] - triR[2]));
		//if (abs(deno) < PL_Surface)
		//{
		//	if (_isPointInTriangular(triR[1], triL) || _isPointInTriangular(triR[2], triL))
		//		return true;
		//}
		//else
		//{
            Vector3d locate = triR[1] + ((veczL.dot(triR[1] - triL[0])) / (veczL.dot(triR[1] - triR[2]))) * (triR[2] - triR[1]); 
			if (_isPointInTriangular(locate, triL))
				return true;
		//}
	}
	if (acrossR2L_C) // first filter
	{
		//double deno = (veczL.dot(triL[2] - triL[0]));
		//if (abs(deno) < PL_Surface)
		//{
		//	if (_isPointInTriangular(triL[2], triL) || _isPointInTriangular(triL[0], triL))
		//		return true;
		//}
		//else
		//{
            Vector3d locate = triR[2] + ((veczL.dot(triR[2] - triL[0])) / (veczL.dot(triR[2] - triR[0]))) * (triR[0] - triR[2]);
			if (_isPointInTriangular(locate, triL))
				return true;
		//}
	}
	return false;
}



bool TriangularIntersectionTest(const std::array<Eigen::Vector3d, 3>& T1, const std::array<Eigen::Vector3d, 3>& T2)
{
    //const Vector3d& A1 = *(const Vector3d*)(&T1.at(0));
    //const Vector3d& B1 = *(const Vector3d*)(&T1.at(1));
    //const Vector3d& C1 = *(const Vector3d*)(&T1.at(2));
    //const Vector3d& A2 = *(const Vector3d*)(&T2.at(0));
    //const Vector3d& B2 = *(const Vector3d*)(&T2.at(1));
    //const Vector3d& C2 = *(const Vector3d*)(&T2.at(2));
    const Vector3d& A1 = T1.at(0);
    const Vector3d& B1 = T1.at(1);
    const Vector3d& C1 = T1.at(2);
    const Vector3d& A2 = T2.at(0);
    const Vector3d& B2 = T2.at(1);
    const Vector3d& C2 = T2.at(2);
    // T1
    Vector3d A1B1 = B1 - A1;
    Vector3d B1C1 = C1 - B1;
    Vector3d C1A1 = A1 - C1;
    // A1 T2
    Vector3d A1A2 = A2 - A1;
    Vector3d A1B2 = B2 - A1;
    Vector3d A1C2 = C2 - A1;
    // B1 T2
    Vector3d B1A2 = A2 - B1;
    Vector3d B1B2 = B2 - B1;
    Vector3d B1C2 = C2 - B1;
    // C1 T2
    Vector3d C1A2 = A2 - C1;
    Vector3d C1B2 = B2 - C1;
    Vector3d C1C2 = C2 - C1;

    // T2
    Vector3d A2B2 = B2 - A2;
    Vector3d B2C2 = C2 - A2;
    Vector3d C2A2 = A2 - C2;
    // A2 T1
    Vector3d A2A1 = A1 - A2;
    Vector3d A2B1 = B1 - A2;
    Vector3d A2C1 = C1 - A2;
    // B2 T1
    Vector3d B2A1 = A1 - B2;
    Vector3d B2B1 = B1 - B2;
    Vector3d B2C1 = C1 - B2;
    // C2 T1
    Vector3d C2A1 = A1 - C2;
    Vector3d C2B1 = B1 - C2;
    Vector3d C2C1 = C1 - C2;

    // n1
    Vector3d n1 = A1B1.cross(B1C1);
    double n1_t1 = A1A2.dot(n1);
    double n1_t2 = A1B2.dot(n1);
    double n1_t3 = A1C2.dot(n1);

    // n2
    Vector3d n2 = A2B2.cross(B2C2);
    double n2_t1 = A2A1.dot(n2);
    double n2_t2 = A2B1.dot(n2);
    double n2_t3 = A2C1.dot(n2);
    unsigned int n1_ts = (n1_t1 > 0.0 ? 18 : (n1_t1 < 0.0 ? 0 : 9)) + (n1_t2 > 0.0 ? 6 : (n1_t2 < 0.0 ? 0 : 3)) + (n1_t3 > 0.0 ? 2 : (n1_t3 < 0.0 ? 0 : 1));
    unsigned int n2_ts = (n2_t1 > 0.0 ? 18 : (n2_t1 < 0.0 ? 0 : 9)) + (n2_t2 > 0.0 ? 6 : (n2_t2 < 0.0 ? 0 : 3)) + (n2_t3 > 0.0 ? 2 : (n2_t3 < 0.0 ? 0 : 1));
    if (n1_ts == 0 || n2_ts == 0 || n1_ts == 26 || n2_ts == 26)
        return false;   // +*/<  72,48,0,18
    // 共面
    if (n1_ts == 13 || n2_ts == 13)
    {
        Vector3d A1B1_outboard = A1B1.cross(n1);
        double A2_k1 = A1A2.dot(A1B1_outboard);
        double B2_k1 = A1B2.dot(A1B1_outboard);
        double C2_k1 = A1C2.dot(A1B1_outboard);
        if (A2_k1 > 0 && B2_k1 > 0 && C2_k1 > 0)
            return false; // +*/<  84,72,0,21
        Vector3d B1C1_outboard = B1C1.cross(n1);
        double A2_k2 = B1A2.dot(B1C1_outboard);
        double B2_k2 = B1B2.dot(B1C1_outboard);
        double C2_k2 = B1C2.dot(B1C1_outboard);
        if (A2_k2 > 0 && B2_k2 > 0 && C2_k2 > 0)
            return false; // +*/<  96,96,0,24
        Vector3d C1A1_outboard = C1A1.cross(n1);
        double A2_k3 = C1A2.dot(C1A1_outboard);
        double B2_k3 = C1B2.dot(C1A1_outboard);
        double C2_k3 = C1C2.dot(C1A1_outboard);
        if (A2_k3 > 0 && B2_k3 > 0 && C2_k3 > 0)
            return false; // +*/<  108,120,0,27 
        Vector3d A2B2_outboard = A2B2.cross(n2);
        double A1_k1 = A2A1.dot(A2B2_outboard);
        double B1_k1 = A2B1.dot(A2B2_outboard);
        double C1_k1 = A2C1.dot(A2B2_outboard);
        if (A1_k1 > 0 && B1_k1 > 0 && C1_k1 > 0)
            return false; // +*/<  120,144,0,30
        Vector3d B2C2_outboard = B2C2.cross(n2);
        double A1_k2 = B2A1.dot(B2C2_outboard);
        double B1_k2 = B2B1.dot(B2C2_outboard);
        double C1_k2 = B2C1.dot(B2C2_outboard);
        if (A1_k1 > 0 && B1_k1 > 0 && C1_k1 > 0)
            return false; // +*/<  132,168,0,33
        Vector3d C2A2_outboard = C2A2.cross(n2);
        double A1_k3 = C2A1.dot(C2A2_outboard);
        double B1_k3 = C2B1.dot(C2A2_outboard);
        double C1_k3 = C2C1.dot(C2A2_outboard);
        if (A1_k1 > 0 && B1_k1 > 0 && C1_k1 > 0)
            return false; // +*/<  144,192,0,36
        return true; // +*/<  144,192,0,18
    }
    Vector3d O1, P1, Q1, O2, P2, Q2;
    switch (n1_ts)
    {
    case 9:
    case 17:
    {
        // A2;
        Vector3d A1B1_outboard = A1B1.cross(n1);
        Vector3d B1C1_outboard = B1C1.cross(n1);
        Vector3d C1A1_outboard = C1A1.cross(n1);
        double k1 = A1A2.dot(A1B1_outboard);
        double k2 = B1A2.dot(B1C1_outboard);
        double k3 = C1A2.dot(C1A1_outboard);
        return (k1 <= 0 && k2 <= 0 && k3 <= 0); // +*/<  90,84,0,21
    }
    case 3:
    case 23:
    {
        // B2;
        Vector3d A1B1_outboard = A1B1.cross(n1);
        Vector3d B1C1_outboard = B1C1.cross(n1);
        Vector3d C1A1_outboard = C1A1.cross(n1);
        double k1 = A1B2.dot(A1B1_outboard);
        double k2 = B1B2.dot(B1C1_outboard);
        double k3 = C1B2.dot(C1A1_outboard);
        return (k1 <= 0 && k2 <= 0 && k3 <= 0); // +*/<  90,84,0,21
    }
    case 1:
    case 25:
    {
        // C2;
        Vector3d A1B1_outboard = A1B1.cross(n1);
        Vector3d B1C1_outboard = B1C1.cross(n1);
        Vector3d C1A1_outboard = C1A1.cross(n1);
        double k1 = A1C2.dot(A1B1_outboard);
        double k2 = B1C2.dot(B1C1_outboard);
        double k3 = C1C2.dot(C1A1_outboard);
        return (k1 <= 0 && k2 <= 0 && k3 <= 0); // +*/<  90,84,0,21
    }
    case 12:
    case 14:
    {
        // A2B2;
        Vector3d A2B2_outboard = A2B2.cross(n1);
        double kA = A2A1.dot(A2B2_outboard);
        double kB = A2B1.dot(A2B2_outboard);
        double kC = A2C1.dot(A2B2_outboard);
        return !((kA > 0 && kB > 0 && kC > 0) || (kA < 0 && kB < 0 && kC < 0));  // +*/<  84,72,0,24
    }
    case 4:
    case 22:
    {
        // B2C2;
        Vector3d B2C2_outboard = B2C2.cross(n1);
        double kA = B2A1.dot(B2C2_outboard);
        double kB = B2B1.dot(B2C2_outboard);
        double kC = B2C1.dot(B2C2_outboard);
        return !((kA > 0 && kB > 0 && kC > 0) || (kA < 0 && kB < 0 && kC < 0)); // +*/<  84,72,0,24
    }
    case 10:
    case 16:
    {
        // C2A2;
        Vector3d C2A2_outboard = C2A2.cross(n1);
        double kA = C2A1.dot(C2A2_outboard);
        double kB = C2B1.dot(C2A2_outboard);
        double kC = C2C1.dot(C2A2_outboard);
        return !((kA > 0 && kB > 0 && kC > 0) || (kA < 0 && kB < 0 && kC < 0)); // +*/<  84,72,0,24
    }
    case 7:     // C2;      A2B2;
    case 19:    // C2;      A2B2;
    case 8:     // C2A2;    A2B2;
    case 18:    // C2A2;    A2B2;
        O2 = A2;
        P2 = B2;
        Q2 = C2;
        break;
    case 11:    // A2;      B2C2;
    case 15:    // A2;      B2C2;
    case 6:     // A2B2;    B2C2;
    case 20:    // A2B2;    B2C2;
        O2 = B2;
        P2 = A2;
        Q2 = C2;
        break;
    case 5:     // B2;      C2A2;
    case 21:    // B2;      C2A2;
    case 2:     // B2C2;    C2A2;
    case 24:    // B2C2;    C2A2;
        O2 = C2;
        P2 = B2;
        Q2 = A2;
        break;
    }
    switch (n2_ts)
    {
    case 9:
    case 17:
    {
        // A1;
        Vector3d A2B2_outboard = A2B2.cross(n2);
        Vector3d B2C2_outboard = B2C2.cross(n2);
        Vector3d C2A2_outboard = C2A2.cross(n2);
        double k1 = A2A1.dot(A2B2_outboard);
        double k2 = B2A1.dot(B2C2_outboard);
        double k3 = C2A1.dot(C2A2_outboard);
        return (k1 <= 0 && k2 <= 0 && k3 <= 0); // +*/<  90,84,0,21
    }
    case 3:
    case 23:
    {
        // B1;
        Vector3d A2B2_outboard = A2B2.cross(n2);
        Vector3d B2C2_outboard = B2C2.cross(n2);
        Vector3d C2A2_outboard = C2A2.cross(n2);
        double k1 = A2B1.dot(A2B2_outboard);
        double k2 = B2B1.dot(B2C2_outboard);
        double k3 = C2B1.dot(C2A2_outboard);
        return (k1 <= 0 && k2 <= 0 && k3 <= 0); // +*/<  90,84,0,21
    }
    case 1:
    case 25:
    {
        // C1;
        Vector3d A2B2_outboard = A1B2.cross(n2);
        Vector3d B2C2_outboard = B1C2.cross(n2);
        Vector3d C2A2_outboard = C1A2.cross(n2);
        double k1 = A2C1.dot(A2B2_outboard);
        double k2 = B2C1.dot(B2C2_outboard);
        double k3 = C2C1.dot(C2A2_outboard);
        return (k1 <= 0 && k2 <= 0 && k3 <= 0); // +*/<  90,84,0,21
    }
    case 12:
    case 14:
    {
        // A1B1;
        Vector3d A2B2_outboard = A2B2.cross(n1);
        double kA = A2A1.dot(A2B2_outboard);
        double kB = A2B1.dot(A2B2_outboard);
        double kC = A2C1.dot(A2B2_outboard);
        return !((kA > 0 && kB > 0 && kC > 0) || (kA < 0 && kB < 0 && kC < 0)); // +*/<  84,72,0,24
    }
    case 4:
    case 22:
    {
        // B1C1;
        Vector3d B2C2_outboard = B2C2.cross(n1);
        double kA = B2A1.dot(B2C2_outboard);
        double kB = B2B1.dot(B2C2_outboard);
        double kC = B2C1.dot(B2C2_outboard);
        return !((kA > 0 && kB > 0 && kC > 0) || (kA < 0 && kB < 0 && kC < 0)); // +*/<  84,72,0,24
    }
    case 10:
    case 16:
    {
        // C1A1;
        Vector3d C2A2_outboard = C2A2.cross(n1);
        double kA = C2A1.dot(C2A2_outboard);
        double kB = C2B1.dot(C2A2_outboard);
        double kC = C2C1.dot(C2A2_outboard);
        return !((kA > 0 && kB > 0 && kC > 0) || (kA < 0 && kB < 0 && kC < 0)); // +*/<  84,72,0,24
    }
    case 7:     // C1;      A1B1;
    case 19:    // C1;      A1B1;
    case 8:     // C1A1;    A1B1;
    case 18:    // C1A1;    A1B1;
        O1 = A1;
        P1 = B1;
        Q1 = C1;
        break;
    case 11:    // A1;      B1C1;
    case 15:    // A1;      B1C1;
    case 6:     // A1B1;    B1C1;
    case 20:    // A1B1;    B1C1;
        O1 = B1;
        P1 = A1;
        Q1 = C1;
        break;
    case 5:     // B1;      C1A1;
    case 21:    // B1;      C1A1;
    case 2:     // B1C1;    C1A2;
    case 24:    // B1C1;    C1A2;
        O1 = C1;
        P1 = B1;
        Q1 = A1;
        break;
    }
    Vector3d O1O2 = O2 - O1;
    Vector3d O1P2 = P2 - O1;
    Vector3d O1Q2 = Q2 - O1;
    Vector3d O1P1 = P1 - O1;
    Vector3d O1Q1 = Q1 - O1;

    Vector3d NP = O1P1.cross(O1O2);
    double kpk = NP.dot(O1Q1);
    double kpp = NP.dot(O1P2);
    double kpq = NP.dot(O1Q2);
    if ((kpk > 0 && kpp < 0 && kpq < 0) || (kpk < 0 && kpp > 0 && kpq > 0))
        return false; // +*/<  89,74,0,24

    Vector3d NQ = O1Q1.cross(O1O2);
    double kqk = NQ.dot(O1P1);
    double kqp = NQ.dot(O1P2);
    double kqq = NQ.dot(O1Q2);
    if ((kqk > 0 && kqp < 0 && kqq < 0) || (kqk < 0 && kqp > 0 && kqq > 0))
        return false; // +*/<  101,98,0,30
    return true;
}

