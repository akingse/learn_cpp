#include "pch.h"
#include <limits>
using namespace para;
using namespace Eigen;
using namespace psykronix;
static const float eps = 1e-6;


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

std::array<Eigen::Vector3f, 3> psykronix::operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector3f, 3>& tri)
{
    std::array<Eigen::Vector3f, 3> res;
    Matrix4f mat_float = mat.cast<float>();
    for (int i = 0; i < 3; i++)
    {
        Vector4f vec4 = mat_float * (tri[i].homogeneous()); //Vector4d(tri[i].x(), tri[i].y(), tri[i].z(), 1);
		res[i] = vec4.hnormalized(); //Vector3f(vec4.x(), vec4.y(), vec4.z()); //
    }
    return res;
}

//std::array<Eigen::Vector3f, 3> psykronix::operator*(const Eigen::Matrix4f& mat, const std::array<Eigen::Vector3f, 3>& tri)
//{
//    std::array<Eigen::Vector3f, 3> res;
//    for (int i = 0; i < 3; i++)
//    {
//        Vector4f vec4 = mat * (tri[i].homogeneous()); //Vector4d(tri[i].x(), tri[i].y(), tri[i].z(), 1);
//		res[i] = vec4.hnormalized(); //Vector3f(vec4.x(), vec4.y(), vec4.z()); //
//    }
//    return res;
//}


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


bool _isPointInTriangle(const Vector3f& point, const std::array<Vector3f, 3>& trigon) //must coplanar
{
	//if (abs(((trigon[1] - trigon[0]) ^ (trigon[2] - trigon[0])) * (point - trigon[0])) > PL_Surface) //not coplanar
    //    return false;
	return ((trigon[1] - trigon[0]).cross(point - trigon[0])).dot((trigon[2] - trigon[0]).cross(point - trigon[0])) < eps &&
		((trigon[0] - trigon[1]).cross(point - trigon[1])).dot((trigon[2] - trigon[1]).cross(point - trigon[1])) < eps; // (x)*(x)<0 || abs()<PLA
}
bool _isPointInTriangle(const Vector3d& point, const std::array<Vector3d, 3>& trigon) //must coplanar
{
    return ((trigon[1] - trigon[0]).cross(point - trigon[0])).dot((trigon[2] - trigon[0]).cross(point - trigon[0])) < eps &&
        ((trigon[0] - trigon[1]).cross(point - trigon[1])).dot((trigon[2] - trigon[1]).cross(point - trigon[1])) < eps; // (x)*(x)<0 || abs()<PLA
}

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
    // ¹²Ãæ
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

bool TriangularIntersectionTest(const std::array<Eigen::Vector3f, 3>& T1, const std::array<Eigen::Vector3f, 3>& T2)
{
    std::array<Vector3d, 3>  triA = { T1[0].cast<double>(), T1[1].cast<double>(), T1[2].cast<double>()};
    std::array<Vector3d, 3>  triB = { T2[0].cast<double>(), T2[1].cast<double>(), T2[2].cast<double>()};
    return TriangularIntersectionTest(triA, triB);
}


bool _isEdgeCrossTriangle(const std::array<Vector3d, 2>& segment, const std::array<Vector3d, 3>& trigon) // must coplanar
{
    // double straddle experiment x3
    bool edgea2segm = ((segment[0] - trigon[0]).cross(trigon[1] - trigon[0])).dot((segment[1] - trigon[0]).cross(trigon[1] - trigon[0])) < eps;
    bool segm2edgea = ((trigon[1] - segment[0]).cross(segment[1] - segment[0])).dot((trigon[0] - segment[0]).cross(segment[1] - segment[0])) < eps;
    if (edgea2segm && segm2edgea)
        return true;
    bool edgeb2segm = ((segment[0] - trigon[1]).cross(trigon[2] - trigon[1])).dot((segment[1] - trigon[1]).cross(trigon[2] - trigon[1])) < eps;
    bool segm2edgeb = ((trigon[2] - segment[0]).cross(segment[1] - segment[0])).dot((trigon[1] - segment[0]).cross(segment[1] - segment[0])) < eps;
    if (edgeb2segm && segm2edgeb)
        return true;
    bool edgec2segm = ((segment[0] - trigon[2]).cross(trigon[0] - trigon[2])).dot((segment[1] - trigon[2]).cross(trigon[0] - trigon[2])) < eps;
    bool segm2edgec = ((trigon[0] - segment[0]).cross(segment[1] - segment[0])).dot((trigon[2] - segment[0]).cross(segment[1] - segment[0])) < eps;
    if (edgec2segm && segm2edgec)
        return true;
    return false;
}

// must cross plane
bool _isSegmentCrossTriangleSurface(const std::array<Vector3d, 2>& segment, const std::array<Vector3d, 3>& trigon) //start point outof plane
{
    // compare angle of normal-vector
    bool isLeftSa = (segment[1] - segment[0]).dot((trigon[0] - segment[0]).cross(trigon[1] - segment[0])) < eps;
    bool isLeftSb = (segment[1] - segment[0]).dot((trigon[1] - segment[0]).cross(trigon[2] - segment[0])) < eps;
    bool isLeftSc = (segment[1] - segment[0]).dot((trigon[2] - segment[0]).cross(trigon[0] - segment[0])) < eps;
    if ((isLeftSa && isLeftSb && isLeftSc) || (!isLeftSa && !isLeftSb && !isLeftSc))
        return true;
    //bool isLeftEa = (segment[0] - segment[1]).dot((trigon[0] - segment[1]).cross(trigon[1] - segment[1])) < eps;
    //bool isLeftEb = (segment[0] - segment[1]).dot((trigon[1] - segment[1]).cross(trigon[2] - segment[1])) < eps;
    //bool isLeftEc = (segment[0] - segment[1]).dot((trigon[2] - segment[1]).cross(trigon[0] - segment[1])) < eps;
    //if ((isLeftEa && isLeftEb && isLeftEc) || (!isLeftEa && !isLeftEb && !isLeftEc))
    //    return true;
    return false;
}


bool isTwoTrianglesIntersection1(const std::array<Vector3d, 3>& triL, const std::array<Vector3d, 3>& triR)
{
    // right edge through left plane
    Vector3d veczL = (triL[1] - triL[0]).cross(triL[2] - triL[0]);
    bool acrossR2L_A = (veczL.dot(triR[0] - triL[0])) * (veczL.dot(triR[1] - triL[0])) < eps; //include point-on-plane
    bool acrossR2L_B = (veczL.dot(triR[1] - triL[0])) * (veczL.dot(triR[2] - triL[0])) < eps;
    bool acrossR2L_C = (veczL.dot(triR[2] - triL[0])) * (veczL.dot(triR[0] - triL[0])) < eps;
    if (!acrossR2L_A && !acrossR2L_B && !acrossR2L_C)
        return false;

    // left edge through right plane
    Vector3d veczR = (triR[1] - triR[0]).cross(triR[2] - triR[0]);
    bool acrossL2R_A = (veczR.dot(triR[0] - triL[0])) * (veczR.dot(triR[1] - triL[0])) < eps;
    bool acrossL2R_B = (veczR.dot(triR[1] - triL[0])) * (veczR.dot(triR[2] - triL[0])) < eps;
    bool acrossL2R_C = (veczR.dot(triR[2] - triL[0])) * (veczR.dot(triR[0] - triL[0])) < eps;
    if (!acrossL2R_A && !acrossL2R_B && !acrossL2R_C)
        return false;

    if (acrossR2L_A) // first filter
    {
        double deno = veczL.dot(triR[0] - triR[1]);
        if (fabs(deno) < eps) // perpendi to veczL
        {
            if (_isEdgeCrossTriangle({ triR[0], triR[1] }, triL)) //segment on plane
                return true;
        }
        else
        {
            Vector3d point = triR[0] + (veczL.dot(triR[0] - triL[0]) / deno) * (triR[1] - triR[0]);
            if (_isPointInTriangle(point, triL))
                return true;
        }
    }
    if (acrossR2L_B) // first filter
    {
        double deno = (veczL.dot(triR[1] - triR[2]));
        if (fabs(deno) < eps) // perpendi to veczL
        {
            if (_isEdgeCrossTriangle({ triR[1], triR[2] }, triL)) //segment on plane
                return true;
        }
        else
        {
            Vector3d point = triR[1] + (veczL.dot(triR[1] - triL[0]) / deno) * (triR[2] - triR[1]);
            if (_isPointInTriangle(point, triL))
                return true;
        }
    }
    if (acrossR2L_C) // first filter
    {
        double deno = (veczL.dot(triR[2] - triR[0]));
        if (fabs(deno) < eps) // perpendi to veczL
        {
            if (_isEdgeCrossTriangle({ triR[2], triR[0] }, triL)) //segment on plane
                return true;
        }
        else
        {
            Vector3d point = triR[2] + (veczL.dot(triR[2] - triL[0]) / deno) * (triR[0] - triR[2]);
            if (_isPointInTriangle(point, triL))
                return true;
        }
    }
    // reversal
    if (acrossL2R_A) // first filter
    {
        double deno = veczR.dot(triL[0] - triL[1]);
        if (fabs(deno) < eps) // perpendi to veczR
        {
            if (_isEdgeCrossTriangle({ triL[0], triL[1] }, triR)) //segment on plane
                return true;
        }
        else
        {
            Vector3d point = triL[0] + (veczL.dot(triL[0] - triR[0]) / deno) * (triL[1] - triL[0]);
            if (_isPointInTriangle(point, triR))
                return true;
        }
    }
    if (acrossL2R_B) // first filter
    {
        double deno = veczR.dot(triL[1] - triL[2]);
        if (fabs(deno) < eps) // perpendi to veczR
        {
            if (_isEdgeCrossTriangle({ triL[1], triL[2] }, triR)) //segment on plane
                return true;
        }
        else
        {
            Vector3d point = triL[0] + (veczL.dot(triL[0] - triR[0]) / deno) * (triL[1] - triL[0]);
            if (_isPointInTriangle(point, triR))
                return true;
        }
    }
    if (acrossL2R_B) // first filter
    {
        double deno = veczR.dot(triL[2] - triL[0]);
        if (fabs(deno) < eps) // perpendi to veczR
        {
            if (_isEdgeCrossTriangle({ triL[2], triL[0] }, triR)) //segment on plane
                return true;
        }
        else
        {
            Vector3d point = triL[2] + (veczL.dot(triL[2] - triR[0]) / deno) * (triL[0] - triL[2]);
            if (_isPointInTriangle(point, triR))
                return true;
        }
    }
    return false;
}


bool isTwoTrianglesIntersection2(const std::array<Vector3d, 3>& triL, const std::array<Vector3d, 3>& triR)
{

    // right edge through left plane
    Vector3d veczL = (triL[1] - triL[0]).cross(triL[2] - triL[0]);
    bool acrossR2L_A = (veczL.dot(triR[0] - triL[0])) * (veczL.dot(triR[1] - triL[0])) < eps; //include point-on-plane
    bool acrossR2L_B = (veczL.dot(triR[1] - triL[0])) * (veczL.dot(triR[2] - triL[0])) < eps;
    bool acrossR2L_C = (veczL.dot(triR[2] - triL[0])) * (veczL.dot(triR[0] - triL[0])) < eps;
    if (!acrossR2L_A && !acrossR2L_B && !acrossR2L_C)
        return false;

    // left edge through right plane
    Vector3d veczR = (triR[1] - triR[0]).cross(triR[2] - triR[0]);
    bool acrossL2R_A = (veczR.dot(triR[0] - triL[0])) * (veczR.dot(triR[1] - triL[0])) < eps;
    bool acrossL2R_B = (veczR.dot(triR[1] - triL[0])) * (veczR.dot(triR[2] - triL[0])) < eps;
    bool acrossL2R_C = (veczR.dot(triR[2] - triL[0])) * (veczR.dot(triR[0] - triL[0])) < eps;
    if (!acrossL2R_A && !acrossL2R_B && !acrossL2R_C)
        return false;

    // using face to-left test
    //Vector3d veczL = (triL[1] - triL[0]).cross(triL[2] - triL[0]); // triL close face triangle
    //bool acrossR2L_A = (veczL.dot(triR[0] - triL[0])) * (veczL.dot(triR[1] - triL[0])) < eps; //include point-on-plane
    if (acrossR2L_A) // first filter
    {
        bool pointOnface_s = fabs(veczL.dot(triR[0] - triL[0])) < eps; //start point
        bool pointOnface_e = fabs(veczL.dot(triR[1] - triL[0])) < eps; //end point
        if (!pointOnface_s)
        {
            if (_isSegmentCrossTriangleSurface({ triR[0], triR[1] }, triL))
                return true;
        }
        else if (!pointOnface_e)
        {
            if (_isSegmentCrossTriangleSurface({ triR[1], triR[0] }, triL))
                return true;
        }
        else if (pointOnface_s && pointOnface_e)
        {
            if (_isEdgeCrossTriangle({ triR[0], triR[1] }, triL)) //segment on plane
                return true;
        }
    }
    //bool acrossR2L_B = (veczL.dot(triR[1] - triL[0])) * (veczL.dot(triR[2] - triL[0])) < eps;
    if (acrossR2L_B) // first filter
    {
        bool pointOnface_s = fabs(veczL.dot(triR[1] - triL[0])) < eps; //start point
        bool pointOnface_e = fabs(veczL.dot(triR[2] - triL[0])) < eps; //end point
        if (!pointOnface_s)
        {
            if (_isSegmentCrossTriangleSurface({ triR[1], triR[2] }, triL))
                return true;
        }
        else if (!pointOnface_e)
        {
            if (_isSegmentCrossTriangleSurface({ triR[2], triR[1] }, triL))
                return true;
        }
        else if (pointOnface_s && pointOnface_e)
        {
            if (_isEdgeCrossTriangle({ triR[1], triR[2] }, triL)) //segment on plane
                return true;
        }
    }
    //bool acrossR2L_C = (veczL.dot(triR[2] - triL[0])) * (veczL.dot(triR[0] - triL[0])) < eps;
    if (acrossR2L_C) // first filter
    {
        bool pointOnface_s = fabs(veczL.dot(triR[2] - triL[0])) < eps; //start point
        bool pointOnface_e = fabs(veczL.dot(triR[0] - triL[0])) < eps; //end point
        if (!pointOnface_s)
        {
            if (_isSegmentCrossTriangleSurface({ triR[2], triR[1] }, triL))
                return true;
        }
        else if (!pointOnface_e)
        {
            if (_isSegmentCrossTriangleSurface({ triR[0], triR[0] }, triL))
                return true;
        }
        else if (pointOnface_s && pointOnface_e)
        {
            if (_isEdgeCrossTriangle({ triR[2], triR[0] }, triL)) //segment on plane
                return true;
        }
    }
    //// exchange two triangles
    //Vector3d veczR = (triR[1] - triR[0]).cross(triR[2] - triR[0]); // triR close face triangle
    //bool acrossL2R_A = (veczR.dot(triL[0] - triR[0])) * (veczR.dot(triL[1] - triR[0])) < eps; //include point-on-plane
    if (acrossL2R_A) // first filter
    {
        bool pointOnface_s = fabs(veczR.dot(triL[0] - triR[0])) < eps; //start point
        bool pointOnface_e = fabs(veczR.dot(triL[1] - triR[0])) < eps; //end point
        if (!pointOnface_s)
        {
            if (_isSegmentCrossTriangleSurface({ triL[0], triL[1] }, triR))
                return true;
        }
        else if (!pointOnface_e)
        {
            if (_isSegmentCrossTriangleSurface({ triL[1], triL[0] }, triR))
                return true;
        }
        else if (pointOnface_s && pointOnface_e)
        {
            if (_isEdgeCrossTriangle({ triL[0], triL[1] }, triR)) //segment on plane
                return true;
        }
    }
    //bool acrossL2R_B = (veczR.dot(triL[1] - triR[0])) * (veczR.dot(triL[2] - triR[0])) < eps;
    if (acrossL2R_B) // first filter
    {
        bool pointOnface_s = fabs(veczR.dot(triL[1] - triR[0])) < eps; //start point
        bool pointOnface_e = fabs(veczR.dot(triL[2] - triR[0])) < eps; //end point
        if (!pointOnface_s)
        {
            if (_isSegmentCrossTriangleSurface({ triL[1], triL[2] }, triR))
                return true;
        }
        else if (!pointOnface_e)
        {
            if (_isSegmentCrossTriangleSurface({ triL[2], triL[1] }, triR))
                return true;
        }
        else if (pointOnface_s && pointOnface_e)
        {
            if (_isEdgeCrossTriangle({ triL[1], triL[2] }, triR)) //segment on plane
                return true;
        }
    }
    //bool acrossL2R_C = (veczR.dot(triL[2] - triL[0])) * (veczR.dot(triL[0] - triL[0])) < eps;
    if (acrossL2R_C) // first filter
    {
        bool pointOnface_s = fabs(veczR.dot(triL[2] - triL[0])) < eps; //start point
        bool pointOnface_e = fabs(veczR.dot(triL[0] - triL[0])) < eps; //end point
        if (!pointOnface_s)
        {
            if (_isSegmentCrossTriangleSurface({ triL[2], triL[1] }, triL))
                return true;
        }
        else if (!pointOnface_e)
        {
            if (_isSegmentCrossTriangleSurface({ triL[0], triL[0] }, triL))
                return true;
        }
        else if (pointOnface_s && pointOnface_e)
        {
            if (_isEdgeCrossTriangle({ triL[2], triL[0] }, triL)) //segment on plane
                return true;
        }
    }
    return false;
}
