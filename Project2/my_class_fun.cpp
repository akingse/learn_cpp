#include "pch.h"
#include "my_class_fun.h"
using namespace Eigen;

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

Eigen::Matrix4d psykronix::mirrorx()
{
    Matrix4d T;
    T << -1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return T;
}
Eigen::Matrix4d psykronix::mirrory()
{
    Matrix4d T;
    T << 1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return T;
}
Eigen::Matrix4d psykronix::mirrorz()
{
    Matrix4d T;
    T << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1;
    return T;
}

Eigen::Vector3d psykronix::operator*=(const Eigen::Matrix4d& mat, const Eigen::Vector3d& vec)
{
    Eigen::Vector4d res = mat * vec.homogeneous();
    return res.hnormalized();
}


std::array<Eigen::Vector3d, 2> psykronix::operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector3d, 2>& seg)
{
    std::array<Eigen::Vector3d, 2> res;
    for (int i = 0; i < 2; i++)
    {
        Vector4d vec4 = mat * (seg[i].homogeneous());
        res[i] = vec4.hnormalized();
    }
    return res;
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
