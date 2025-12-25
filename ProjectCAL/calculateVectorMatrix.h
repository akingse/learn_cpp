#pragma once
/*******************************************************************************
* Editor    :  akingse		                                                   *
* Date      :  from June 2023												   *
* Website   :  https://github.com/akingse                                      *
* Copyright :  All rights reserved											   *
* Purpose   :  Some common vector and matrix calculation methods			   *
* License   :  MIT									                           *
*******************************************************************************/
#ifndef CALCULATE_VECTORMATRIX_H
#define CALCULATE_VECTORMATRIX_H
namespace eigen
{
    // inline function
    inline int math_sign(double num)
    {
        if (num == 0.0)
            return 0;
        return (0.0 < num) ? 1 : -1;
    }

    inline Eigen::Vector2d to_vec2(const Eigen::Vector3d& vec3)
    {
        return Eigen::Vector2d(vec3[0], vec3[1]); // to replace shadow matrix of relative coordinate
    }

    inline std::vector<Eigen::Vector2d> to_vec2(const std::vector<Eigen::Vector3d>& vec3s)
    {
        std::vector<Eigen::Vector2d> res(vec3s.size());
        for (int i = 0; i < (int)res.size(); ++i)
            res[i] = to_vec2(vec3s[i]);
        return res;
    }

    inline std::array<Eigen::Vector2d, 3> to_vec2(const std::array<Eigen::Vector3d, 3>& vec3s)
    {
        std::array<Eigen::Vector2d, 3> vec2s;
        for (int i = 0; i < 3; ++i)
            vec2s[i] = to_vec2(vec3s[i]);
        return vec2s;
    }

    inline Eigen::Vector3d to_vec3(const Eigen::Vector2d& vec2)
    {
        return Eigen::Vector3d(vec2[0], vec2[1], 0.0);
    }

    inline std::vector<Eigen::Vector3d> to_vec3(const std::vector<Eigen::Vector2d>& vec2s)
    {
        std::vector<Eigen::Vector3d> res(vec2s.size());
        for (int i = 0; i < (int)res.size(); ++i)
            res[i] = to_vec3(vec2s[i]);
        return res;
    }

    inline std::array<Eigen::Vector3d, 3> to_vec3(const std::array<Eigen::Vector2d, 3>& vec2s)
    {
        std::array<Eigen::Vector3d, 3> vec3s;
        for (int i = 0; i < 3; ++i)
            vec3s[i] = to_vec3(vec2s[i]);
        return vec3s;
    }

    inline Eigen::Vector4d to_vec4(const Eigen::Vector2d& vec2)
    {
        return Eigen::Vector4d(vec2[0], vec2[1], 0.0, 1.0);
    }

    inline Eigen::Vector3d cross(const Eigen::Vector2d& v0, const Eigen::Vector2d& v1)
    {
        return Eigen::Vector3d(0, 0, v0[0] * v1[1] - v0[1] * v1[0]);
    }

    inline double cross2d(const Eigen::Vector2d& v0, const Eigen::Vector2d& v1)
    {
        return v0[0] * v1[1] - v0[1] * v1[0];
    }

    //avoid type ambiguous
    //inline double angle_two_vectors(const Eigen::Vector2d& v0, const Eigen::Vector2d& v1)
    //{
    //    double cosRes = v0.dot(v1) / (v0.norm() * v1.norm());
    //    return std::acos(cosRes);
    //}

    inline double get_angle_of_two_vectors(const Eigen::Vector3d& V0, const Eigen::Vector3d& V1)
    {
        double cosres = cosres = V0.dot(V1) / (V0.norm() * V1.norm()); //+1 or -1 
        if (1.0 < cosres) //avoid nan
            cosres = 1.0;
        else if (-1.0 > cosres)
            cosres = -1.0;
        return std::acos(cosres); // 0->PI
    }

    inline double get_angle_of_two_unit(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1)
    {
        double cosres = v0.dot(v1);// / (v0.norm() * v1.norm());
        if (1.0 < cosres) //avoid nan
            cosres = 1.0;
        else if (-1.0 > cosres)
            cosres = -1.0;
        return std::acos(cosres); // 0->PI
    }

    // dynamic accuracy
    //bool isParallel(const Eigen::Vector2d& vecA, const Eigen::Vector2d& vecB);
    //bool isParallel(const Eigen::Vector3d& vecA, const Eigen::Vector3d& vecB);
    //bool isPerpendi(const Eigen::Vector2d& vecA, const Eigen::Vector2d& vecB);
    //bool isPerpendi(const Eigen::Vector3d& vecA, const Eigen::Vector3d& vecB);

    inline bool isParallel2d(const Eigen::Vector2d& vecA, const Eigen::Vector2d& vecB, double tole = FLT_EPSILON)// input accuracy
    {
        return fabs(vecA[0] * vecB[1] - vecA[1] * vecB[0]) <= tole;
    }

    inline bool isParallel3d(const Eigen::Vector3d& vecA, const Eigen::Vector3d& vecB, double tole = FLT_EPSILON)// input accuracy
    {
        return vecA.cross(vecB).isZero(tole);
    }

    inline bool isPerpendi2d(const Eigen::Vector2d& vecA, const Eigen::Vector2d& vecB, double tole = FLT_EPSILON)// input accuracy
    {
        return vecA.dot(vecB) <= tole;
    }

    inline bool isPerpendi3d(const Eigen::Vector3d& vecA, const Eigen::Vector3d& vecB, double tole = FLT_EPSILON)// input accuracy
    {
        return vecA.dot(vecB) <= tole;
    }

    inline bool isParallelDA(const Eigen::Vector2d& vecA, const Eigen::Vector2d& vecB, double tole = FLT_EPSILON)// dynamic accuracy
    {
        double k = 0;
        for (int i = 0; i < 2; ++i)
            k = std::max(std::max(fabs(vecA[i]), fabs(vecB[i])), k);
        return fabs(vecA[0] * vecB[1] - vecA[1] * vecB[0]) <= k * tole;// support both zero
    }

    inline bool isParallelDA(const Eigen::Vector3d& vecA, const Eigen::Vector3d& vecB, double tole = FLT_EPSILON)// dynamic accuracy
    {
        double k = 0;
        for (int i = 0; i < 3; ++i)
            k = std::max(std::max(fabs(vecA[i]), fabs(vecB[i])), k);
        return vecA.cross(vecB).isZero(k * tole);
    }

    inline bool isPerpendiDA(const Eigen::Vector2d& vecA, const Eigen::Vector2d& vecB, double tole = FLT_EPSILON)// dynamic accuracy
    {
        double k = 0;
        for (int i = 0; i < 2; ++i)
            k = std::max(std::max(fabs(vecA[i]), fabs(vecB[i])), k); 
        return vecA.dot(vecB) <= k * tole;// support both zero
    }
    
    inline bool isPerpendiDA(const Eigen::Vector3d& vecA, const Eigen::Vector3d& vecB, double tole = FLT_EPSILON)// dynamic accuracy
    {
        double k = 0;
        for (int i = 0; i < 3; ++i)
            k = std::max(std::max(fabs(vecA[i]), fabs(vecB[i])), k); 
        return vecA.dot(vecB) <= k * tole;// support both zero
    }

    //overload
    inline Eigen::Matrix2d scale2d(double x, double y)
    {
        Eigen::Matrix2d mat = Eigen::Matrix2d::Identity();
        mat <<
            x, 0,
            0, y;
        return mat;
    }

    inline Eigen::Matrix4d translate(double x, double y, double z = 0.0)
    {
        Eigen::Matrix4d T;
        T <<
            1, 0, 0, x,
            0, 1, 0, y,
            0, 0, 1, z,
            0, 0, 0, 1;
        return T;
    }

    inline Eigen::Matrix4d translate(const Eigen::Vector3d& vec)
    {
        Eigen::Matrix4d T;
        T <<
            1, 0, 0, vec[0],
            0, 1, 0, vec[1],
            0, 0, 1, vec[2],
            0, 0, 0, 1;
        return T;
    }

    inline Eigen::Matrix4d rotx(double theta)
    {
        Eigen::Matrix4d R;
        R << 
            1, 0, 0, 0,
            0, cos(theta), -sin(theta), 0,
            0, sin(theta), cos(theta), 0,
            0, 0, 0, 1;
        return R;
    }

    inline Eigen::Matrix4d roty(double theta)
    {
        Eigen::Matrix4d R;
        R << 
            cos(theta), 0, sin(theta), 0,
            0, 1, 0, 0,
            -sin(theta), 0, cos(theta), 0,
            0, 0, 0, 1;
        return R;
    }

    inline Eigen::Matrix4d rotz(double theta)
    {
        Eigen::Matrix4d R;
        R << 
            cos(theta), -sin(theta), 0, 0,
            sin(theta), cos(theta), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
        return R;
    }

    inline Eigen::Matrix4d rotate(const Eigen::Vector3d& axis = { 0, 0, 1 }, double theta = 0.0)
    {
        Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(theta, axis.normalized()));
        Eigen::Matrix3d R = q.toRotationMatrix();
        Eigen::Matrix4d mat4d = Eigen::Matrix4d::Identity();
        mat4d.block<3, 3>(0, 0) = R; //block<rows,cols>(row_index,col_index) <>is child size, () is begin index
        //mat4d.block<1, 3>(3, 0) << 0, 0, 0; 
        //mat4d(3, 3) = 1; 
        return mat4d;
    }

    inline Eigen::Matrix4d rotate(const Eigen::Vector3d& position, const Eigen::Vector3d& axis, double theta = 0.0)
    {
        Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(theta, axis.normalized()));
        Eigen::Matrix3d R = q.toRotationMatrix();
        Eigen::Matrix4d mat4d = Eigen::Matrix4d::Identity();
        mat4d.block<3, 3>(0, 0) = R; //block<rows,cols>(row_index,col_index) <>is child size, () is begin index
		return eigen::translate(-position) * mat4d * translate(position);
    }

    inline Eigen::Matrix4d translate(const Eigen::Vector2d& vec)
    {
        Eigen::Matrix4d T;
        T <<
            1, 0, 0, vec.x(),
            0, 1, 0, vec.y(),
            0, 0, 1, 0,
            0, 0, 0, 1;
        return T;
    }

    inline Eigen::Matrix4d scale(double x)
    {
        Eigen::Matrix4d T;
        T << 
            x, 0, 0, 0,
            0, x, 0, 0,
            0, 0, x, 0,
            0, 0, 0, 1;
        return T;
    }
    
    inline Eigen::Matrix4d scale(double x, double y, double z = 1.0)
    {
        Eigen::Matrix4d T;
        T << 
            x, 0, 0, 0,
            0, y, 0, 0,
            0, 0, z, 0,
            0, 0, 0, 1;
        return T;
    }

    inline Eigen::Matrix4d mirrorx()
    {
        Eigen::Matrix4d T;
        T << 
            -1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
        return T;
    }

    inline Eigen::Matrix4d mirrory() 
    {
        Eigen::Matrix4d T;
        T << 
            1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
        return T;
    }

    inline Eigen::Matrix4d mirrorz()
    {
        Eigen::Matrix4d T;
        T << 
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, -1, 0,
            0, 0, 0, 1;
        return T;
    }

    inline Eigen::Matrix3d toMatrix3d(const Eigen::Matrix4d& mat4)
    {
        Eigen::Matrix3d mat3 = mat4.block<3, 3>(0, 0);
        return mat3;
    }
    inline Eigen::Matrix4d toMatrix4d(const Eigen::Matrix3d& mat3)
    {
        Eigen::Matrix4d mat4 = Eigen::Matrix4d::Identity();
        mat4.block<3, 3>(0, 0) = mat3;
        return mat4;
    }

    inline Eigen::Matrix4d rotx90()
    {
        Eigen::Matrix4d R;
        R <<
            1, 0, 0, 0,
            0, 0, -1, 0,
            0, 1, 0, 0,
            0, 0, 0, 1;
        return R;
    }

    inline Eigen::Matrix4d roty90()
    {
        Eigen::Matrix4d R;
        R <<
            0, 0, 1, 0,
            0, 1, 0, 0,
            -1, 0, 0, 0,
            0, 0, 0, 1;
        return R;
    }

    inline Eigen::Matrix4d rotz90()
    {
        Eigen::Matrix4d R;
        R <<
            0, -1, 0, 0,
            1, 0, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
        return R;
    }

    //operator overload
    inline Eigen::Vector3d operator_mul(const Eigen::Matrix4d& mat, const Eigen::Vector3d& vec)
    {
        return (mat * vec.homogeneous()).hnormalized();
    }

    inline std::array<Eigen::Vector3d, 2> operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector3d, 2>& line)
    {
        std::array<Eigen::Vector3d, 2> segment = {
            (mat * line[0].homogeneous()).hnormalized(),
            (mat * line[1].homogeneous()).hnormalized() };
        return segment;
    }

    inline std::array<Eigen::Vector3d, 3> operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector3d, 3>& trigon)
    {
        std::array<Eigen::Vector3d, 3> triangle = {
            (mat * trigon[0].homogeneous()).hnormalized(),
            (mat * trigon[1].homogeneous()).hnormalized(),
            (mat * trigon[2].homogeneous()).hnormalized() };
        return triangle;
    }

    inline std::array<Eigen::Vector2d, 2> operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector2d, 2>& line)
    {
        std::array<Eigen::Vector4d, 2> segment = {
            mat * Eigen::Vector4d(line[0][0],line[0][1],0,1),
            mat * Eigen::Vector4d(line[1][0],line[1][1],0,1) };
        return {
            Eigen::Vector2d(segment[0][0],segment[0][1]),
            Eigen::Vector2d(segment[1][0],segment[1][1]) };
    }

    inline std::array<Eigen::Vector3d, 3> operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector2d, 3>& trigon)
    {
        std::array<Eigen::Vector4d, 3> triangle = {
            mat * Eigen::Vector4d(trigon[0][0],trigon[0][1],0,1),
            mat * Eigen::Vector4d(trigon[1][0],trigon[1][1],0,1),
            mat * Eigen::Vector4d(trigon[2][0],trigon[2][1],0,1) };
        return {
            triangle[0].hnormalized(),
            triangle[1].hnormalized(),
            triangle[2].hnormalized() };
    }

    inline std::array<Eigen::Matrix4d, 2> getMatrixFromThreePoints(const std::array<Eigen::Vector3d, 3>& triangle)
    {
        // legal triangle
        Eigen::Vector3d axisx = (triangle[1] - triangle[0]).normalized();
        if (axisx.isZero(clash::epsF)) //safe check
            axisx = Eigen::Vector3d(1, 0, 0);
        Eigen::Vector3d axisy = (triangle[2] - triangle[1]);
        if (axisy.isZero(clash::epsF))
            axisy = Eigen::Vector3d(0, 1, 0);
        Eigen::Vector3d axisz = axisx.cross(axisy).normalized();
        if (axisz.isZero(clash::epsF))
            axisz = Eigen::Vector3d(0, 0, 1);
        axisy = axisz.cross(axisx);
        Eigen::Matrix4d matFor, matInv;
        matFor << //forword matrix
            axisx[0], axisy[0], axisz[0], triangle[0][0],
            axisx[1], axisy[1], axisz[1], triangle[0][1],
            axisx[2], axisy[2], axisz[2], triangle[0][2],
            0, 0, 0, 1;
        matInv << //inverse matrix, transpose and negation
            axisx[0], axisx[1], axisx[2], -axisx.dot(triangle[0]),
            axisy[0], axisy[1], axisy[2], -axisy.dot(triangle[0]),
            axisz[0], axisz[1], axisz[2], -axisz.dot(triangle[0]),
            0, 0, 0, 1;
        return { matFor,matInv };
    }

    inline std::array<Eigen::Matrix4d, 2> getMatrixFromThreePoints(const std::array<Eigen::Vector2d, 3>& triangle)
    {
        return getMatrixFromThreePoints(std::array<Eigen::Vector3d, 3>{
            to_vec3(triangle[0]),
            to_vec3(triangle[1]),
            to_vec3(triangle[2]) });
    }

    //direciton is axis-x
    inline Eigen::Matrix4d getMatrixFromTwoPoints(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1)
    {
        Eigen::Vector3d axisx = (p1 - p0).normalized();
        if (axisx.isZero(clash::epsF)) //safe check
            axisx = Eigen::Vector3d(1, 0, 0);
        Eigen::Vector3d axisz(0, 0, 1);
        Eigen::Vector3d axisy = axisz.cross(axisx).normalized();
        if (axisy.isZero(clash::epsF))
            axisy = Eigen::Vector3d(0, 1, 0);
        axisz = axisx.cross(axisy).normalized();
        Eigen::Matrix4d matFor, matInv;
        matFor << //forword matrix
            axisx[0], axisy[0], axisz[0], p0[0],
            axisx[1], axisy[1], axisz[1], p0[1],
            axisx[2], axisy[2], axisz[2], p0[2],
            0, 0, 0, 1;
        return matFor;
    }
    inline Eigen::Matrix4d getMatrixFromTwoPointsZ(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1)
    {
        Eigen::Vector3d axisz = (p1 - p0).normalized();
        if (axisz.isZero()) //safe check
            axisz = Eigen::Vector3d(0, 0, 1);
        Eigen::Vector3d axisx(1, 0, 0);
        Eigen::Vector3d axisy = axisz.cross(axisx).normalized();
        if (axisy.isZero())
            axisy = Eigen::Vector3d(0, 1, 0);
        axisx = axisy.cross(axisz).normalized();
        Eigen::Matrix4d matFor, matInv;
        matFor << //forword matrix
            axisx[0], axisy[0], axisz[0], p0[0],
            axisx[1], axisy[1], axisz[1], p0[1],
            axisx[2], axisy[2], axisz[2], p0[2],
            0, 0, 0, 1;
        return matFor;
    }
    //inline Eigen::Matrix4d getMatrixFromOneVector(const Eigen::Vector3d& vec)
    //{
    //    return getMatrixFromTwoPoints(Eigen::Vector3d(0, 0, 0), vec);
    //}

    // generate matrix
    DLLEXPORT_CAL Eigen::Matrix4d getProjectionMatrixByPlane(const Eigen::Vector3d& origin, const Eigen::Vector3d& normal);
    //DLLEXPORT_CAL Eigen::Matrix4d getProjectionMatrixByPlane(const clash::Plane3d& plane);
    DLLEXPORT_CAL std::array<Eigen::Matrix4d, 2> getRelativeMatrixByProjectionPlane(const Eigen::Vector3d& origin, const Eigen::Vector3d& normal);
    //DLLEXPORT_CAL std::array<Eigen::Matrix4d, 2> getRelativeMatrixByProjectionPlane(const clash::Plane3d& plane);
    DLLEXPORT_CAL Eigen::Matrix4d inverseOrth(const Eigen::Matrix4d& mat);
}

#endif// CALCULATE_VECTORMATRIX_H
