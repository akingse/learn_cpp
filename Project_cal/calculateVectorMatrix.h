#pragma once

namespace eigen
{
    // inline function
    inline int math_sign(double num)
    {
        if (num == 0)
            return 0;
        return (0.0 < num) ? 1 : -1;
    }

    inline Eigen::Vector2d to_vec2(const Eigen::Vector3d& vec3)
    {
        return Eigen::Vector2d(vec3[0], vec3[1]); // to replace shadow matrix of relative coordinate
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

    inline std::array<Eigen::Vector3d, 3> to_vec3(const std::array<Eigen::Vector2d, 3>& vec2s)
    {
        std::array<Eigen::Vector3d, 3> vec3s;
        for (int i = 0; i < 3; ++i)
            vec3s[i] = to_vec3(vec2s[i]);
        return vec3s;
    }

    inline Eigen::Vector3d cross(const Eigen::Vector2d& v0, const Eigen::Vector2d& v1)
    {
        return Eigen::Vector3d(0, 0, v0[0] * v1[1] - v0[1] * v1[0]);
    }

    inline double cross2d(const Eigen::Vector2d& v0, const Eigen::Vector2d& v1)
    {
        return v0[0] * v1[1] - v0[1] * v1[0];
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
        Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(theta, axis));
        Eigen::Matrix3d R = q.toRotationMatrix();
        Eigen::Matrix4d mat4d = Eigen::Matrix4d::Identity();
        mat4d.block<3, 3>(0, 0) = R; //block<rows,cols>(row_index,col_index) <>is child size, () is begin index
        //mat4d.block<1, 3>(3, 0) << 0, 0, 0; 
        //mat4d(3, 3) = 1; 
        return mat4d;
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
            1, 0, 0, vec.x(),
            0, 1, 0, vec.y(),
            0, 0, 1, vec.z(),
            0, 0, 0, 1;
        return T;
    }

    inline Eigen::Matrix4d rotate(const Eigen::Vector3d& position, const Eigen::Vector3d& axis, double theta = 0.0)
    {
        Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(theta, axis));
        Eigen::Matrix3d R = q.toRotationMatrix();
        Eigen::Matrix4d mat4d = Eigen::Matrix4d::Identity();
        mat4d.block<3, 3>(0, 0) = R; //block<rows,cols>(row_index,col_index) <>is child size, () is begin index
		return translate(-position) * mat4d * translate(position);
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

    //operator overload
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
            axisx[0], axisx[1], axisx[2], -triangle[0][0],
            axisy[0], axisy[1], axisy[2], -triangle[0][1],
            axisz[0], axisz[1], axisz[2], -triangle[0][2],
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

    //inline Eigen::Vector3d operator*=(const Eigen::Matrix4d& mat, const Eigen::Vector3d& vec) //operator* been occupied
    //{
    //    Eigen::Vector4d res = mat * vec.homogeneous();
    //    return res.hnormalized();
    //}
    //inline std::array<Eigen::Vector3d, 2> operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector3d, 2>& seg)
    //{
    //    std::array<Eigen::Vector3d, 2> res;
    //    for (int i = 0; i < 2; i++)
    //    {
    //        Eigen::Vector4d vec4 = mat * (seg[i].homogeneous());
    //        res[i] = vec4.hnormalized();
    //    }
    //    return res;
    //}
    //inline std::array<Eigen::Vector3d, 3> operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector3d, 3>& tri)
    //{
    //    std::array<Eigen::Vector3d, 3> res;
    //    for (int i = 0; i < 3; i++)
    //    {
    //        Eigen::Vector4d vec4 = mat * (tri[i].homogeneous()); //Vector4d(tri[i].x(), tri[i].y(), tri[i].z(), 1);
    //        res[i] = vec4.hnormalized(); // Vector3d(vec4.x(), vec4.y(), vec4.z());
    //    }
    //    return res;
    //}
    //inline std::array<Eigen::Vector3f, 3> operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector3f, 3>& tri)
    //{
    //    std::array<Eigen::Vector3f, 3> res;
    //    Eigen::Matrix4f mat_float = mat.cast<float>();
    //    for (int i = 0; i < 3; i++)
    //    {
    //        Eigen::Vector4f vec4 = mat_float * (tri[i].homogeneous()); //Vector4d(tri[i].x(), tri[i].y(), tri[i].z(), 1);
    //        res[i] = vec4.hnormalized(); //Vector3f(vec4.x(), vec4.y(), vec4.z()); //
    //    }
    //    return res;
    //}

    inline void print_triangle(const std::array<Eigen::Vector3d, 3>& T1) //not accurate
    {
        std::cout << "trigon= " << std::endl << "(" << T1[0].x() << ", " << T1[0].y() << ", " << T1[0].z() << ")" << std::endl;
        std::cout << "(" << T1[1].x() << ", " << T1[1].y() << ", " << T1[1].z() << ")" << std::endl;
        std::cout << "(" << T1[2].x() << ", " << T1[2].y() << ", " << T1[2].z() << ")" << std::endl;
    }

    inline void print_triangle(const std::array<Eigen::Vector3d, 3>& T1, const std::array<Eigen::Vector3d, 3>& T2)
    {
        std::cout << "triPair=" << std::endl << "(" << T1[0].x() << ", " << T1[0].y() << ", " << T1[0].z() << ")" << std::endl;
        std::cout << "(" << T1[1].x() << ", " << T1[1].y() << ", " << T1[1].z() << ")" << std::endl;
        std::cout << "(" << T1[2].x() << ", " << T1[2].y() << ", " << T1[2].z() << ")" << std::endl;
        std::cout << "(" << T2[0].x() << ", " << T2[0].y() << ", " << T2[0].z() << ")" << std::endl;
        std::cout << "(" << T2[1].x() << ", " << T2[1].y() << ", " << T2[1].z() << ")" << std::endl;
        std::cout << "(" << T2[2].x() << ", " << T2[2].y() << ", " << T2[2].z() << ")" << std::endl;
    }

}
