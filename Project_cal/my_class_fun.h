#pragma once

namespace psykronix
{
    static constexpr size_t N_10E_3 = (size_t)1e3;
    static constexpr size_t N_10E_4 = (size_t)1e4;
    static constexpr size_t N_10E_5 = (size_t)1e5;
    static constexpr size_t N_10E_6 = (size_t)1e6;
    static constexpr size_t N_10E_7 = (size_t)1e7;
    static constexpr size_t N_10E_8 = (size_t)1e8;

#ifndef CLASH_DETECTION_SOLUTION
    struct InterTriInfo
    {
        std::array<std::array<Eigen::Vector3d, 3>, 2> trianglePair;
        std::array<unsigned long long, 2> entityPair;
        double distance;
    };
#endif

    class Vertex3d
    {
    public:
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        Vertex3d() {}
        Vertex3d(double x_, double y_, double z_ = 0.0) :
            x(x_),
            y(y_),
            z(z_)
        {
        }
        inline bool operator==(const Vertex3d& rhs) const
        {
            return memcmp(this, &rhs, sizeof(Vertex3d)) == 0;
        }
        inline bool operator<(const Vertex3d& rhs) const
        {
            return memcmp(this, &rhs, sizeof(Vertex3d)) < 0;
        }
#ifdef EIGEN_WORLD_VERSION
        Vertex3d(const Eigen::Vector3d& vec) :
            x(vec.x()),
            y(vec.y()),
            z(vec.z())
        {
        }
        Vertex3d(const Eigen::Vector3f& vec) :
            x(vec.x()),
            y(vec.y()),
            z(vec.z())
        {
        }
        operator Eigen::Vector3d() const
        {
            return Eigen::Vector3d(x, y, z);
        }
        operator Eigen::Vector3f() const
        {
            return Eigen::Vector3f((float)x, (float)y, (float)z);
        }
        //auto operator[](int i)
        //{
        //    if (i == 0)
        //        return x;
        //}
#endif
    };
}

namespace eigen
{
    // inline function
    inline Eigen::Vector2d to_vec2(const Eigen::Vector3d& vec3)
    {
        return Eigen::Vector2d(vec3[0], vec3[1]); // to replace shadow matrix of relative coordinate
    }

    inline Eigen::Vector3d to_vec3(const Eigen::Vector2d& vec2)
    {
        return Eigen::Vector3d(vec2[0], vec2[1], 0.0);
    }

    inline Eigen::Vector3d cross(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2)
    {
        return Eigen::Vector3d(0, 0, v1[0] * v2[1] - v1[1] * v2[0]);
    }

    inline double cross2d(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2)
    {
        return v1[0] * v2[1] - v1[1] * v2[0];
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

    inline Eigen::Vector3d operator*=(const Eigen::Matrix4d& mat, const Eigen::Vector3d& vec) //operator* been occupied
    {
        Eigen::Vector4d res = mat * vec.homogeneous();
        return res.hnormalized();
    }

    inline std::array<Eigen::Vector3d, 2> operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector3d, 2>& seg)
    {
        std::array<Eigen::Vector3d, 2> res;
        for (int i = 0; i < 2; i++)
        {
            Eigen::Vector4d vec4 = mat * (seg[i].homogeneous());
            res[i] = vec4.hnormalized();
        }
        return res;
    }

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

    inline std::array<Eigen::Vector3d, 3> operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector3d, 3>& trigon)
    {
        std::array<Eigen::Vector3d, 3> triangle = {
            (mat * trigon[0].homogeneous()).hnormalized(),
            (mat * trigon[1].homogeneous()).hnormalized(),
            (mat * trigon[2].homogeneous()).hnormalized() };
        return triangle;
    }

    inline std::array<Eigen::Vector2d, 3> operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector2d, 3>& trigon)
    {
        std::array<Eigen::Vector4d, 3> triangle = {
            mat * Eigen::Vector4d(trigon[0][0],trigon[0][1],0,1),
            mat * Eigen::Vector4d(trigon[1][0],trigon[1][1],0,1),
            mat * Eigen::Vector4d(trigon[2][0],trigon[2][1],0,1) };
        return {
            Eigen::Vector2d(triangle[0][0],triangle[0][1]),
            Eigen::Vector2d(triangle[1][0],triangle[1][1]),
            Eigen::Vector2d(triangle[2][0],triangle[2][1]) };
    }

    inline std::array<Eigen::Matrix4d, 2> getMatrixFromThreePoints(const std::array<Eigen::Vector3d, 3>& triangle)
    {
        // legal triangle
        Eigen::Vector3d axisx = (triangle[1] - triangle[0]).normalized();
        if (axisx.isZero(psykronix::eps)) //safe check
            axisx = Eigen::Vector3d(1, 0, 0);
        Eigen::Vector3d axisy = (triangle[2] - triangle[1]);
        if (axisy.isZero(psykronix::eps))
            axisy = Eigen::Vector3d(0, 1, 0);
        Eigen::Vector3d axisz = axisx.cross(axisy).normalized();
        if (axisz.isZero(psykronix::eps))
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

}
