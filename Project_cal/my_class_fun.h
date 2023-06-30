#pragma once
namespace psykronix
{
    class Vertex
    {
    public:
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        Vertex() {}
        Vertex(double x_, double y_, double z_ = 0.0) :
            x(x_),
            y(y_),
            z(z_)
        {
        }
        inline bool operator==(const Vertex& rhs) const
        {
            return memcmp(this, &rhs, sizeof(Vertex)) == 0;
        }
        inline bool operator<(const Vertex& rhs) const
        {
            return memcmp(this, &rhs, sizeof(Vertex)) < 0;
        }
#ifdef EIGEN_WORLD_VERSION
        Vertex(const Eigen::Vector3d& vec) :
            x(vec.x()),
            y(vec.y()),
            z(vec.z())
        {
        }
        Vertex(const Eigen::Vector3f& vec) :
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
            return Eigen::Vector3f(x, y, z);
        }
        //auto operator[](int i)
        //{
        //    if (i == 0)
        //        return x;
        //}
#endif
    };

    //overload
    DLLEXPORT Eigen::Matrix4d rotx(double theta);
    DLLEXPORT Eigen::Matrix4d roty(double theta);
    DLLEXPORT Eigen::Matrix4d rotz(double theta);
    DLLEXPORT Eigen::Matrix4d rotate(const Eigen::Vector3d& axis = { 0, 0, 1 }, double theta = 0.0);
    DLLEXPORT Eigen::Matrix4d translate(const Eigen::Vector3d& vec);
    DLLEXPORT Eigen::Matrix4d translate(double x, double y, double z = 0.0);
    DLLEXPORT Eigen::Matrix4d scale(double x, double y, double z = 1.0);
    DLLEXPORT Eigen::Matrix4d mirrorx();
    DLLEXPORT Eigen::Matrix4d mirrory();
    DLLEXPORT Eigen::Matrix4d mirrorz();
    DLLEXPORT Eigen::Vector3d operator*=(const Eigen::Matrix4d& mat, const Eigen::Vector3d& vec); //operator* been occupied
    DLLEXPORT std::array<Eigen::Vector3d, 3> operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector3d, 3>& tri);
    DLLEXPORT std::array<Eigen::Vector3f, 3> operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector3f, 3>& tri);


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
