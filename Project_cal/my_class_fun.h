#pragma once

#ifndef CLASH_DETECTION_SOLUTION
struct InterTriInfo
{
    std::array<std::array<Eigen::Vector3d, 3>, 2> trianglePair;
    std::array<unsigned long long, 2> entityPair;
    double distance;
};

struct ModelMesh
{
    std::vector<Eigen::Vector3d> vbo_;
    std::vector<std::array<int, 3>> ibo_;
    Eigen::AlignedBox3d bounding_;
    Eigen::Affine3d pose_; // Eigen::Affine3d::Identity()
    bool convex_; // isConvex default true
    //uint64_t instanceid_;
    std::vector<int> iboRaw_; //for test debug
    uint64_t entityid_;
};
#endif

namespace psykronix
{
    static constexpr size_t N_10E_3 = (size_t)1e3;
    static constexpr size_t N_10E_4 = (size_t)1e4;
    static constexpr size_t N_10E_5 = (size_t)1e5;
    static constexpr size_t N_10E_6 = (size_t)1e6;
    static constexpr size_t N_10E_7 = (size_t)1e7;
    static constexpr size_t N_10E_8 = (size_t)1e8;

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

    //overload
    Eigen::Matrix4d rotx(double theta);
    Eigen::Matrix4d roty(double theta);
    Eigen::Matrix4d rotz(double theta);
    Eigen::Matrix4d rotate(const Eigen::Vector3d& axis = { 0, 0, 1 }, double theta = 0.0);
    Eigen::Matrix4d translate(const Eigen::Vector3d& vec);
    Eigen::Matrix4d translate(double x, double y, double z = 0.0);
    Eigen::Matrix4d scale(double x, double y, double z = 1.0);
    Eigen::Matrix4d mirrorx();
    Eigen::Matrix4d mirrory();
    Eigen::Matrix4d mirrorz();
    Eigen::Vector3d operator*=(const Eigen::Matrix4d& mat, const Eigen::Vector3d& vec); //operator* been occupied
    std::array<Eigen::Vector3d, 2> operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector3d, 2>& seg);
    std::array<Eigen::Vector3d, 3> operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector3d, 3>& tri);
    std::array<Eigen::Vector3f, 3> operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector3f, 3>& tri);

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