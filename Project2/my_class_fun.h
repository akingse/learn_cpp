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
    DLLEXPORT std::array<Eigen::Vector3d, 2> operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector3d, 2>& seg);
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


inline Vec2 _get_rand()
{
    double r = 100;
    return Vec2(rand(), rand());
}

inline Eigen::Vector3d _to2D(const Eigen::Vector3d& vec3)
{
    return Eigen::Vector3d(vec3.x(), vec3.y(), 0.0);
}

inline std::array<para::BPParaVec, 3> _get_rand3v()
{
    // rand -16384 -> 16384 
    //srand((int)time(0));
    //Sleep(100);
    return std::array<para::BPParaVec, 3> {
        para::BPParaVec(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
            para::BPParaVec(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
            para::BPParaVec(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff) };
}
inline std::array<Eigen::Vector3d, 3> _get_rand3(int i = 0)
{
    if (i == 0)
        return std::array<Eigen::Vector3d, 3> {
        Eigen::Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
            Eigen::Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
            Eigen::Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff) };
    if (i == 1)
        return std::array<Eigen::Vector3d, 3> {
        Eigen::Vector3d(rand(), rand(), rand()),
            Eigen::Vector3d(rand(), rand(), rand()),
            Eigen::Vector3d(rand(), rand(), rand()) };
    if (i == -1)
        return std::array<Eigen::Vector3d, 3> {
        Eigen::Vector3d(rand() - 0x7fff, rand() - 0x7fff, rand() - 0x7fff),
            Eigen::Vector3d(rand() - 0x7fff, rand() - 0x7fff, rand() - 0x7fff),
            Eigen::Vector3d(rand() - 0x7fff, rand() - 0x7fff, rand() - 0x7fff) };
}

inline std::array<Eigen::Vector3d, 2> _get_rand2()
{
    return std::array<Eigen::Vector3d, 2> {
        Eigen::Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
            Eigen::Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff) };
}

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
    std::vector<int> iboRaw_; //for test debug
};
#endif
