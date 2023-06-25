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
    DLLEXPORT std::array<Eigen::Vector3d, 3> operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector3d, 3>& tri);
    DLLEXPORT std::array<Eigen::Vector3f, 3> operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector3f, 3>& tri);

}
