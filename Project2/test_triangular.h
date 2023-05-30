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
            //x = 2 * x_;
        }
        inline bool operator==(const Vertex& other) const
        {
            return x == other.x &&
                y == other.y &&
                z == other.z;
        }
#ifdef EIGEN_WORLD_VERSION
        Vertex(const Eigen::Vector3d& vec) :
            x(vec.x()),
            y(vec.y()),
            z(vec.z())
        {
        }
        operator Eigen::Vector3d() const
        {
            return Eigen::Vector3d(x, y, z);
        }
#endif
    };

    //overload
    __declspec(dllexport) Eigen::Matrix4d rotx(double theta);
    __declspec(dllexport) Eigen::Matrix4d roty(double theta);
    __declspec(dllexport) Eigen::Matrix4d rotz(double theta);
	__declspec(dllexport) Eigen::Matrix4d rotate(const Eigen::Vector3d& axis = { 0, 0, 1 }, double theta = 0.0);
	__declspec(dllexport) Eigen::Matrix4d translate(const Eigen::Vector3d& vec);
	//__declspec(dllexport) Eigen::Matrix4d translate(const Eigen::Vector4d& vec);
	__declspec(dllexport) Eigen::Matrix4d translate(double x, double y, double z = 0.0);
    __declspec(dllexport) Eigen::Matrix4d scale(double x, double y, double z = 1.0);
    __declspec(dllexport) std::array<Eigen::Vector3d, 3> operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector3d, 3>& tri);

}

//__declspec(dllexport) bool _isTwoTriangularIntersection(const std::array<BPParaVec, 3>& tBase, const std::array<BPParaVec, 3>& tLine);
__declspec(dllexport) bool isTwoTrianglesIntersection(const std::array<Eigen::Vector3d, 3>& triL, const std::array<Eigen::Vector3d, 3>& triR);
__declspec(dllexport) bool TriangularIntersectionTest(const std::array<Eigen::Vector3d, 3>& T1, const std::array<Eigen::Vector3d, 3>& T2);

