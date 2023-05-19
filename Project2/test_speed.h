#pragma once
bool _isTwoTriangularIntersection(const std::array<BPParaVec, 3>& tBase, const std::array<BPParaVec, 3>& tLine);

namespace psykronix
{
    class Vertex
    {
    public:
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        Vertex() {}
        Vertex(double x_, double y_, double z_) :
            x(x_),
            y(y_),
            z(z_)
        {
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
}