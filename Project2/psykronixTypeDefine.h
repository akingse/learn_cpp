#pragma once
namespace psykronix
{
    // global type and variable define 
    typedef std::array<Eigen::Vector3d, 2> Segment;
    typedef std::array<Eigen::Vector3d, 2> Segment3d;
    typedef std::array<Eigen::Vector3d, 3> Triangle;
    typedef std::array<Eigen::Vector3d, 3> Triangle3d;
    typedef std::array<Eigen::Vector3d, 2> PosVec3d;
    typedef std::tuple<std::vector<Eigen::Vector3d>, std::vector<std::array<int, 3>>> Polyhedron;
    //global constexpr
    static const Eigen::Vector3d gVecNaN(std::nan("0"), std::nan("0"), std::nan("0"));
    static const Triangle gSegNaN = { gVecNaN, gVecNaN };
    static const Triangle gTirNaN = { gVecNaN, gVecNaN, gVecNaN };
    static const PosVec3d gPVNaN = { gVecNaN ,gVecNaN };
    static const Eigen::Vector3d gVecZero = Eigen::Vector3d::Zero();// Vector3d(0, 0, 0);
    //static const Eigen::Vector3d gVecAxisX(1, 0, 0); //Eigen::Vector3d::UnitX()
    //static const Eigen::Vector3d gVecAxisY(0, 1, 0); //Eigen::Vector3d::UnitY()
    //static const Eigen::Vector3d gVecAxisZ(0, 0, 1); //Eigen::Vector3d::UnitZ()
    static constexpr double eps = FLT_EPSILON; //1e-7
    static constexpr double _eps = -FLT_EPSILON;
    //static constexpr unsigned long long ULL_MAX = 18446744073709551615; // 2 ^ 64 - 1 //ULLONG_MAX

    class Plane3d
    {
    public:
        Eigen::Vector3d m_origin;
        Eigen::Vector3d m_normal;
        Plane3d()
        {
            m_origin = Eigen::Vector3d::Zero();
            m_normal = Eigen::Vector3d::UnitZ(); //default plane XoY
        }
        Plane3d(const Eigen::Vector3d& origin, const Eigen::Vector3d& normal)
        {
            m_origin = origin;
            m_normal = (normal.isApprox(Eigen::Vector3d::Zero(), 0)) ? gVecNaN : normal;
        }
        //get the plane origin through world coordinate origin
        const Eigen::Vector3d& origin() const
        {
            double o_n = m_origin.dot(m_normal);
			return (fabs(o_n) < eps) ? m_origin : o_n / m_normal.squaredNorm() * m_normal;
        }
        //get normalized vector
        const Eigen::Vector3d& normal() const
        {
			if (fabs(m_normal.squaredNorm() - 1) < eps)
                return m_normal;
            return m_normal.normalized();
        }

    };

    // equal BPEntityId
    struct UnifiedIdentify
    {
        int32_t mid; //PModelId=-2; 
        uint64_t eid; //PEntityId=0; 
    };

    enum class RelationOfTwoTriangles : int //two intersect triangle
    {
        COPLANAR = 0,   //intersect or separate
        CONTACT,        //intersect but depth is zero
        INTRUSIVE,      //sat intersect all
        //PARALLEL,       //but not coplanr, must separate
        //SEPARATE,
    };

    enum class RelationOfTrigon : int
    {
        SEPARATE = 0,
        INTERSECT, //intersect point local one or two trigon
        COPLANAR_AINB, //A_INSIDE_B
        COPLANAR_BINA, //B_INSIDE_A
        COPLANAR_INTERSECT,
    };

    enum class RelationOfPointAndMesh : int
    {
        SURFACE = 0,
        INNER,
        OUTER,
    };

    enum class RelationOfTwoMesh : int
    {
        SEPARATE = 0,
        INTRUSIVE, //d>0
        CONTACT_OUTER, //d==0
        INSEDE_AINB, //total inside
        INSEDE_AINB_CONT, // partly cont 
        INSEDE_AINB_FIT, //all vertex cont
        INSEDE_BINA,
        INSEDE_BINA_CONT,
        INSEDE_BINA_FIT,
    };

    enum class RelationOfRayAndTrigon : int
    {
        CROSS_OUTER = 0,
        CROSS_INNER,
        CROSS_VERTEX_0,
        CROSS_VERTEX_1,
        CROSS_VERTEX_2,
        CROSS_EDGE_01,
        CROSS_EDGE_12,
        CROSS_EDGE_20,
        COIN_EDGE_01, //collinear
        COIN_EDGE_12,
        COIN_EDGE_20,
    };

}
