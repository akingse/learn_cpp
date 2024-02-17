#pragma once

struct ModelMesh
{
    std::vector<Eigen::Vector3d> vbo_;
    std::vector<std::array<int, 3>> ibo_;
    std::vector<Eigen::Vector3d> fno_; //Face Normal
    Eigen::AlignedBox3d bounding_;
    Eigen::Affine3d pose_; // Eigen::Affine3d::Identity()
    bool convex_; // isConvex default true
    int genus_ = 0; //number of genus, default 0
    std::vector<int> iboRaw_; //for test debug
    uint64_t entityid_;
    //uint64_t instanceid_;
};

namespace psykronix
{
    // global type and variable define 
    typedef std::array<Eigen::Vector3d, 2> Segment;
    typedef std::array<Eigen::Vector2d, 2> Segment2d;
    typedef std::array<Eigen::Vector3d, 2> Segment3d;
    typedef std::array<Eigen::Vector3d, 3> Triangle;
    typedef std::array<Eigen::Vector2d, 3> Triangle2d;
    typedef std::array<Eigen::Vector3d, 3> Triangle3d;
    typedef std::array<Eigen::Vector3d, 2> PosVec3d;
    typedef std::tuple<std::vector<Eigen::Vector3d>, std::vector<std::array<int, 3>>> Polyhedron;
    //global constexpr
    static const Eigen::Vector3d gVecNaN(std::nan("0"), std::nan("0"), std::nan("0"));
    static const Triangle gSegNaN = { gVecNaN, gVecNaN };
    static const Triangle gTirNaN = { gVecNaN, gVecNaN, gVecNaN };
    static const PosVec3d gPVNaN = { gVecNaN ,gVecNaN };
    //static const Eigen::Vector3d gVecZero = Eigen::Vector3d::Zero();// Vector3d(0, 0, 0);
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
        Eigen::Vector3d origin() const
        {
            double o_n = m_origin.dot(m_normal);
			return (fabs(o_n) < eps) ? m_origin : o_n / m_normal.squaredNorm() * m_normal;
        }
        //get normalized vector
        Eigen::Vector3d normal() const
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

//for Hidden-line removal
enum class OcclusionStatus :int //means cover
{
    EXPOSED = 0,
    HIDDEN,
    SHIELDED, //shielded by other triangle
    INTERSECT,
    OCCLUSION, //shielded+intersect
    DEGENERACY, // become segment
};

namespace eigen
{
    struct TrigonPart
    {
#ifdef CLASH_DETECTION_DEBUG_TEMP
        double m_area = -1;
#endif
        long long m_index;
        std::array<int, 3> m_number; // mesh index | triangle index | graphic index
        OcclusionStatus m_visible = OcclusionStatus::EXPOSED;
        Eigen::AlignedBox3d m_box3d;
        Eigen::AlignedBox2d m_box2d;
        Eigen::Vector3d m_normal; //normal of m_triangle3d, always upward
        std::array<Eigen::Vector3d, 3> m_triangle3d;
        std::array<Eigen::Vector2d, 3> m_triangle2d;
        // profile boolean operation
        //std::vector<std::array<int, 2>> m_intersect;
        //std::vector<std::array<int, 2>> m_shielded;
        std::vector<long long> m_shielded;
        std::vector<std::vector<Eigen::Vector2d>> m_contour; //the profile of this trigon after clipper
        bool operator<(const TrigonPart& rhs) const
        {
#ifdef CLASH_DETECTION_DEBUG_TEMP
            return m_area < rhs.m_area;
#else
            return m_index < rhs.m_index;
#endif
        }
    };

    struct ContourPart
    {
#ifdef CLASH_DETECTION_DEBUG_TEMP
        double m_area = -1;
#endif
        long long m_index; // mesh index
		//uint64_t m_entityid = -2; // record belong to same polyface
        Eigen::AlignedBox3d m_box3d; //to judge front
        Eigen::AlignedBox2d m_box2d; // contained in box3d
        std::vector<long long> m_shielded;
        std::vector<std::vector<Eigen::Vector2d>> m_contour; //contour of mesh
        std::vector<std::vector<Eigen::Vector2d>> m_profile; //fillArea, the bool result
        std::vector<TrigonPart> m_trigons; //to judge front
        OcclusionStatus m_visible = OcclusionStatus::EXPOSED;
        // for contour calculate method
        bool operator<(const ContourPart& rhs) const
        {
#ifdef CLASH_DETECTION_DEBUG_TEMP
            return m_area < rhs.m_area;
#else
            return m_index < rhs.m_index;
#endif
        }
        bool isEmpty() const
        {
            return m_contour.empty();
        }
        const std::vector<Eigen::Vector2d>& getOuter() const
        {
            return m_contour[0];
        }
    };

}
