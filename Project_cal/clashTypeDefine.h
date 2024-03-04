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
//#ifdef CLASH_DETECTION_DEBUG_TEMP
    std::vector<int> iboRaw_; //for test debug
    uint64_t index_ = ULLONG_MAX; // record belong to same polyface
    //uint64_t instanceid = 0; //0 means not instance
//#endif
};
static const ModelMesh gMeshEmpty = {};

namespace clash //collide //psykronix
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
    static constexpr double epsF = FLT_EPSILON; //1e-7
    static constexpr double epsA = 100 * FLT_EPSILON;
    static constexpr double _epsF = -FLT_EPSILON;
    //static constexpr double epsA = 1e-6;
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
            double o_n = m_origin.dot(normal());
            double maxCoor = 0;
            for (int i = 0; i < 3; ++i)
                maxCoor = std::max(fabs(m_origin[i]), maxCoor); // maxCoeff
            return (fabs(o_n) < epsF * maxCoor) ? m_origin : o_n / m_normal.squaredNorm() * m_normal;
        }
        //get normalized vector
        Eigen::Vector3d normal() const
        {
			//if (fabs(m_normal.squaredNorm() - 1) < epsF)
   //             return m_normal;
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

enum class OcclusionState :int //means cover
{
    EXPOSED = 0,
    HIDDEN,
    SHIELDED, //shielded by other triangle
    COPLANAR, //COPLANAR with other-triangle
    INTERSECT, //ignore
    OCCLUSION, //shielded+intersect
    DEGENERACY, // become segment
    UNKNOWN,
};

enum class FrontState :int
{
    // state of 3d trigon, all 2d projection penetration
    COPLANAR = 0, //and intersect
    A_FRONTOF,
    B_FRONTOF,
    INTERSECT, //3d intersect
    UNKNOWN,
};

namespace eigen
{
    typedef std::vector<std::vector<Eigen::Vector2d>> ContourProfile;
    struct TrigonPart
    {
        long long m_index;
        OcclusionState m_visible = OcclusionState::EXPOSED;
        double m_area = -1;
        std::array<int, 3> m_number; // mesh index | triangle index | graphic index
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
            return m_index < rhs.m_index;
#ifdef CLASH_DETECTION_DEBUG_TEMP
            //return m_area < rhs.m_area;
#endif
        }
    };

    struct ContourPart
    {
        long long m_index; // mesh index
        int m_number; // graphic index
		//uint64_t m_entityid = -2; // record belong to same polyface
        OcclusionState m_visible = OcclusionState::EXPOSED;
        Eigen::AlignedBox3d m_box3d; //to judge front
        Eigen::AlignedBox2d m_box2d; // contained in box3d
        double m_area = -1;
        std::vector<long long> m_shielded;
        std::vector<std::vector<Eigen::Vector2d>> m_profile; //fillArea, the boolean result
        std::vector<std::vector<Eigen::Vector2d>> m_contour; //contour of mesh
        std::vector<TrigonPart> m_trigons; //to judge front
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
        std::vector<Eigen::Vector2d> getOuter() const
        {
            if (m_contour.empty())
                return {};
            return m_contour[0];
        }
    };

}
