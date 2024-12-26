#pragma once
/*******************************************************************************
* Editor    :  akingse		                                                   *
* Date      :  from June 2023												   *
* Website   :  https://github.com/akingse                                      *
* Copyright :  All rights reserved											   *
* Purpose   :  Some geometry class and relation enum type define               *
* License   :  MIT									                           *
*******************************************************************************/
#ifndef CALCULATE_TYPEDEFINE_H
#define CALCULATE_TYPEDEFINE_H

namespace clash
{
    static constexpr size_t N_10E_3 = (size_t)1e3;
    static constexpr size_t N_10E_4 = (size_t)1e4;
    static constexpr size_t N_10E_5 = (size_t)1e5;
    static constexpr size_t N_10E_6 = (size_t)1e6;
    static constexpr size_t N_10E_7 = (size_t)1e7;
    static constexpr size_t N_10E_8 = (size_t)1e8;

    struct ModelMesh
    {
        std::vector<Eigen::Vector3d> vbo_;
#ifdef STORAGE_VERTEX_DATA_2D
        std::vector<Eigen::Vector2d> vbo2_; //using for 2d
#endif
        std::vector<Eigen::Vector3i> ibo_; //array<int, 3>
        std::vector<Eigen::Vector3d> fno_; //Face Normal
        Eigen::AlignedBox3d bounding_;
        Eigen::Affine3d pose_ = Eigen::Affine3d::Identity();
        bool convex_; // isConvex default true
        int genus_ = 0; //number of genus, default 0
        //#ifdef FILL_PROFILE_DEBUG_TEMP
        std::vector<int> iboRaw_; //for test debug
        uint64_t index_ = UINT64_MAX;// ULLONG_MAX; // record belong to same polyface
        //uint64_t instanceid = 0; //0 means not instance
    //#endif
#ifdef STORAGE_VERTEX_DATA_2D
        inline void to2d()
        {
            vbo2_.resize(vbo_.size());
#pragma omp parallel for schedule(dynamic)
            for (int i = 0; i < vbo2_.size(); ++i)
                vbo2_[i] = Eigen::Vector2d(vbo_[i][0], vbo_[i][1]);
        }
#endif
    };
    //static const ModelMesh gMeshEmpty = {};

    //convert easy
    struct Polyface //equal PolyfaceHandle
    {
        std::vector<Eigen::Vector3d> m_point;
        std::vector<int> m_pointIndex; //divide by zero
    };

}

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
    typedef std::array<Eigen::Vector3d, 2> RayLine;
    typedef std::tuple<std::vector<Eigen::Vector3d>, std::vector<std::array<int, 3>>> Polyhedron;
    //typedef std::array<Eigen::Vector2d, 3> TrigonEigen; same as Triangle2d
    typedef std::vector<std::vector<Eigen::Vector2d>> PathsEigen; //ContourProfile
    typedef std::vector<std::vector<Eigen::Vector3d>> PathsEigen3d;

    //global constexpr
    static const Eigen::Vector2d gVecNaN2d(std::nan("0"), std::nan("0"));
    static const Eigen::Vector3d gVecNaN(std::nan("0"), std::nan("0"), std::nan("0"));
    static const Triangle gSegNaN = { gVecNaN, gVecNaN };
    static const Triangle gTirNaN = { gVecNaN, gVecNaN, gVecNaN };
    static const PosVec3d gPVNaN = { gVecNaN ,gVecNaN };
    //static const Eigen::Vector3d gVecZero = Eigen::Vector3d::Zero();// Vector3d(0, 0, 0);
    //static const Eigen::Vector3d gVecAxisX(1, 0, 0); //Eigen::Vector3d::UnitX()
    //static const Eigen::Vector3d gVecAxisY(0, 1, 0); //Eigen::Vector3d::UnitY()
    //static const Eigen::Vector3d gVecAxisZ(0, 0, 1); //Eigen::Vector3d::UnitZ()
    static constexpr double epsF = FLT_EPSILON; //epsFloat=1e-7
    static constexpr double epsArea = 100 * FLT_EPSILON;
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
        Plane3d(const std::array<Eigen::Vector3d, 3>& triangle)
        {
            m_origin = triangle[0];
            m_normal = (triangle[1] - triangle[0]).cross(triangle[2] - triangle[1]);//without normalize
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

namespace eigen
{
    //fill profile alg
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
        USING_CUDA,
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

    struct TrigonPart
    {
        int m_index;
        OcclusionState m_visible = OcclusionState::EXPOSED;
        double m_area = -1.0;
        std::array<int, 4> m_number; // mesh index | triangle index | graphic index | polyface index in graphic container
        Eigen::AlignedBox3d m_box3d;
        Eigen::AlignedBox2d m_box2d;
        Eigen::Vector3d m_normal; //normal of m_triangle3d, always upward
        std::array<Eigen::Vector3d, 3> m_triangle3d;
        std::array<Eigen::Vector2d, 3> m_triangle2d;
        // profile boolean operation
        //std::vector<std::array<int, 2>> m_intersect;
        //std::vector<std::array<int, 2>> m_shielded;
        std::vector<int> m_shielded;
        std::vector<std::vector<Eigen::Vector2d>> m_contour; //the profile of this trigon after clipper
        std::vector<int> m_preInter; //for cuda calculate
        bool operator<(const TrigonPart& rhs) const
        {
            return m_index < rhs.m_index;
#ifdef FILL_PROFILE_DEBUG_TEMP
            //return m_area < rhs.m_area;
#endif
        }
#ifdef FILL_PROFILE_DEBUG_TEMP
        double min_angle = 1.0; //rad
#endif
    };

    struct ContourPart
    {
        int m_index; // mesh index, global unique
        int m_number; // graphic index
		//uint64_t m_entityid = -2; // record belong to same polyface
        OcclusionState m_visible = OcclusionState::EXPOSED;
        Eigen::AlignedBox3d m_box3d; //to judge front
        Eigen::AlignedBox2d m_box2d; // contained in box3d
        double m_area = -1.0;
        std::vector<int> m_shielded;
        std::vector<std::vector<Eigen::Vector2d>> m_profile; //fillArea, the boolean result
        std::vector<std::vector<Eigen::Vector2d>> m_contour; //contour of mesh //current useless
        std::vector<TrigonPart> m_trigons; //to judge front
        // for contour calculate method
        bool operator<(const ContourPart& rhs) const
        {
#ifdef FILL_PROFILE_DEBUG_TEMP
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
#endif// CALCULATE_TYPEDEFINE_H
