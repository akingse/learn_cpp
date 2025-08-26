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

    static constexpr double epsF = FLT_EPSILON; //epsFloat=1e-7
    static constexpr double epsArea = 100 * FLT_EPSILON;
    //static constexpr double epsA = 1e-6;
    //static constexpr unsigned long long ULL_MAX = 18446744073709551615; // 2 ^ 64 - 1 //ULLONG_MAX

    struct TriMesh //trigon mesh, simplified
    {
        std::vector<Eigen::Vector3d> vbo_;
        std::vector<Eigen::Vector3i> ibo_;
        std::vector<Eigen::Vector3d> fno_; //Face Normal
        Eigen::AlignedBox3d bounding_;
        //Eigen::Affine3d pose_ = Eigen::Affine3d::Identity();
        int index_ = -1; //int type index
        //bool convex_ = true; // isConvex default true
        bool convex_ = true;
    };

    inline TriMesh operator*(const Eigen::Matrix4d& mat, const TriMesh& mesh)
    {
        TriMesh res = mesh;
        Eigen::AlignedBox3d box; //re calculate
        for (int i = 0; i < (int)mesh.vbo_.size(); ++i)
        {
            res.vbo_[i] = (mat * mesh.vbo_[i].homogeneous()).hnormalized();
            box.extend(res.vbo_[i]);
        }
        for (int i = 0; i < (int)mesh.fno_.size(); ++i)
            res.fno_[i] = (mat * mesh.fno_[i].homogeneous()).hnormalized();
        res.bounding_ = box;
        return res;
    }

    struct ModelMesh //TriMesh
    {
        std::vector<Eigen::Vector3d> vbo_;
#ifdef STORAGE_VERTEX_DATA_2D
        std::vector<Eigen::Vector2d> vbo2_; //using for 2d
#endif
        std::vector<Eigen::Vector3i> ibo_; //array<int, 3>
        std::vector<Eigen::Vector3d> fno_; //Face Normal
        Eigen::AlignedBox3d bounding_;
        Eigen::Affine3d pose_ = Eigen::Affine3d::Identity();
        bool convex_ = true; // isConvex default true
        int genus_ = 0; //number of genus, default 0
        int number_ = -1; //int type index
#ifdef FILL_PROFILE_DEBUG_TEMP
        std::vector<int> iboRaw_; //for test debug
        uint64_t index_ = UINT64_MAX;// ULLONG_MAX; // record belong to same polyface
        //uint64_t instanceid = 0; //0 means not instance
        double area_ = 0;
        double volume_ = 0;
#endif
#ifdef STORAGE_VERTEX_DATA_2D
        inline void to2d()
        {
            vbo2_.resize(vbo_.size());
            for (int i = 0; i < vbo2_.size(); ++i)
                vbo2_[i] = Eigen::Vector2d(vbo_[i][0], vbo_[i][1]);
        }
#endif
        inline operator TriMesh() const
        {
            TriMesh mesh;
            mesh.vbo_ = vbo_;
            mesh.ibo_ = ibo_;
            mesh.fno_ = fno_;
            mesh.bounding_ = bounding_;
            mesh.index_ = number_;
            mesh.convex_ = convex_;
            return mesh;
        }
        inline TriMesh toTriMesh() const
        {
            return operator TriMesh();
        }
        static std::vector<TriMesh> toTriMeshs(const std::vector<ModelMesh>& meshVct)
        {
            std::vector<TriMesh> meshRes(meshVct.size());
            for (int i = 0; i < (int)meshVct.size(); ++i)
                meshRes[i] = meshVct[i]; //operator
            return meshRes;
        }
        inline bool empty() const
        {
            return vbo_.empty() || ibo_.empty();
        }
        inline double area() const
        {
            double area = 0;
            for (int i = 0; i < (int)ibo_.size(); ++i)
            {
                area += (vbo_[ibo_[i][1]] - vbo_[ibo_[i][0]]).cross
                        (vbo_[ibo_[i][2]] - vbo_[ibo_[i][1]]).norm();
            }
            return 0.5 * area;
        }
        inline double volume() const
        {
            //Eigen::Vector3d centroid(0, 0, 0);
            //for (const auto& vertex : vbo_) 
            //    centroid += vertex;
            //centroid = 1.0 / vbo_.size() * centroid;
            if (vbo_.empty())
                return 0;
            Eigen::Vector3d origin = vbo_[0];
            //tetrahedronVolume
            double volume = 0;
            for (int i = 0; i < (int)ibo_.size(); ++i)
            {
                volume += (vbo_[ibo_[i][0]] - origin).dot((vbo_[ibo_[i][1]] - origin).cross(vbo_[ibo_[i][2]] - origin));
            }
            return volume / 6.0;
        }

        inline std::pair<double, Eigen::Vector3d> volume_moment() const
        {
            if (vbo_.empty())
                return { 0, Eigen::Vector3d(0,0,0) };
            Eigen::Vector3d origin = vbo_[0];
            //tetrahedronVolume
            double volume = 0;
            Eigen::Vector3d moment = Eigen::Vector3d(0, 0, 0);
            for (int i = 0; i < (int)ibo_.size(); ++i)
            {
                const Eigen::Vector3d vector0 = vbo_[ibo_[i][0]] - origin;
                const Eigen::Vector3d vector1 = vbo_[ibo_[i][1]] - origin;
                const Eigen::Vector3d vector2 = vbo_[ibo_[i][2]] - origin;
                double det = (vector0).dot((vector1).cross(vector2));
                volume += det;
                moment += det * (vector0 + vector1 + vector2);
            }
            return { volume / 6.0,moment / 24.0 };
        }

        bool operator==(const ModelMesh& rhs)const
        {
            if (vbo_.size() != rhs.vbo_.size() || ibo_.size() != rhs.ibo_.size())
                return false;
            double areaThis = area();
            double voluThis = volume();
            double areaRhs = rhs.area();
            double voluRhs = rhs.volume();
            if (1 < fabs(areaThis - areaRhs) || 1 < fabs(voluThis - voluRhs))
                return false;
            return true;
        }

        static bool isEqualMesh(const ModelMesh& meshA, const ModelMesh& meshB)
        {
            if (meshA.ibo_.size() != meshB.ibo_.size() || meshA.vbo_.size() != meshB.vbo_.size())
                return false;
            if (!meshA.bounding_.isApprox(meshB.bounding_))
                return false;
            if (meshA.ibo_.empty() || meshA.vbo_.empty() || meshB.ibo_.empty() || meshB.vbo_.empty())
                return false;
            if (meshA.ibo_.front() != meshB.ibo_.front())
                return false;
            if (!meshA.vbo_.front().isApprox(meshB.vbo_.front()))
                return false;
            constexpr double toleFixed = 1e-8;
            std::pair<double, Eigen::Vector3d> vmA = meshA.volume_moment();
            std::pair<double, Eigen::Vector3d> vmB = meshB.volume_moment();
            if (!vmA.second.isApprox(vmB.second))
                return false;
            if (toleFixed < fabs(vmA.first - vmB.first))
                return false;
            if (toleFixed < fabs(meshA.area() - meshB.area()))
                return false;
            return true;
        }
        static bool isSameMesh(const ModelMesh& meshA, const ModelMesh& meshB)
        {
            if (meshA.ibo_.size() != meshB.ibo_.size() || meshA.vbo_.size() != meshB.vbo_.size())
                return false;
            if (!meshA.bounding_.isApprox(meshB.bounding_))
                return false;
            if (!meshA.vbo_.empty() && meshA.vbo_.front() != meshB.vbo_.front())
                return false;
            for (int i = 0; i < (int)meshA.ibo_.size(); i++)
            {
                if (meshA.ibo_[i] != meshB.ibo_[i])
                    return false;
            }
            return true;
        }

    };

    inline TriMesh toTriMesh(const ModelMesh& mesh)
    {
        TriMesh res;
        res.vbo_ = mesh.vbo_;
        res.ibo_ = mesh.ibo_;
        res.fno_ = mesh.fno_;
        res.bounding_ = mesh.bounding_;
        res.index_ = mesh.number_;
        res.convex_ = mesh.convex_;
        return res;
    }

    inline ModelMesh toModelMesh(const TriMesh& mesh)
    {
        ModelMesh res;
        res.vbo_ = mesh.vbo_;
        res.ibo_ = mesh.ibo_;
        res.fno_ = mesh.fno_;
        res.bounding_ = mesh.bounding_;
        res.number_ = mesh.index_;
        res.convex_ = mesh.convex_;
        return res;
    }

    //static const ModelMesh gMeshEmpty = {};

}

namespace clash //collide //psykronix
{
    // global type and variable define 
    typedef std::array<Eigen::Vector3d, 2> Segment;
    typedef std::array<Eigen::Vector2d, 2> Segment2d;
    typedef std::array<Eigen::Vector3d, 2> Segment3d; //can be line, two points form
    typedef std::array<Eigen::Vector3d, 3> Triangle;
    typedef std::array<Eigen::Vector2d, 3> Triangle2d;
    typedef std::array<Eigen::Vector3d, 3> Triangle3d;//can be plane, three points form
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
        UNKNOWN,
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
        //not using
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
