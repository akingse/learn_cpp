#pragma once
namespace clash
{
    /// <summary>
    /// self full clash detection of whole trimesh
    /// </summary>
    /// <param name="meshVct"></param>
    /// <param name="tolerance"></param>
    /// <returns> the index pair of mesh </returns>
    class ClashDetection //DLLEXPORT_CAL
    {
    private:
        static std::vector<TriMesh> sm_meshStore;

    public:
        static void addMeshs(const std::vector<TriMesh>& meshVct)
        {
            sm_meshStore.insert(sm_meshStore.end(), meshVct.begin(), meshVct.end()); //copy data
        }
        static void clearMeshs()
        {
            sm_meshStore.clear();
        }
        //using index
        static std::vector<std::pair<int, int>> executeAssignClashDetection(const std::vector<int>& indexesLeft, const std::vector<int>& indexesRight, 
			const double tolerance = 0.0, const std::function<bool(float, int)>& callback = nullptr);

    public:
        //using meshs
        static std::vector<std::pair<int, int>> executeFullClashDetection(const std::vector<TriMesh>& meshVct,
			const double tolerance = 0.0, const std::function<bool(float, int)>& callback = nullptr);
        static std::vector<std::pair<int, int>> executePairClashDetection(const std::vector<TriMesh>& meshsLeft, const std::vector<TriMesh>& meshsRight,
            const double tolerance = 0.0, const std::function<bool(float, int)>& callback = nullptr);

        //using trigon tree
        static std::vector<std::pair<int, int>> executeFullClashDetectionByTrigon(const std::vector<TriMesh>& meshVct,
            const double tolerance = 0.0, const std::function<bool(float, int)>& callback = nullptr);

#if 0
        //overload
        static void addMeshs(const std::vector<ModelMesh>& meshVct)
        {
            std::vector<TriMesh> triMeshVct(meshVct.size());
            for (int i = 0; i < (int)meshVct.size(); ++i)
                triMeshVct[i] = meshVct[i].toTriMesh();
            addMeshs(triMeshVct);
        }
        static std::vector<std::pair<int, int>> executeFullClashDetection(const std::vector<ModelMesh>& meshVct,
            const double tolerance = 0.0, const std::function<bool(float, int)>& callback = nullptr)
        {
            std::vector<TriMesh> triMeshVct(meshVct.size());
            for (int i = 0; i < (int)meshVct.size(); ++i)
                triMeshVct[i] = meshVct[i].toTriMesh();
            return executeFullClashDetection(triMeshVct, tolerance, callback);
        }
        static std::vector<std::pair<int, int>> executePairClashDetection(const std::vector<ModelMesh>& meshsLeft, const std::vector<ModelMesh>& meshsRight,
            const double tolerance = 0.0, const std::function<bool(float, int)>& callback = nullptr)
        {
            std::vector<TriMesh> triMeshsLeft(meshsLeft.size());
            for (int i = 0; i < (int)meshsLeft.size(); ++i)
                triMeshsLeft[i] = meshsLeft[i].toTriMesh();
            std::vector<TriMesh> triMeshsRight(meshsRight.size());
            for (int i = 0; i < (int)meshsRight.size(); ++i)
                triMeshsRight[i] = meshsRight[i].toTriMesh();
            return executePairClashDetection(triMeshsLeft, triMeshsRight, tolerance, callback);
        }
#endif

    };

}

namespace sat //to distinguish clash
{
    //positive tolerance means magnify
    inline Eigen::AlignedBox3d boxExtendTolerance(const Eigen::AlignedBox3d& box, const double tolerance)
    {
        Eigen::Vector3d toleSize = Eigen::Vector3d(tolerance, tolerance, tolerance);
        return Eigen::AlignedBox3d(box.min() - toleSize, box.max() + toleSize);
    }
    DLLEXPORT_CAL bool isPointOnTriangleSurface(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon, const Eigen::Vector3d& normal);
    DLLEXPORT_CAL bool isPointInTriangle(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon, const Eigen::Vector3d& normal); //3D
    DLLEXPORT_CAL bool isMeshInsideOtherMesh(const clash::TriMesh& meshIn, const clash::TriMesh& meshOut);
    DLLEXPORT_CAL bool isTwoTrianglesIntrusionSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB, double tolerance = 0.0);
    DLLEXPORT_CAL bool isTriangleAndBoundingBoxIntersectSAT(const std::array<Eigen::Vector3d, 3>& trigon, const Eigen::AlignedBox3d& box);
    DLLEXPORT_CAL bool isTwoTrianglesBoundingBoxIntersect(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB, double tolerance = 0.0);
    DLLEXPORT_CAL std::array<std::vector<int>, 2> trianglesAndCommonBoxPreclash(const clash::TriMesh& meshA, const clash::TriMesh& meshB, double tolerance);
    DLLEXPORT_CAL bool isTwoMeshsIntersectSAT(const clash::TriMesh& meshA, const clash::TriMesh& meshB, double tolerance = 0.0);

}
