#pragma once
namespace clash
{
    class ClashDetection
    {
        struct ClashInfo
        {
            std::pair<int, int> m_indexpair;      
            Eigen::AlignedBox3d m_boundbox;    
            double m_distance = 0.0;           
            std::vector<Eigen::Vector3d> m_points;
        };

    private:
        static std::vector<ModelMesh> sm_meshStore;

    public:
        static void addMeshs(const std::vector<ModelMesh>& meshVct);
        static void clearMeshs();
        static std::vector<std::pair<int, int>> executeAssignClashDetection(const std::vector<int>& indexesLeft, const std::vector<int>& indexesRight, 
			const double tolerance = 0.0, const std::function<bool(float, int)>& callback = nullptr);

    public:
        /// <summary>
        /// self whole clash detection
        /// </summary>
        /// <param name="meshVct"></param>
        /// <param name="tolerance"></param>
        /// <returns> the index pair of mesh </returns>
        static std::vector<std::pair<int, int>> executeFullClashDetection(const std::vector<ModelMesh>& meshVct, 
			const double tolerance = 0.0, const std::function<bool(float, int)>& callback = nullptr);
        static std::vector<std::pair<int, int>> executePairClashDetection(const std::vector<ModelMesh>& meshsLeft, const std::vector<ModelMesh>& meshsRight, 
            const double tolerance = 0.0, const std::function<bool(float, int)>& callback = nullptr);

    };
}