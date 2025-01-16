#pragma once
namespace clash
{
    class ClashDetection
    {
    public:
        static std::vector<std::pair<int, int>> executeFullClashDetection(const std::vector<ModelMesh>& meshVct);
        static std::vector<std::pair<int, int>> executeClashDetection(const std::vector<ModelMesh>& meshsLeft, const std::vector<ModelMesh>& meshsRight);

    };


}