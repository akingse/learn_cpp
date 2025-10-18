#include "pch.h"
using namespace std;
using namespace clash;
using namespace eigen;
//using namespace land;

std::array<std::vector<Eigen::Vector3d>, 4> clash::splitContourToEdge(
    const std::vector<Eigen::Vector3d>& boundContour, const std::array<Eigen::Vector2d, 4>& cornerPoints, bool isFirst /*= false*/)
{
    std::map<double, int> angleMap; //夹角排序，轮廓中有的地方夹角很大
    if (isFirst)
    {
        int n = (int)boundContour.size();
        double minCorner = 1;
        for (int i = 0; i < n; i++)
        {
            int h = (i == 0) ? n - 1 : i - 1; //last
            int j = (i + 1) % n; //next
            Eigen::Vector3d veci = boundContour[i] - boundContour[h];
            Eigen::Vector3d vecj = boundContour[j] - boundContour[i];
            double angle = eigen::angle_two_vectors(to_vec3(to_vec2(veci)), to_vec3(to_vec2(vecj)));
            if (angle < minCorner)
                continue;
            angleMap.emplace(angle, i);
        }
        if (angleMap.size() < 4)
            return {};
    }
    int firstpoint = 0;
    double distance = DBL_MAX;
    if (isFirst)
        for (auto iter = angleMap.begin(); iter != angleMap.end(); iter++)
        {
            double temp = (cornerPoints[0] - to_vec2(boundContour[iter->second])).squaredNorm();
            if (temp < distance)
            {
                distance = temp;
                firstpoint = iter->second;
            }
        }
    else
        for (int j = 0; j < (int)boundContour.size(); j++)
        {
            double temp = (cornerPoints[0] - to_vec2(boundContour[j])).squaredNorm();
            if (temp < distance)
            {
                distance = temp;
                firstpoint = j;
            }
        }
    //std::vector<Eigen::Vector3d> ccwContour = boundContour;
    std::vector<Eigen::Vector3d> ccwContour(boundContour.size());
    bool isCCW = isContourCCW(boundContour);
    int n = (int)boundContour.size();
    for (int i = 0; i < n; i++)
        ccwContour[i] = boundContour[(i + firstpoint) % n];
    //ccwContour[n - 1] = ccwContour[0]; //closed, for reverse
    if (!isCCW)
    {
        ccwContour.push_back(ccwContour.front());
        std::reverse(ccwContour.begin(), ccwContour.end());
        ccwContour.pop_back();
    }
    std::array<int, 4> cornerIndex = { 0,0,0,0 };
    for (int i = 1; i < 4; i++)
    {
        double distance = DBL_MAX;
        if (isFirst)
            for (auto iter = angleMap.begin(); iter != angleMap.end(); iter++)
            {
                // angleMap record origin index
                double temp = (cornerPoints[i] - to_vec2(boundContour[iter->second])).squaredNorm();
                if (temp < distance)
                {
                    distance = temp;
                    cornerIndex[i] = (iter->second + n - firstpoint) % n; //avoid nega
                    if (!isCCW)
                        cornerIndex[i] = n - cornerIndex[i];
                }
            }
        else
            for (int j = 0; j < (int)ccwContour.size(); j++)
            {
                double temp = (cornerPoints[i] - to_vec2(ccwContour[j])).squaredNorm();
                if (temp < distance)
                {
                    if (j <= cornerIndex[i - 1])
                        continue;
                    distance = temp;
                    cornerIndex[i] = j;
                }
            }
    }
    //left - right - down - up
    std::array<std::vector<Eigen::Vector3d>, 4> edgeUV4;
    //int countLess = 0;
    //for (int i = 0; i < 4; i++)
    //{
    //    if (cornerIndex[i] < cornerIndex[(i + 1) % 4])
    //        countLess++;
    //}
    //bool isCCW = countLess == 3;
    for (int i = 0; i < 4; i++)
    {
        int start = cornerIndex[i];
        int end = cornerIndex[(i + 1) % 4];
        std::vector<Eigen::Vector3d> edge;
        if (start < end)
        {
            for (int j = start; j <= end; j++)
                edge.push_back(ccwContour[j]);
        }
        else
        {
            for (int j = start; j < ccwContour.size(); j++)
                edge.push_back(ccwContour[j]);
            for (int j = 0; j <= end; j++)
                edge.push_back(ccwContour[j]);
        }
        if (i == 2 || i == 3)//(!isCCW && (i == 0 || i == 1)))
            std::reverse(edge.begin(), edge.end());
        edgeUV4[i] = edge;
    }
    return edgeUV4;
}

#define TEST_TERRAINMESH
#ifdef TEST_TERRAINMESH
static int enrol = []()->int
    {
        //testTerrainMesh_UVline_0();
        cout << "test_model_mesh finished.\n" << endl;
        return 0;
    }();

int main()
{
    return 0;
}
#endif // TEST_TERRAINMESH
