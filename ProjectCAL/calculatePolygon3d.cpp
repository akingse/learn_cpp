#include "pch.h"
using namespace std;
using namespace clash;
using namespace land;


//compute by ibo index count
std::pair<std::vector<Eigen::Vector3d>, std::unordered_set<int>> computeBoundaryEdges(const ModelMesh& mesh)
{
    std::unordered_map<Edge, int> edgeCount;
    // count every edge
    for (const auto& triangle : mesh.ibo_)
    {
        std::array<Edge, 3> edges = {
            Edge(triangle[0], triangle[1]),
            Edge(triangle[1], triangle[2]),
            Edge(triangle[2], triangle[0])
        };
        for (const auto& edge : edges)
            edgeCount[edge]++;
    }
    // collect
    std::vector<Edge> boundEdges;
    std::unordered_set<int> uniqueEdge;
    for (const auto& pair : edgeCount)
    {
        if (pair.second == 1)
        {
            boundEdges.push_back(pair.first);
            uniqueEdge.insert(pair.first.v1);
            uniqueEdge.insert(pair.first.v2);
        }
    }
    //return boundEdges; //extractboundContour
    // 建立边界的邻接表
    std::unordered_map<int, std::vector<int>> adjacencyList;
    for (const auto& edge : boundEdges)
    {
        adjacencyList[edge.v1].push_back(edge.v2);
        adjacencyList[edge.v2].push_back(edge.v1);
    }
    std::vector<Eigen::Vector3d> boundContour;
    // 从一个顶点开始，遍历构建轮廓线
    int startVertex = boundEdges[0].v1; // 选择任意边的起始顶点
    int currentVertex = startVertex;
    int previousVertex = -1;
    std::vector<int> check;
    std::set<int> checkset;
    do {
        boundContour.push_back(mesh.vbo_[currentVertex]);
        const std::vector<int>& neighbors = adjacencyList[currentVertex];
        for (const int neighbor : neighbors)
        {
            if (neighbor != previousVertex) {
                previousVertex = currentVertex;
                currentVertex = neighbor;
                break;
            }
        }
        if (boundEdges.size() < boundContour.size())
            return {};// break;
    } while (currentVertex != startVertex);
    return { boundContour,uniqueEdge };
}

//copy all vbo, filter ibo
ModelMesh getInnerMeshByBoundary(const ModelMesh& mesh, const std::unordered_set<int>& boundEdge)
{
    //ModelMesh removeOuterEdges(const ModelMesh& mesh,const std::vector<Edge>& boundEdges)
    ModelMesh innMesh;
    innMesh.vbo_ = mesh.vbo_;
    ModelMesh outMesh;
    outMesh.vbo_ = mesh.vbo_;
    unordered_set<int> outContour;
    unordered_set<Edge> outEdges;
    for (const auto& face : mesh.ibo_)
    {
        bool isOut = false;
        for (const auto& i : face)
        {
            //if (std::find(boundEdges.begin(), boundEdges.end(), edge) != boundEdges.end())
            if (boundEdge.find(i) != boundEdge.end())
            {
                isOut = true;
                break;
            }
        }
        //bool indepen = outEdges.find(Edge(face[0], face[1])) == outEdges.end() ||
        //    outEdges.find(Edge(face[1], face[2])) == outEdges.end() ||
        //    outEdges.find(Edge(face[2], face[0])) == outEdges.end();
        //bool isInter = isOut && indepen &&(
        //    outContour.find(face[0]) != outContour.end() ||
        //    outContour.find(face[1]) != outContour.end() ||
        //    outContour.find(face[2]) != outContour.end());
        //if (isInter)
        //{
        //    array<Eigen::Vector3d, 3> trigon = {
        //        mesh.vbo_[face[0]],
        //        mesh.vbo_[face[1]],
        //        mesh.vbo_[face[2]] };
        //    //_drawTriangle(trigon, colorRed);
        //}
        if (isOut)
        {
            outMesh.ibo_.push_back(face);
            for (int i = 0; i < 3; i++)
            {
                outEdges.insert(Edge(face[i], face[(i + 1) % 3]));
                if (boundEdge.find(face[i]) == boundEdge.end())
                    outContour.insert(face[i]);
            }
        }
        else
        {
            innMesh.ibo_.push_back(face);
        }
    }
    //_drawPolyface(getPolyfaceHandleFromModelMesh(innMesh));
    //_drawPolyface(getPolyfaceHandleFromModelMesh(outMesh), colorRand());
    return innMesh;
}

std::array<std::vector<Eigen::Vector3d>, 4> splitFirstContourToEdge(
    const std::vector<Eigen::Vector3d>& boundContour, const std::array<Eigen::Vector2d, 4>& cornerPoints)
{
    std::map<double, int> angleMap; //夹角排序，轮廓中有的地方夹角很大
    int n = (int)boundContour.size();
    double minCorner = 1;
    for (int i = 0; i < boundContour.size(); i++)
    {
        int _i = (i == 0) ? n - 1 : i - 1;
        int i_ = (i + 1) % n;
        Eigen::Vector3d veci = boundContour[i] - boundContour[_i];
        Eigen::Vector3d vecj = boundContour[i_] - boundContour[i];
        double angle = angle_two_vectors(to_vec3(to_vec2(veci)), to_vec3(to_vec2(vecj)));
        if (angle < minCorner)
            continue;
        angleMap.emplace(angle, i);
    }
    if (angleMap.size() < 4)
        return {};
    std::array<int, 4> corner_index;
    for (int i = 0; i < 4; i++)
    {
        double distance = DBL_MAX;
        for (auto iter = angleMap.begin(); iter != angleMap.end(); iter++)
        {
            double temp = (cornerPoints[i] - to_vec2(boundContour[iter->second])).squaredNorm();
            if (temp < distance)
            {
                distance = temp;
                corner_index[i] = iter->second;
            }
        }
    }
    //for (int i = 0; i < 4; i++)
    //    _drawSolidBase(GeSphere(boundContour[corner_index[i]]));
    std::array<std::vector<Eigen::Vector3d>, 4> edgeUV;
    for (int i = 0; i < 4; i++)
    {
        int end = corner_index[i];
        int start = corner_index[(i + 1) % 4];
        std::vector<Eigen::Vector3d> temp;
        if (start < end)
        {
            for (int j = start; j <= end; j++)
                temp.push_back(boundContour[j]);
        }
        else
        {
            for (int j = start; j < boundContour.size(); j++)
                temp.push_back(boundContour[j]);
            for (int j = 0; j <= end; j++)
                temp.push_back(boundContour[j]);
        }
        edgeUV[i] = temp;
    }
    return edgeUV;
}

std::array<std::vector<Eigen::Vector3d>, 4> splitContourToEdge(
    const std::vector<Eigen::Vector3d>& boundContour, const std::array<Eigen::Vector2d, 4>& cornerPoints)
{
    std::array<int, 4> corner_index;
    for (int i = 0; i < 4; i++)
    {
        double distance = DBL_MAX;
        for (int j = 0; j < (int)boundContour.size(); j++)
        {
            double temp = (cornerPoints[i] - to_vec2(boundContour[j])).squaredNorm();
            if (temp < distance)
            {
                distance = temp;
                corner_index[i] = j;
            }
        }
    }
    std::array<std::vector<Eigen::Vector3d>, 4> edgeUV;
    for (int i = 0; i < 4; i++)
    {
        int end = corner_index[i];
        int start = corner_index[(i + 1) % 4];
        std::vector<Eigen::Vector3d> temp;
        if (start < end)
        {
            for (int j = start; j <= end; j++)
                temp.push_back(boundContour[j]);
        }
        else
        {
            for (int j = start; j < (int)boundContour.size(); j++)
                temp.push_back(boundContour[j]);
            for (int j = 0; j <= end; j++)
                temp.push_back(boundContour[j]);
        }
        edgeUV[i] = temp;
    }
    return edgeUV;
}

//Advancing Front Technique
void testAdvancingFront(const ModelMesh& mesh)
{
    pair<vector<Vector3d>, unordered_set<int>> boundContour = computeBoundaryEdges(mesh);
    std::array<Eigen::Vector2d, 4> cornerPoints = {
        Eigen::Vector2d(0,0),
        Eigen::Vector2d(mesh.bounding_.max()[0],0),
        Eigen::Vector2d(mesh.bounding_.max()[0],mesh.bounding_.max()[1]),
        Eigen::Vector2d(0,mesh.bounding_.max()[1]),
    };
    std::array<std::vector<Eigen::Vector3d>, 4> edgeUV = splitFirstContourToEdge(boundContour.first, cornerPoints);
    ModelMesh meshI = getInnerMeshByBoundary(mesh, boundContour.second);
    //for (const auto& iter : edgeUV)
    //    _drawPolygon(iter, colorRand(), true);

    //for (int t = 0; t < 50; t++)
    int count = 0;
    while (true)
    {
        count++;
        boundContour = computeBoundaryEdges(meshI);
        if (boundContour.first.empty())
            break;
        for (int i = 0; i < 4; i++)
            cornerPoints[i] = to_vec2(edgeUV[i].front());
        edgeUV = splitContourToEdge(boundContour.first, cornerPoints);
        meshI = getInnerMeshByBoundary(meshI, boundContour.second);
        //for (const auto& iter : edgeUV)
        //    _drawPolygon(iter, colorRand(), true);
        if (meshI.ibo_.empty())
            break;
    }
    //PolyfaceHandlePtr polyface = getPolyfaceHandleFromModelMesh(meshI);
    //_drawPolyface(polyface);
    return;
}

static void testTerrainMesh_UVline_0()
{
    string filename = R"(C:\Users\Aking\source\repos\learn_cpp\ProjectCGAL\OutputObj\modelmesh_terrain_0.obj)";
    std::vector<ModelMesh> meshVct = ModelMesh::readFromFile(filename);
    if (meshVct.empty())
        return;
    ModelMesh mesh = meshVct[0];
    //Matrix4d mat = mesh_relative_transform(mesh);
    //testAdvancingFront(mesh);
    return;
}

static int enrol = []()->int
    {
        testTerrainMesh_UVline_0();
        cout << "test_model_mesh finished.\n" << endl;
        return 0;
    }();


