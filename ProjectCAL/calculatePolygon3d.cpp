#include "pch.h"
using namespace std;
using namespace clash;
using namespace land;


//bool isBoundaryEdgeClose(const std::unordered_set<int>& index)
//{
//    return false;
//}

//Advancing Front Technique
void processAdvancingFrontTechnique(const ModelMesh& mesh)
{
    pair<vector<Vector3d>, unordered_set<int>> boundContour = computeBoundaryEdges(mesh.vbo_, mesh.ibo_);
    std::array<Eigen::Vector2d, 4> cornerPoints = {
        Eigen::Vector2d(0,0),
        Eigen::Vector2d(mesh.bounding_.max()[0],0),
        Eigen::Vector2d(mesh.bounding_.max()[0],mesh.bounding_.max()[1]),
        Eigen::Vector2d(0,mesh.bounding_.max()[1]),
    };
    std::array<std::vector<Eigen::Vector3d>, 4> edgeUV = splitContourToEdge(boundContour.first, cornerPoints, true);
    vector<Vector3i> meshIbo = getInnerMeshFacesByBoundary(mesh.ibo_, boundContour.second);
    //for (const auto& iter : edgeUV)
    //    _drawPolygon(iter, colorRand(), true);

    vector<vector<Vector3d>> allEdges;
    vector<vector<Eigen::Vector3d>> edgesLt;
    vector<vector<Eigen::Vector3d>> edgesRt;
    vector<vector<Eigen::Vector3d>> edgesDn;
    vector<vector<Eigen::Vector3d>> edgesUp;
    int count = 0;
    while (true)
    {
        count++;
        boundContour = computeBoundaryEdges(mesh.vbo_, meshIbo);
        if (boundContour.first.empty())
            break;
        for (int i = 0; i < 4; i++)
            cornerPoints[i] = to_vec2(edgeUV[i].front());
        edgeUV = splitContourToEdge(boundContour.first, cornerPoints);
        meshIbo = getInnerMeshFacesByBoundary(meshIbo, boundContour.second);
        //for (const auto& iter : edgeUV)
        //    _drawPolygon(iter, colorRand(), true);
        if (meshIbo.empty())
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

#define TEST_TERRAINMESH
#ifdef TEST_TERRAINMESH
static int enrol = []()->int
    {
        testTerrainMesh_UVline_0();
        cout << "test_model_mesh finished.\n" << endl;
        return 0;
    }();

int main()
{
    return 0;
}
#endif // TEST_TERRAINMESH
