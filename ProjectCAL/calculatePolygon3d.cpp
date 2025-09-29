#include "pch.h"
using namespace std;
using namespace clash;
using namespace land;


//bool isBoundaryEdgeClose(const std::unordered_set<int>& index)
//{
//    return false;
//}

//Advancing Front Technique
//void processAdvancingFrontTechnique(const ModelMesh& mesh)
//{
//
//}

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
