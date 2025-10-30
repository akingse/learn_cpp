#include "pch.h"
using namespace std;
using namespace clash;
using namespace Eigen;
using namespace eigen;
using namespace accura;

static void readTerrainDataToMesh_csv1()
{
    string filename = R"(C:\Users\Aking\source\repos\bimbase\src\P3d2Stl\OutputObj\modelmesh_terrain_0.obj)";
    std::vector<ModelMesh> meshVct = ModelMesh::readFromFile(filename);
    if (meshVct.empty())
        return;
    const ModelMesh& mesh = meshVct[0];

    return;
}

static void test_mesh_0()
{
    ModelMesh mesh;
    mesh.vbo_ = {
        Vector3d(0,0,0),
        Vector3d(1,0,0),
        Vector3d(1,1,0),
        Vector3d(0,1,0),
    };

    mesh.ibo_ = {
        Vector3i(0,1,3),
        Vector3i(1,2,3),
    };
    mesh.fno_ = {
        Vector3d(0,0,1),
        Vector3d(0,0,1),
    };

    ModelMesh sim = games::meshMergeFacesBaseNormal(mesh);
    return;
}

static int enrol = []()->int
	{
        test_mesh_0();
		//readTerrainDataToMesh_csv1(); //Îö¹¹¿¨ËÀ
		cout << get_filepath_filename(__FILE__) << " finished.\n" << endl;
		return 0;
	}();

