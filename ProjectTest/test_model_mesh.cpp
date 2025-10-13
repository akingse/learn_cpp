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


static int enrol = []()->int
	{
		//readTerrainDataToMesh_csv1(); //Îö¹¹¿¨ËÀ
		cout << "test_model_mesh finished.\n" << endl;
		return 0;
	}();

