#include "pch.h"
using namespace std;
using namespace clash;
using namespace land;


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
