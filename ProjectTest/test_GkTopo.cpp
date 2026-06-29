#include "pch.h"
#include "test_GkTopo.h"
using namespace std;
using namespace GeomKernel;

static void test0()
{
    GkMaVec vec2 = GkMaVec(1, exp(1), M_PI);
    GkMaVec vec3 = GkMaVec(1, 2, 3);

    GkMaPos pos3 = GkMaPos(2, 2, 3);
    string nump3 = pos3.to_str();
    //string nump2 =  GkMaPos(1, 2, 3).to_str(); //crush
    shared_ptr<GkMaPos> pvec3 = make_shared<GkMaPos>(GkMaPos(2, 3, 4));

    return;
}

static void test1()
{
    GkVertex vertex(GkMaPos(1, 2, 3));
    string name_v = vertex.debug_this();

    GkVertex v0(GkMaPos(0, 0, 0));
    GkVertex v1(GkMaPos(1, 0, 0));
    GkEdge edge(v0, v1);
    string name_e = edge.debug_this();
    int size = edge.size();
    //GkVertex* data = edge.data();
    std::vector<GkVertex> data = edge.debug_owning();

    std::vector<GkEdge> edges =
    {
        GkEdge(GkVertex(GkMaPos(0, 0, 0)), GkVertex(GkMaPos(1, 0, 0))),
        GkEdge(GkVertex(GkMaPos(1, 0, 0)), GkVertex(GkMaPos(1, 1, 0))),
        GkEdge(GkVertex(GkMaPos(1, 1, 0)), GkVertex(GkMaPos(0, 1, 0))),
        GkEdge(GkVertex(GkMaPos(0, 1, 0)), GkVertex(GkMaPos(0, 0, 0))),
    };
    GkLoop loop(edges);
    std::vector<GkEdge> getedges = loop.debug_owning();

    std::vector<GkMaPos> points;
    for (int i = 0; i < 100000; i++)
    {
        points.push_back(GkMaPos(i, i, i));
    }

    return;
}

static int enrol = []()->int
    {
        GkMaVec vec = GkMaVec(0, 0, 0);
        vec.debug();
        GkMaPos pos = GkMaPos(0, 0, 0);
        pos.debug();
        pos.debug_this();

        GkVertex vertex;
        vertex.debug_this();
        vertex.debug_owner();
        GkEdge edge;
        edge.debug_this();
        edge.debug_curve();
        //edge.child();
        //edge.size();
        edge.debug_owning();
        edge.debug_owner();
        GkLoop loop;
        loop.debug_this();
        loop.debug_owning();
        loop.debug_owner();

        //test0();
        test1();

        cout << clash::get_filepath_filename(__FILE__) << " finished.\n" << endl;
        return 0;
    }();