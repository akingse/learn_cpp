#include "pch.h"
#include "test_GkTopo.h"
using namespace std;
using namespace Eigen;
//using namespace GeomKernel;
static string s_empty;
//__declspec(noinline)

namespace Local
{
    GkEdge GkVertex::debug_owner() const
    {
        GkVertex v0(Vector3d(10, 0, 0));
        GkVertex v1(Vector3d(20, 0, 0));
        GkEdge edge(v0, v1);
        return edge;
    }
    GkLoop GkEdge::debug_owner() const
    {
        return GkLoop();
    }
    GkFace GkLoop::debug_owner() const
    {
        return GkFace();
    }
    GkShell GkFace::debug_owner() const
    {
        return GkShell();
    }

}

#if 0
namespace Static
{
    GkEdge GkVertex::debug_owner() const
    {
        GkVertex v0(Vector3d(10, 0, 0));
        GkVertex v1(Vector3d(20, 0, 0));
        GkEdge edge(v0, v1);
        return edge;
    }
    GkLoop GkEdge::debug_owner() const
    {
        return GkLoop();
    }
    GkFace GkLoop::debug_owner() const
    {
        return GkFace();
    }
    GkShell GkFace::debug_owner() const
    {
        return GkShell();
    }

    const std::string& GkVertex::debug_this() //const;
    {
        if (!m_impl)
            return {};
        std::string num = "coord";
        num += "(";
        for (int i = 0; i < 3; i++)
        {
            std::ostringstream oss;
            oss << std::setprecision(std::numeric_limits<double>::max_digits10) << *((double*)m_impl + i);
            oss.clear();
            num += oss.str() + ",";
        }
        num.pop_back();
        num += ")";
        //return num;
        static std::string infoGkVertex;
        infoGkVertex.clear();
        infoGkVertex = num;
        return infoGkVertex;
    }

    const std::string& GkEdge::debug_this()
    {
        std::string m_infothis = __FUNCTION__;
        static std::string infoGkEdge;
        infoGkEdge.clear();
        infoGkEdge = m_infothis;
        return infoGkEdge;
    }
    const std::string& GkEdge::debug_geom()
    {
        static std::string infoEdgeCur;
        infoEdgeCur.clear();
        infoEdgeCur = " getGeometry.curve";
        return infoEdgeCur;
    }

    const std::string& GkLoop::debug_this()//const
    {
        std::string m_infothis = __FUNCTION__;
        static std::string infoGkLoop;
        infoGkLoop.clear();
        infoGkLoop = m_infothis;
        return infoGkLoop;
    }

    const std::string& GkFace::debug_this() //const
    {
        std::string m_infothis = __FUNCTION__;
        static std::string infoGkFace;
        infoGkFace.clear();
        infoGkFace = m_infothis;
        return infoGkFace;
    }
    const std::string& GkFace::debug_geom()
    {
        static std::string infoFaceSur;
        infoFaceSur.clear();
        infoFaceSur = " getGeometry.surface";
        return infoFaceSur;
    }

    const std::string& GkShell::debug_this() //const
    {
        std::string m_infothis = __FUNCTION__;
        static std::string infoGkShell;
        infoGkShell.clear();
        infoGkShell = m_infothis;
        return infoGkShell;
    }
}

namespace CharP
{
#ifdef USING_CONST_REFER
    const GkEdge& GkVertex::debug_owner() const
    {
        GkVertex v0(Vector3d(10, 0, 0));
        GkVertex v1(Vector3d(20, 0, 0));
        GkEdge edge(v0, v1);
        return edge;
    }
    const GkLoop& GkEdge::debug_owner() const
    {
        return GkLoop();
    }
    const GkFace& GkLoop::debug_owner() const
    {
        return GkFace();
    }
    const GkShell& GkFace::debug_owner() const
    {
        return GkShell();
    }
#else
    GkEdge GkVertex::debug_owner() const
    {
        GkVertex v0(Vector3d(10, 0, 0));
        GkVertex v1(Vector3d(20, 0, 0));
        GkEdge edge(v0, v1);
        return edge;
    }
    GkLoop GkEdge::debug_owner() const
    {
        return GkLoop();
    }
    GkFace GkLoop::debug_owner() const
    {
        return GkFace();
    }
    GkShell GkFace::debug_owner() const
    {
        return GkShell();
    }
#endif

    const char* GkVertex::debug_this() //const;
    {
        if (!m_impl)
            return {};
        std::string num = "coord";
        num += "(";
        for (int i = 0; i < 3; i++)
        {
            std::ostringstream oss;
            oss << std::setprecision(std::numeric_limits<double>::max_digits10) << *((double*)m_impl + i);
            oss.clear();
            num += oss.str() + ",";
        }
        num.pop_back();
        num += ")";
        //return num;
        std::string* info = new std::string(num);
        return info->c_str();
    }

    const char* GkEdge::debug_this()
    {
        std::string m_infothis = __FUNCTION__;
        std::string* info = new std::string(m_infothis);
        return info->c_str();
    }
    const char* GkEdge::debug_geom()
    {
        std::string infoEdgeCur = " getGeometry.curve";
        std::string* info = new std::string(infoEdgeCur);
        return info->c_str();
    }

    const char* GkLoop::debug_this()//const
    {
        std::string m_infothis = __FUNCTION__;
        std::string* info = new std::string(m_infothis);
        return info->c_str();
    }

    const char* GkFace::debug_this() //const
    {
        std::string m_infothis = __FUNCTION__;
        std::string* info = new std::string(m_infothis);
        return info->c_str();
    }
    const char* GkFace::debug_geom()
    {
        std::string infoFaceSur = " getGeometry.surface";
        std::string* info = new std::string(infoFaceSur);
        return info->c_str();
    }

    const char* GkShell::debug_this() //const
    {
        std::string m_infothis = __FUNCTION__;
        std::string* info = new std::string(m_infothis);
        return info->c_str();
    }

}
#endif

namespace GeomKernel
{
    const std::string& GkVertex::debug_this() //const
    {
        if (!m_impl)
            return s_empty;
        //static std::string infoVertex;
        //m_infothis.clear();
        //m_infothis = "GeomKernel::GkVertex::debug_this; ";
        m_infothis = ((GkMaPos*)m_impl)->debug();
        return m_infothis;
    }

    //#ifdef _DEBUG
    //void force_link_GkVertex_debug_this(const GkVertex& v)
    //{
    //    volatile auto s = v.debug_this();
    //}
    //#endif

    //implement
    const GkEdge& GkVertex::debug_owner() const
    {
        GkVertex v0(GkMaPos(10, 0, 0));
        GkVertex v1(GkMaPos(20, 0, 0));
        GkEdge edge(v0, v1);
        return edge;
    }

    const GkLoop& GkEdge::debug_owner() const
    {
        return GkLoop();
    }
    const std::string& GkEdge::debug_this() //const
    {
        //static std::string infoEdge;
        //infoEdge.clear();
        m_infothis = __FUNCTION__;
        return m_infothis;
    }
    const std::string& GkEdge::debug_curve() //const
    {
        //static std::string infoEdgeCur;
        //infoEdgeCur.clear();
        m_infocurve = " getGeometry.curve";
        return m_infocurve;
    }

    const GkFace& GkLoop::debug_owner() const
    {
        return GkFace();
    }
    const std::string& GkLoop::debug_this() //const
    {
        static std::string infoLoop;
        infoLoop.clear();
        infoLoop = __FUNCTION__;
        return infoLoop;
    }

    const std::string& GkFace::debug_this() //const
    {
        static std::string infoFace;
        infoFace.clear();
        infoFace = __FUNCTION__;
        return infoFace;
    }
    const std::string& GkFace::debug_surface() //const
    {
        //static std::string infoFaceSur;
        //infoFaceSur.clear();
        m_infosurf = " getGeometry.surface";
        return m_infosurf;
    }
    const GkShell& GkFace::debug_owner() const
    {
        return GkShell();
    }

    const std::string& GkShell::debug_this() //const
    {
        static std::string infoShell;
        infoShell.clear();
        infoShell = __FUNCTION__;
        return infoShell;
    }
}

//enrol
namespace GeomKernel
{
    static void enrol_0()
    {
        GeomKernel::GkMaVec vec = GeomKernel::GkMaVec(0, 0, 0);
        vec.debug();
        GeomKernel::GkMaPos pos = GeomKernel::GkMaPos(0, 0, 0);
        pos.debug();
        pos.debug_this();

        GeomKernel::GkVertex vertex;
        //vertex.debug_this();
        //vertex.debug_owner();

        GeomKernel::GkEdge edge;
        edge.debug_this();
        edge.debug_curve();
        //edge.child();
        //edge.size();
        edge.debug_owning();
        edge.debug_owner();

        GeomKernel::GkLoop loop;
        loop.debug_this();
        loop.debug_owning();
        loop.debug_owner();

        GeomKernel::GkFace face;
        face.debug_this();
        face.debug_owning();
        face.debug_owner();
    }
}

namespace Local
{
    static void enrol_1()
    {
        GkVertex vertex;
        vertex.debug_this();
        vertex.debug_owner();

        GkEdge edge;
        edge.debug_this();
        edge.debug_geom();
        edge.debug_owning();
        edge.debug_owner();

        GkLoop loop;
        loop.debug_this();
        loop.debug_owning();
        loop.debug_owner();

        GkFace face;
        face.debug_this();
        face.debug_geom();
        face.debug_owning();
        face.debug_owner();
    }
}

#if 0
namespace Static
{
    static void enrol_2()
    {
        GkVertex vertex;
        vertex.debug_this();
        vertex.debug_owner();

        GkEdge edge;
        edge.debug_this();
        edge.debug_geom();
        edge.debug_owning();
        edge.debug_owner();

        GkLoop loop;
        loop.debug_this();
        loop.debug_owning();
        loop.debug_owner();

        GkFace face;
        face.debug_this();
        face.debug_geom();
        face.debug_owning();
        face.debug_owner();
    }
}

namespace CharP
{
    static void enrol_3()
    {
        GkVertex vertex;
        vertex.debug_this();
        vertex.debug_owner();

        GkEdge edge;
        edge.debug_this();
        edge.debug_geom();
        edge.debug_owning();
        edge.debug_owner();

        GkLoop loop;
        loop.debug_this();
        loop.debug_owning();
        loop.debug_owner();

        GkFace face;
        face.debug_this();
        face.debug_geom();
        face.debug_owning();
        face.debug_owner();
    }
}
#endif

namespace GeomKernel
{
    //向量
    static void test0()
    {
        GkMaVec vec2 = GkMaVec(1, exp(1), M_PI);
        GkMaVec vec3 = GkMaVec(1, 2, 3);

        GkMaPos pos3 = GkMaPos(2, 2, 3);
        string nump3 = pos3.to_str();
        //string nump2 =  GkMaPos(1, 2, 3).to_str(); //crush
        shared_ptr<GkMaPos> pvec3 = make_shared<GkMaPos>(GkMaPos(2, 3, 4));

        Eigen::Matrix4d mat = eigen::rotz(1)* eigen::rotx(1);
        string num = "GkMaTrans";
        num += "(\n";
        for (int i = 0; i < 4; i++)
        {
            num += "[";
            for (int j = 0; j < 4; j++)
            {
                std::ostringstream oss;
                oss << std::setprecision(std::numeric_limits<double>::max_digits10) << mat(i,j);
                oss.clear();
                num += oss.str() + ",";
            }
            num.pop_back();
            num += "],\n";
        }
        num += ")";
        return;
    }

    //效率测试
    static void test1()
    {
        string info;
        std::ostringstream oss;
        oss << std::setprecision(std::numeric_limits<double>::max_digits10) << 1;
        info = oss.str();
        oss.str("");  // 清空内部字符串缓冲区
        oss.clear();  // 重置错误状态标志（防止之前的错误影响后续操作）
        oss << std::setprecision(std::numeric_limits<double>::max_digits10) << 2;
        info = oss.str();
        oss.str("");
        oss.clear();

        GkVertex vertex(GkMaPos(1, 2, 3));
        //string name_v = vertex.debug_this();

        GkVertex v0(GkMaPos(0, 0, 0));
        GkVertex v1(GkMaPos(1, 0, 0));
        GkEdge edge(v0, v1);
        string name_e = edge.debug_this();


        int size = edge.size();
        std::vector<GkVertex> data = edge.debug_owning();
        //std::vector<GkEdge> getedges = loop.debug_owning();

        //效率测试
        std::vector<GkMaPos> points;
        for (int i = 0; i < 100000; i++)
        {
            points.push_back(GkMaPos(i, i, i));
        }

        return;
    }
}

namespace GeomKernel
{
    static void test2()
    {
        GkVertex v0(GkMaPos(0, 0, 0));
        GkVertex v1(GkMaPos(1, 0, 0));
        GkEdge edge(v0, v1);

        std::vector<GkEdge> edges =
        {
            GkEdge(GkVertex(GkMaPos(0, 0, 0)), GkVertex(GkMaPos(1, 0, 0))),
            GkEdge(GkVertex(GkMaPos(1, 0, 0)), GkVertex(GkMaPos(1, 1, 0))),
            GkEdge(GkVertex(GkMaPos(1, 1, 0)), GkVertex(GkMaPos(0, 1, 0))),
            GkEdge(GkVertex(GkMaPos(0, 1, 0)), GkVertex(GkMaPos(0, 0, 0))),
        };
        GkLoop loop(edges);

        GkFace face({ loop });

        return;
    }
}

namespace Local
{
    //局部变量版本
    static void test3()
    {
        GkMaPos pos = GkMaPos(2, 2, 3);
        string info = pos.debug_this();

        GkMaVec vec = GkMaVec(2, 2, 3);
        std::shared_ptr<std::string> info2 = vec.debug_this();

        GkVertex v0(Vector3d(0, 0, 0));
        GkVertex v1(Vector3d(1, 0, 0));
        std::string info_v0 = v0.debug_this();

        GkEdge edge(v0, v1);

        std::vector<GkEdge> edges =
        {
            GkEdge(GkVertex(Vector3d(0, 0, 0)), GkVertex(Vector3d(1, 0, 0))),
            GkEdge(GkVertex(Vector3d(1, 0, 0)), GkVertex(Vector3d(1, 1, 0))),
            GkEdge(GkVertex(Vector3d(1, 1, 0)), GkVertex(Vector3d(0, 1, 0))),
            GkEdge(GkVertex(Vector3d(0, 1, 0)), GkVertex(Vector3d(0, 0, 0))),
        };
        GkLoop loop(edges);
        GkEdge edge0 = loop.debug_owning()[0];
        GkLoop lp_ow = edge0.debug_owner();

        GkFace face({ loop });

        return;
    }
}

#if 0
namespace Static
{
    //静态变量版本
    static void test4()
    {
        GkVertex v0(Vector3d(0, 0, 0));
        GkVertex v1(Vector3d(1, 0, 0));
        std::string info_v0 = v0.debug_this();

        GkEdge edge(v0, v1);

        std::vector<GkEdge> edges =
        {
            GkEdge(GkVertex(Vector3d(0, 0, 0)), GkVertex(Vector3d(1, 0, 0))),
            GkEdge(GkVertex(Vector3d(1, 0, 0)), GkVertex(Vector3d(1, 1, 0))),
            GkEdge(GkVertex(Vector3d(1, 1, 0)), GkVertex(Vector3d(0, 1, 0))),
            GkEdge(GkVertex(Vector3d(0, 1, 0)), GkVertex(Vector3d(0, 0, 0))),
        };
        GkLoop loop(edges);
        GkEdge edge0 = loop.debug_owning()[0];
        GkLoop lp_ow = edge0.debug_owner();

        GkFace face({ loop });

        return;
    }
}

namespace CharP
{
    //CharP成员版本
    static void test5()
    {
        GkVertex v0(Vector3d(0, 0, 0));
        GkVertex v1(Vector3d(1, 0, 0));
        std::string info_v0 = v0.debug_this();

        GkEdge edge(v0, v1);

        std::vector<GkEdge> edges =
        {
            GkEdge(GkVertex(Vector3d(0, 0, 0)), GkVertex(Vector3d(1, 0, 0))),
            GkEdge(GkVertex(Vector3d(1, 0, 0)), GkVertex(Vector3d(1, 1, 0))),
            GkEdge(GkVertex(Vector3d(1, 1, 0)), GkVertex(Vector3d(0, 1, 0))),
            GkEdge(GkVertex(Vector3d(0, 1, 0)), GkVertex(Vector3d(0, 0, 0))),
        };
        GkLoop loop(edges);
        GkEdge edge0 = loop.debug_owning()[0];
        GkLoop lp_ow = edge0.debug_owner();

        GkFace face({ loop });

        return;
        //嵌套和返回值拷贝和引用有关，引用可以，拷贝不行；
    }
}
#endif

static int enrol = []()->int
    {
        GeomKernel::enrol_0();
        Local::enrol_1();
        //Static::enrol_2();
        //CharP::enrol_3();

        GeomKernel::test0();
        GeomKernel::test1();
        GeomKernel::test2();

        //Local
        Local::test3();
        //Static::test4();
        //CharP::test5();

        cout << clash::get_filepath_filename(__FILE__) << " finished.\n" << endl;
        return 0;
    }();