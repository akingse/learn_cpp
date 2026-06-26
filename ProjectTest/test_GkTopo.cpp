#include "pch.h"
using namespace std;

//≤‚ ‘natvisµ˜ ‘º‡ ”

namespace GeomKernel
{
    class GkMaVec
    {
    public:
        void* m_impl = 0;
        GkMaVec(double x, double y, double z)
        {
            double* imp = new double(3);
            imp[0] = x;
            imp[1] = y;
            imp[2] = z;
            m_impl = imp;
        }
        std::string to_str() const
        {
            string num;
            for (int i = 0; i < 3; i++)
            {
                ostringstream oss;
                oss << std::setprecision(numeric_limits<double>::max_digits10) << *((double*)m_impl + i);
                oss.clear();
                num += oss.str() + ",";
            }
            //double a = 0;
            //num += to_string(1.0 / a);
            //if (*(double*)m_impl == 1)
            //{
            //    double* p = 0;
            //    num += to_string(*p);
            //}
            num.pop_back();
            return num;
        }
        std::string debug() const
        {
            return to_str();
        }
    };
    class GkMaPos
    {
    public:
        void* m_impl = 0;
        GkMaPos(double x, double y, double z)
        {
            double* imp = new double(3);
            imp[0] = x;
            imp[1] = y;
            imp[2] = z;
            m_impl = imp;
        }
        double operator[](int i) const
        {
            return *((double*)m_impl + i);
        }

        std::string to_str() const
        {
            string num = "(";
            for (int i = 0; i < 3; i++)
            {
                ostringstream oss;
                oss << std::setprecision(numeric_limits<double>::max_digits10) << *((double*)m_impl + i);
                oss.clear();
                num += oss.str() + ",";
            }
            num.pop_back();
            num += ")";
            return num;
        }
        std::string debug() const
        {
            return to_str();
        }
        std::string debug_this() const
        {
            return to_str();
        }
    };
    //Topo
    class GkVertex;
    class GkEdge;
    class GkLoop;
    class GkFace;

    class GkVertex
    {
    public:
        void* m_impl = 0;
        GkVertex() = default;
        GkVertex(const GkMaPos& p)
        {
            GkMaPos* impl = new GkMaPos(p[0], p[1], p[2]);
            m_impl = impl;
        }
        GkMaPos point() const
        {
            return *(GkMaPos*)m_impl;
        }
        GkVertex(const GkVertex& p)
        {
            m_impl = p.m_impl;
        }
        inline std::string debug_this() const
        {
            if (!m_impl)
                return {};
            //fullname
            string info = __FUNCTION__;
            info += "; " + ((GkMaPos*)m_impl)->debug();
            return info;
        }

        GkEdge debug_owner() const;

    };

    class GkEdge
    {
        void* m_impl = 0;
    public:
        GkEdge() = default;
        GkEdge(const GkEdge& edge)
        {
            std::vector<GkVertex>* imp = (std::vector<GkVertex>*)m_impl;
            if (!imp)
                return;
            std::vector<GkVertex>* new_imp = new std::vector<GkVertex>;
            for (int i = 0; i < imp->size(); i++)
            {
                new_imp->push_back((*imp)[i]);
            }
            m_impl = new_imp;
        }
        GkEdge(const GkVertex& v0, const GkVertex& v1)
        {
            std::vector<GkVertex>* imp = new std::vector<GkVertex>;// { v0, v1 };
            imp->push_back(v0);
            imp->push_back(v1);
            m_impl = imp;
        }

        std::string child() const
        {
            std::vector<GkVertex>* imp = (std::vector<GkVertex>*)m_impl;
            if (!imp)
                return {};
            string info;
            for (int i = 0; i < imp->size(); i++)
                info += imp->at(i).debug_this() + "\n";
            return info;
        }
        int size() const
        {
            std::vector<GkVertex>* imp = (std::vector<GkVertex>*)m_impl;
            if (!imp)
                return 0;
            return imp->size();
        }
        //GkVertex* data() const
        //{
        //    std::vector<GkVertex>* imp = (std::vector<GkVertex>*)m_impl;
        //    if (!imp)
        //        return nullptr;
        //    return imp->data();
        //}


        std::string debug_this() const
        {
            return __FUNCTION__ + string(" getGeometry.");
        }
        std::vector<GkVertex> debug_owning() const
        {
            std::vector<GkVertex>* imp = (std::vector<GkVertex>*)m_impl;
            if (!imp)
                return {};
            return *imp;
        }
        GkLoop debug_owner() const;
    };
    
    class GkLoop
    {
        void* m_impl = 0;
    public:
        GkLoop() = default;
        GkLoop(const std::vector<GkEdge>& edges)
        {
            std::vector<GkEdge>* imp = new std::vector<GkEdge>;
            for (int i = 0; i < edges.size(); i++)
            {
                imp->push_back(edges[i]);
            }
            m_impl = imp;
        }
        std::string debug_this() const
        {
            return __FUNCTION__ + string(" getGeometry.");
        }
        std::vector<GkEdge> debug_owning() const
        {
            std::vector<GkEdge>* imp = (std::vector<GkEdge>*)m_impl;
            if (!imp)
                return {};
            return *imp;
        }
        GkFace debug_owner() const;
    };
    
    class GkFace
    {
        void* m_impl = 0;
    public:
        GkFace() = default;
        GkFace(const std::vector<GkLoop>& loops)
        {
            std::vector<GkLoop>* imp = new std::vector<GkLoop>;
            for (int i = 0; i < loops.size(); i++)
                imp->push_back(loops[i]);
            m_impl = imp;
        }
    };

    //implement
    GkEdge GkVertex::debug_owner() const
    {
        return GkEdge();
    }
    GkLoop GkEdge::debug_owner() const
    {
        return GkLoop();
    }
    GkFace GkLoop::debug_owner() const
    {
        return GkFace();
    }
}
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
        edge.child();
        edge.size();
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