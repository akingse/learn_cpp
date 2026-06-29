#pragma once

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
        inline std::string to_str() const
        {
            std::string num;
            for (int i = 0; i < 3; i++)
            {
                std::ostringstream oss;
                oss << std::setprecision(std::numeric_limits<double>::max_digits10) << *((double*)m_impl + i);
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
        inline std::string debug() const
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
        inline double operator[](int i) const
        {
            return *((double*)m_impl + i);
        }

        inline std::string to_str() const
        {
            std::string num = "(";
            for (int i = 0; i < 3; i++)
            {
                std::ostringstream oss;
                oss << std::setprecision(std::numeric_limits<double>::max_digits10) << *((double*)m_impl + i);
                oss.clear();
                num += oss.str() + ",";
            }
            num.pop_back();
            num += ")";
            return num;
        }
        inline std::string debug() const
        {
            return to_str();
        }
        inline std::string debug_this() const
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
        void* m_impl = 0;
        std::string info;
    public:
        GkVertex() = default;
        GkVertex(const GkMaPos& p)
        {
            GkMaPos* impl = new GkMaPos(p[0], p[1], p[2]);
            m_impl = impl;
        }
        //GkMaPos point() const
        //{
        //    return *(GkMaPos*)m_impl;
        //}
        GkVertex(const GkVertex& p)
        {
            m_impl = p.m_impl;
        }
        //__declspec(noinline)
        std::string& debug_this(); //const;

        //inline std::string debug_this() const
        //{
        //    if (!m_impl)
        //        return {};
        //    //fullname
        //    std::string info = __FUNCTION__;
        //    info += "; " + ((GkMaPos*)m_impl)->debug();
        //    return info;
        //}

        GkEdge debug_owner() const;

    };

    class GkEdge
    {
        void* m_impl = 0;
        std::string m_info0;
        std::string m_info1;
    public:
        GkEdge() = default;
        GkEdge(const GkEdge& edge)
        {
            std::vector<GkVertex>* imp = (std::vector<GkVertex>*)edge.m_impl;
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

        inline std::string child() const
        {
            std::vector<GkVertex>* imp = (std::vector<GkVertex>*)m_impl;
            if (!imp)
                return {};
            std::string info;
            for (int i = 0; i < imp->size(); i++)
                info += imp->at(i).debug_this() + "\n";
            return info;
        }
        inline int size() const
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


        inline std::string& debug_this() //const
        {
            //return __FUNCTION__;
            m_info0 = __FUNCTION__;
            return m_info0;
        }
        inline std::string& debug_curve() //const
        {
            //return " getGeometry.";
            m_info1 = " getGeometry.";
            return m_info1;
        }
        inline std::vector<GkVertex> debug_owning() const
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
        inline std::string debug_this() const
        {
            return __FUNCTION__ + std::string(" getGeometry.");
        }
        inline std::vector<GkEdge> debug_owning() const
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


}