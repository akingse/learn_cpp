#pragma once

//测试natvis调试监视
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
            std::string num = typeid(*this).name();
            num = num.substr(6);
            num += "(";
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
        inline double operator[](int i) const
        {
            return *((double*)m_impl + i);
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

        inline std::string to_str() const
        {
            std::string num = typeid(*this).name();
            num = num.substr(6);
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
            return num;
        }
        inline std::string debug() const
        {
            return to_str();
        }

        inline const std::string& debug_this()
        {
            return to_str();
        }
    };

    //Topo
    class GkVertex;
    class GkEdge;
    class GkLoop;
    class GkFace;
    class GkShell;

    class GkVertex
    {
        void* m_impl = 0;
        std::string m_infothis;
    public:
        GkVertex() = default;
        GkVertex(const GkMaPos& p)
        {
            GkMaPos* impl = new GkMaPos(
                *((double*)p.m_impl + 0), *((double*)p.m_impl + 1), *((double*)p.m_impl + 2));
            m_impl = impl;
        }
        //GkMaPos point() const
        //{
        //    return *(GkMaPos*)m_impl;
        //}
        //GkVertex(const GkVertex& p)
        //{
        //    m_impl = p.m_impl;
        //}
        //__declspec(noinline)
        const std::string& debug_this(); //const;

        //inline std::string debug_this() const
        //{
        //    if (!m_impl)
        //        return {};
        //    //fullname
        //    std::string info = __FUNCTION__;
        //    info += "; " + ((GkMaPos*)m_impl)->debug();
        //    return info;
        //}

        const GkEdge& debug_owner() const;

    };

    class GkEdge
    {
        void* m_impl = 0;
        std::string m_infothis;
        std::string m_infocurve;
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

        inline const std::vector<GkVertex>& debug_owning() const
        {
            std::vector<GkVertex>* imp = (std::vector<GkVertex>*)m_impl;
            if (!imp)
                return {};
            return *imp;
        }
        const GkLoop& debug_owner() const;

        //debug
        const std::string& debug_this();
        const std::string& debug_curve();

    };

    class GkLoop
    {
        void* m_impl = 0;
        std::string m_infothis;
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

        inline const std::vector<GkEdge>& debug_owning() const
        {
            std::vector<GkEdge>* imp = (std::vector<GkEdge>*)m_impl;
            if (!imp) //null not crush
                return {};
            return *imp;
        }
        const GkFace& debug_owner() const;
        const std::string& debug_this(); //const

    };

    class GkFace
    {
        void* m_impl = 0;
        std::string m_infothis;
        std::string m_infosurf;
    public:
        GkFace() = default;
        GkFace(const std::vector<GkLoop>& loops)
        {
            std::vector<GkLoop>* imp = new std::vector<GkLoop>;
            for (int i = 0; i < loops.size(); i++)
                imp->push_back(loops[i]);
            m_impl = imp;
        }

        const GkShell& debug_owner() const;
        inline const std::vector<GkLoop>& debug_owning() const
        {
            std::vector<GkLoop>* imp = (std::vector<GkLoop>*)m_impl;
            if (!imp)
                return {};
            return *imp;
        }
        const std::string& debug_this(); //const
        const std::string& debug_surface();
    };

    class GkShell
    {
        void* m_impl = 0;
        std::string info;
    public:
        const std::string& debug_this(); //const
    };
}

//局部变量版本
namespace Local
{
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

        inline const char* debug_this()
        {
            std::string num = typeid(*this).name();
            num = num.substr(6);
            num += "(\n";
            for (int i = 0; i < 3; i++)
            {
                std::ostringstream oss;
                oss << std::setprecision(std::numeric_limits<double>::max_digits10) << *((double*)m_impl + i);
                oss.clear();
                num += oss.str() + ",\n";
            }
            //num.pop_back();
            num += ")";
            //num += "\n(oss << std::setprecision(std::numeric_limits<double>::max_digits10) << *((double*)m_impl + i);)";
            std::string* info = new std::string(num);
            return info->c_str();
        }
    };

    class GkMaVec
    {
    public:
        std::shared_ptr<std::string> m_info;
        void* m_impl = 0;
        GkMaVec(double x, double y, double z)
        {
            double* imp = new double(3);
            imp[0] = x;
            imp[1] = y;
            imp[2] = z;
            m_impl = imp;
        }
        inline std::shared_ptr<std::string> debug_this()
        {
            std::string num = typeid(*this).name();
            num = num.substr(6);
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
            return std::make_shared<std::string>(num);
        }

    };

    //Topo
    class GkVertex;
    class GkEdge;
    class GkLoop;
    class GkFace;
    class GkShell;

    //class DebugInfo
    //{
    //public:
    //    std::string m_infothis;
    //    std::string m_infogeom;
    //    friend class GkVertex;
    //    friend class GkEdge;
    //};

    class GkVertex
    {
        void* m_impl = 0;
    public:
        GkVertex() = default;
        GkVertex(const Eigen::Vector3d& p)
        {
            double* impl = new double(3);
            impl[0] = p[0];
            impl[1] = p[1];
            impl[2] = p[2];
            m_impl = impl;
        }

        std::string debug_this() //const;
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
            return num;
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

        inline std::vector<GkVertex> debug_owning() const
        {
            std::vector<GkVertex>* imp = (std::vector<GkVertex>*)m_impl;
            if (!imp)
                return {};
            return *imp;
        }
        GkLoop debug_owner() const;

        //debug
        std::string debug_this()
        {
            std::string m_infothis = __FUNCTION__;
            return m_infothis;
        }
        std::string debug_geom()
        {
            std::string infoEdgeCur;
            infoEdgeCur.clear();
            infoEdgeCur = " getGeometry.curve";
            return infoEdgeCur;
        }

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

        inline std::vector<GkEdge> debug_owning() const
        {
            std::vector<GkEdge>* imp = (std::vector<GkEdge>*)m_impl;
            if (!imp)
                return {};
            return *imp;
        }
        GkFace debug_owner() const;
        std::string debug_this()//const
        {
            std::string m_infothis = __FUNCTION__;
            return m_infothis;
        }

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

        GkShell debug_owner() const;
        inline std::vector<GkLoop> debug_owning() const
        {
            std::vector<GkLoop>* imp = (std::vector<GkLoop>*)m_impl;
            if (!imp)
                return {};
            return *imp;
        }

        std::string debug_this() //const
        {
            std::string m_infothis = __FUNCTION__;
            return m_infothis;
        }
        std::string debug_geom()
        {
            std::string infoFaceSur;
            infoFaceSur.clear();
            infoFaceSur = " getGeometry.surface";
            return infoFaceSur;
        }

    };

    class GkShell
    {
        void* m_impl = 0;
    public:
        std::string debug_this() //const
        {
            std::string m_infothis = __FUNCTION__;
            return m_infothis;
        }
    };
}

#if 0
//静态变量版本
namespace Static
{
    //Topo
    class GkVertex;
    class GkEdge;
    class GkLoop;
    class GkFace;
    class GkShell;

    class GkVertex
    {
        void* m_impl = 0;
    public:
        GkVertex() = default;
        GkVertex(const Eigen::Vector3d& p)
        {
            double* impl = new double(3);
            impl[0] = p[0];
            impl[1] = p[1];
            impl[2] = p[2];
            m_impl = impl;
        }

        const std::string& debug_this(); //const;
        GkEdge debug_owner() const;

    };

    class GkEdge
    {
        void* m_impl = 0;
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

        inline std::vector<GkVertex> debug_owning() const
        {
            std::vector<GkVertex>* imp = (std::vector<GkVertex>*)m_impl;
            if (!imp)
                return {};
            return *imp;
        }
        GkLoop debug_owner() const;

        //debug
        const std::string& debug_this();
        const std::string& debug_geom();

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

        inline std::vector<GkEdge> debug_owning() const
        {
            std::vector<GkEdge>* imp = (std::vector<GkEdge>*)m_impl;
            if (!imp)
                return {};
            return *imp;
        }
        GkFace debug_owner() const;
        const std::string& debug_this();//const

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

        GkShell debug_owner() const;
        inline std::vector<GkLoop> debug_owning() const
        {
            std::vector<GkLoop>* imp = (std::vector<GkLoop>*)m_impl;
            if (!imp)
                return {};
            return *imp;
        }

        const std::string& debug_this(); //const
        const std::string& debug_geom();

    };

    class GkShell
    {
        void* m_impl = 0;
    public:
        const std::string& debug_this(); //const

    };
}

//const char* 
#define USING_CONST_REFER
namespace CharP
{
    //Topo
    class GkVertex;
    class GkEdge;
    class GkLoop;
    class GkFace;
    class GkShell;

    class GkVertex
    {
        void* m_impl = 0;
        char* m_infothis = 0;
    public:
        GkVertex() = default;
        GkVertex(const Eigen::Vector3d& p)
        {
            double* impl = new double(3);
            impl[0] = p[0];
            impl[1] = p[1];
            impl[2] = p[2];
            m_impl = impl;
        }
#ifdef USING_CONST_REFER
        const GkEdge& debug_owner() const;
#else
        GkEdge debug_owner() const;
#endif
        const char* debug_this();

    };

    class GkEdge
    {
        void* m_impl = 0;
        char* m_infothis = 0;
        char* m_infocurve = 0;
    public:
        GkEdge() = default;
        GkEdge(const GkVertex& v0, const GkVertex& v1)
        {
            std::vector<GkVertex>* imp = new std::vector<GkVertex>;// { v0, v1 };
            imp->push_back(v0);
            imp->push_back(v1);
            m_impl = imp;
        }
#ifdef USING_CONST_REFER
        const GkLoop& debug_owner() const;
        inline const std::vector<GkVertex>& debug_owning() const
#else
        GkLoop debug_owner() const;
        inline std::vector<GkVertex> debug_owning() const
#endif
        {
            std::vector<GkVertex>* imp = (std::vector<GkVertex>*)m_impl;
            if (!imp)
                return {};
            return *imp;
        }

        //debug
        const char* debug_this();
        const char* debug_geom();

    };

    class GkLoop
    {
        void* m_impl = 0;
        char* m_infothis = 0;
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
#ifdef USING_CONST_REFER
        const GkFace& debug_owner() const;
        inline const std::vector<GkEdge>& debug_owning() const
#else
        GkFace debug_owner() const;
        inline std::vector<GkEdge> debug_owning() const
#endif
        {
            std::vector<GkEdge>* imp = (std::vector<GkEdge>*)m_impl;
            if (!imp) //null not crush
                return {};
            return *imp;
        }
        const char* debug_this(); //const

    };

    class GkFace
    {
        void* m_impl = 0;
        char* m_infothis = 0;
        char* m_infosurf = 0;
    public:
        GkFace() = default;
        GkFace(const std::vector<GkLoop>& loops)
        {
            std::vector<GkLoop>* imp = new std::vector<GkLoop>;
            for (int i = 0; i < loops.size(); i++)
                imp->push_back(loops[i]);
            m_impl = imp;
        }
#ifdef USING_CONST_REFER
        const GkShell& debug_owner() const;
        inline const std::vector<GkLoop>& debug_owning() const
#else
        GkShell debug_owner() const;
        inline std::vector<GkLoop> debug_owning() const
#endif
        {
            std::vector<GkLoop>* imp = (std::vector<GkLoop>*)m_impl;
            if (!imp)
                return {};
            return *imp;
        }
        const char* debug_this(); //const
        const char* debug_geom();
    };

    class GkShell
    {
        void* m_impl = 0;
        char* info = 0;
    public:
        const char* debug_this(); //const
    };
}
#endif
