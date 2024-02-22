#pragma once

namespace accura
{
    static constexpr int MAX_DECIMAL_PRECISION = 8;
    static const int64_t MAX_COORD = INT64_MAX >> 2; //clipper parameter
    //using dynamic accuracy
    class GlobalAccuracy //singleton
    {
    public:
        static GlobalAccuracy& getInstance()
        {
            static GlobalAccuracy instance;
            return instance;
        }
        Eigen::AlignedBox3d m_modelBox3d; // in relative coordinate
        int m_precision = 0; //current precision
        Eigen::Vector3d m_relaOrigin = Eigen::Vector3d(0, 0, 0); //relative origin point
        inline int getDynamicPrecision()
        {
            if (m_modelBox3d.isEmpty())
                return MAX_DECIMAL_PRECISION; //the max precision
            if (m_precision != 0)
                return m_precision;
            double maxCoord = 0;
            for (int i = 0; i < 3; ++i)
            {
                maxCoord = std::max(std::max(fabs(m_modelBox3d.min()[i]), fabs(m_modelBox3d.max()[i])), maxCoord);
            }
            //double maxAccur = std::sqrt(MAX_COORD / std::pow(10, MAX_DECIMAL_PRECISION));
            int maxPrec = (int)std::floor(std::log10(MAX_COORD / (maxCoord * maxCoord)));
            m_precision = std::min(maxPrec, MAX_DECIMAL_PRECISION);
            return m_precision;
        }
        inline double eps() //const
        {
            int precision = getDynamicPrecision();
            return std::pow(10.0, -precision + 1);
        }
        inline double epsArea() //const //for area
        {
            int precision = getDynamicPrecision();
            return std::pow(10.0, -precision + 3);
        }

    private:
        GlobalAccuracy() = default;
        GlobalAccuracy(const GlobalAccuracy&) = delete;
        GlobalAccuracy operator=(const GlobalAccuracy&) = delete;
    };

    // DynamicAccuracy function, same name
    DLLEXPORT_CAL bool isTwoTrianglesPenetrationSAT(const std::array<Eigen::Vector2d, 3>& triA, const std::array<Eigen::Vector2d, 3>& triB, double tolerance);
    DLLEXPORT_CAL bool isTwoTrianglesPenetrationSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB, double tolerance);
    DLLEXPORT_CAL bool isPointInTriangle(const Eigen::Vector2d& point, const std::array<Eigen::Vector2d, 3>& trigon, double tolerance); // 2D

}
