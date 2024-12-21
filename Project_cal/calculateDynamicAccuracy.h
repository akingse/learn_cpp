#pragma once
/*******************************************************************************
* Author    :  akingse		                                                   *
* Date      :  from June 2023												   *
* Website   :  https://github.com/akingse                                      *
* Copyright :  All rights reserved											   *
* Purpose   :  Some common calculation methods using dynamic accuracy 		   *
* License   :  MIT									                           *
*******************************************************************************/
#ifndef CALCULATE_DYNAMICACCURACY_H
#define CALCULATE_DYNAMICACCURACY_H

namespace accura
{
    static constexpr int MAX_DECIMAL_PRECISION = 8;
    static const int64_t MAX_COORD = INT64_MAX >> 2; //clipper parameter
    //using dynamic accuracy
    class GlobalAccuracy //singleton
    {
        enum class DataUnit
        {
            MILLIMETER, //mm
            DECIMETER, //dm
            METER, //m
            KILOMETRE, //km
        };
    public:
        static GlobalAccuracy& getInstance()
        {
            static GlobalAccuracy instance;
            return instance;
        }
        Eigen::AlignedBox3d m_modelBox3d; // in relative coordinate
        int m_precision = 0; //current precision
        int m_meshAngleTole = 8; // 36;
        Eigen::Vector3d m_relaOrigin = Eigen::Vector3d(0, 0, 0); //relative origin point
        double m_toleAngle = M_PI / 1080; //0.003
        double m_toleDist = 1e-5;
        double m_toleArea = 1e-4;
        const double m_toleFixed = 1e-8;
        double m_toleMerge2 = 0.0; //record square
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
        inline void reset()
        {
            m_modelBox3d = Eigen::AlignedBox3d();
            m_toleAngle = M_PI / 1080; //0.003
            m_toleDist = 1e-5;
            m_toleArea = 1e-4;
        }

        //inline double eps() //const
        //{
        //    int precision = getDynamicPrecision();
        //    return std::pow(10.0, -precision + 1);
        //}
        //inline double epsArea() //const //for area
        //{
        //    int precision = getDynamicPrecision();
        //    return std::pow(10.0, -precision + 3);
        //}
        inline void setMergeTolerance(const double toleMerge) //to merge duplicate points
        {
            m_toleMerge2 = toleMerge * toleMerge;
        }
    private:
        GlobalAccuracy() = default;
        GlobalAccuracy(const GlobalAccuracy&) = delete;
        GlobalAccuracy operator=(const GlobalAccuracy&) = delete;
    };

    // DynamicAccuracy function, same name
    DLLEXPORT_CAL bool isTwoTrianglesPenetrationSAT(const std::array<Eigen::Vector2d, 3>& triA, const std::array<Eigen::Vector2d, 3>& triB, double toleDist, double toleAngle);
    DLLEXPORT_CAL bool isTwoTrianglesPenetrationSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB, double toleDist, double toleAngle);
    DLLEXPORT_CAL bool isTwoTrianglesPenetrationSAT(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB,
        const Eigen::Vector3d& normalA, const Eigen::Vector3d& normalB, double toleDist, double toleAngle);
    DLLEXPORT_CAL bool isPointInTriangle(const Eigen::Vector2d& point, const std::array<Eigen::Vector2d, 3>& trigon, double toleDist); // 2D
    DLLEXPORT_CAL bool isTwoSegmentsIntersect(const std::array<Eigen::Vector2d, 2>& segmA, const std::array<Eigen::Vector2d, 2>& segmB, double toleDist);

}
#endif// CALCULATE_DYNAMICACCURACY_H
