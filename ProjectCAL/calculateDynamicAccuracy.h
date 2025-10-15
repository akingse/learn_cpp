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

}

//for MeshGeometry precison
namespace accura
{
    //copy from Precision\Precision.hxx
    class DLLEXPORT_CAL Precision
    {
        static double sm_toleAngle;
        static double sm_toleConfusion;

    public:
        static constexpr void setAngular(const double angle) { sm_toleAngle = angle; }

        //to judge IsParallel, perpendicular
        static constexpr double Angular() { return sm_toleAngle; }

        static constexpr void setConfusion(const double confusion) { sm_toleConfusion = confusion; }

        //coincidence of two points, the tolerance of intersection
        static constexpr double Confusion() { return sm_toleConfusion; }

        static constexpr double ConfusionSquare() { return sm_toleConfusion * sm_toleConfusion; }

        static constexpr double Intersection() { return sm_toleConfusion * 0.01; }

        static constexpr double Approximation() { return sm_toleConfusion * 10.0; }


        static bool isParallel(const Eigen::Vector3d& vecA, const Eigen::Vector3d& vecB, double tolerance = Angular())
        {
            //double area = vecA.cross(vecB).norm(); // 0=|a|*|b|*sin(theta)
            Eigen::Vector3d vecAU = vecA.normalized();
            Eigen::Vector3d vecBU = vecB.normalized();
            Eigen::Vector3d croPro = vecAU.cross(vecBU);
            return croPro.norm() <= tolerance;//tobe optimize
        }

        static bool isPerpendi(const Eigen::Vector3d& vecA, const Eigen::Vector3d& vecB, double tolerance = Angular())//perpendicular
        {
            //double proj = vecA.dot(vecB); // 0=|a|*|b|*cos(theta)
            Eigen::Vector3d vecAU = vecA.normalized();
            Eigen::Vector3d vecBU = vecB.normalized();
            double dotPro = vecAU.dot(vecBU); //scalar products
            return dotPro <= tolerance;
        }

        //point relation
        static bool isPointsCoincident(const Eigen::Vector3d& pntA, const Eigen::Vector3d& pntB, double tolerance = 10 * Confusion())
        {
            double distance2 = (pntA - pntB).squaredNorm();
            return distance2 <= tolerance * tolerance;
        }

        static bool isPointOnLine(const Eigen::Vector3d& point, const clash::Segment3d& line, double tolerance = Confusion())
        {
            double distance = eigen::getDistanceOfPointAndLine(point, line);
            return distance <= tolerance;
        }

        static bool isPointOnPlane(const Eigen::Vector3d& point, const clash::Plane3d& plane, double tolerance = 0.1 * Confusion())
        {
            double distance = eigen::getDistanceOfPointAndPlane(point, plane.m_origin, plane.m_normal);
            return distance <= tolerance;
        }

        //line relation
        static bool isParallelTwoLines(const clash::Segment3d& lineA, const clash::Segment3d& lineB, double tolerance = Angular())
        {
            Eigen::Vector3d vecA = lineA[1] - lineA[0];
            Eigen::Vector3d vecB = lineB[1] - lineB[0];
            if (vecA.isZero() || vecB.isZero())
                return true; //error zero vector
            return isParallel(vecA, vecB, tolerance); //normalize inner
        }

        static bool isParallelLineAndPlane(const clash::Segment3d& line, const clash::Plane3d& plane, double tolerance = Angular())
        {
            Eigen::Vector3d vecA = line[1] - line[0];
            const Eigen::Vector3d& vecB = plane.m_normal;
            if (vecA.isZero() || vecB.isZero())
                return true; //error zero vector
            return isPerpendi(vecA, vecB, tolerance); //normalize inner
        }

        static bool isParallelTwoPlanes(const clash::Plane3d& planeA, const clash::Plane3d& planeB, double tolerance = Angular())
        {
            const Eigen::Vector3d& vecA = planeA.m_normal;
            const Eigen::Vector3d& vecB = planeB.m_normal;
            if (vecA.isZero() || vecB.isZero())
                return true; //error zero vector
            return isParallel(vecA, vecB, tolerance); //normalize inner
        }


    };
}
#endif// CALCULATE_DYNAMICACCURACY_H
