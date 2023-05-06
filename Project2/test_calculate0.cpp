#include "pch.h"

Vec2 _get_circumcircle_center(const std::array<Vec2, 3>& trigon)
{
    {
        // _inverse_2x2
        double x1 = trigon[0].x;
        double y1 = trigon[0].y;
        double x2 = trigon[1].x;
        double y2 = trigon[1].y;
        double x3 = trigon[2].x;
        double y3 = trigon[2].y;

        double m1 = 2 * (x2 - x1);
        double n1 = 2 * (y2 - y1);
        double k1 = x2 * x2 - x1 * x1 + y2 * y2 - y1 * y1;
        double m2 = 2 * (x3 - x1);
        double n2 = 2 * (y3 - y1);
        double k2 = x3 * x3 - x1 * x1 + y3 * y3 - y1 * y1;
        double k = (m1 * n2 - n1 * m2);
        if (k == 0)  // is_float_zero(k)
            return {};
        return Vec2((n2 * k1 - n1 * k2) / k, (-m2 * k1 + m1 * k2) / k);
    }
}
