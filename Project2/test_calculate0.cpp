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

bool _isDoubleZero(double num)
{
    return abs(num) < 1e-10;
}

double _test_custom_calculate(double nums[11])
{
    double c_add = nums[0] + nums[1] + nums[2] + nums[3] + nums[4] + nums[5] + nums[6] + nums[7] + nums[8] + nums[9] + nums[9];
    double c_sub = nums[0] - nums[1] - nums[2] - nums[3] - nums[4] - nums[5] - nums[6] - nums[7] - nums[8] - nums[9] - nums[9];
    double c_mul = c_add * c_sub;
    double c_dev = 1.0;
    if (!_isDoubleZero(c_sub))
		c_dev = c_add / c_sub;
    for (int i = 0; i < 4; i++)
    {
        c_mul = c_mul * c_dev;
        c_dev = c_mul / c_dev;
    }
    double c_sin = sin(c_add);
    double c_cos = cos(c_add);
    double c_tan = tan(c_add);
	double c_atan = atan2(c_add, c_sub);
    double c_log = log(c_add);
	return c_add + c_dev + c_mul + c_dev + c_sin + c_cos + c_tan + c_atan + c_log;
}
