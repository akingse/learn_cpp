#include "pch.h"


double clash::calculatePolygonArea(const std::vector<Eigen::Vector2d>& polygon) //using Shoelace-Gauss method
{
	double area = 0.0;
	size_t n = polygon.size();
	for (int i = 0; i < n; ++i)
	{
		int j = (i + 1) % n; //avoid over bound
		area += (polygon[i][0] + polygon[j][0]) * (polygon[i][1] - polygon[j][1]); //wond
		//area += polygon[i][0] * polygon[j][1] - polygon[i][1] * polygon[j][0]; //cross2d
	}
	return 0.5 * std::fabs(area);
}