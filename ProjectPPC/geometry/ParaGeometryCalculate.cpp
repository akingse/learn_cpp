#include "pch.h"
using namespace std;
namespace para
{
	//--------------------------------------------------------------------------------------------------------------
	//                                              arc
	//--------------------------------------------------------------------------------------------------------------
	PGArc getArcFromThreePoints(const std::vector<BPParaVec>& points)
	{
		if (points.size() != 3)
			return PGArc(g_MatrixO);
		return getArcFromThreePoints(points[0], points[1], points[2]);
	}
	PGArc getArcFromThreePoints(const BPParaVec& point1, const BPParaVec& point2, const BPParaVec& point3)
	{
		BPParaTransform forwM = getMatrixFromThreePoints({ point1 ,point2, point3 });
		BPParaTransform invM = inverse(forwM);
		double x1 = point1.x();
		double y1 = point1.x();
		double x2 = point2.x();
		double y2 = point2.y();
		double x3 = point3.x();
		double y3 = point3.y();
		double yc = ((x3 - x2) * (x3 * x3 + y3 * y3 - x1 * x1 - y1 * y1) - (x3 - x1) * (x3 * x3 + y3 * y3 - x2 * x2 - y2 * y2)) / (2 * (x3 - x2) * (y3 - y1) - 2 * (x3 - x1) * (y3 - y2));
		double xc = ((y3 - y2) * (x3 * x3 + y3 * y3 - x1 * x1 - y1 * y1) - (y3 - y1) * (x3 * x3 + y3 * y3 - x2 * x2 - y2 * y2)) / (2 * (y3 - y2) * (x3 - x1) - 2 * (y3 - y1) * (x3 - x2));
		double R = sqrt((xc - x1) * (xc - x1) + (yc - y1) * (yc - y1));
		BPParaTransform arcMat = forwM * translate(xc, yc) * scale(R) * rotz(atan2(y1 - yc, x1 - xc));
		double interAngle = (atan2(y3 - yc, x3 - xc) - atan2(y1 - yc, x1 - xc));
		return PGArc(arcMat, interAngle);
	}
	double getPerimeterOfArc(const PGArc& arc)
	{
		double a = getMatrixsScale(arc.m_mat).x();
		double b = getMatrixsScale(arc.m_mat).y();
		if (abs(a - b) < PL_Length) // circle
			return arc.m_scope * a;
		else
		{
			double lamda = (a - b) / (a + b);
			double perimeter = M_PI * (a + b) * (1 + (3 * lamda * lamda) / (10 + sqrt(4 - 3 * lamda * lamda)));//  Ramanujan formula
			return arc.m_scope / (2 * M_PI) * perimeter;
		}
	}

	//--------------------------------------------------------------------------------------------------------------
	//                                             spline
	//--------------------------------------------------------------------------------------------------------------
	std::vector<double> splineQuasiNodeVector(int  n, int  k)
	{
		vector<double> nodeVector(n + k + 2, 0); //(int nNum, value)
		int piecewise = n - k + 1;
		if (piecewise == 0)
			return nodeVector; //prevent division by zero
		if (piecewise == 1)
		{
			for (int i = n + 1; i < n + k + 2; i++)
				nodeVector[i] = 1;
		}
		else
		{
			int flag = 1;
			while (flag != piecewise)
			{
				nodeVector[k + flag] = nodeVector[k - 1 + flag] + 1 / double(piecewise);
				flag += 1;
			}
			for (int i = n + 1; i < n + k + 2; i++)
				nodeVector[i] = 1;
		}
		return nodeVector;
	}

	double splineBaseFunction(int i, int k, double u, const std::vector<double>& nodeVector)
	{
		double niku;
		double coef1, coef2;
		if (k == 0)
		{
			if (u >= nodeVector[i] && u < nodeVector[i + 1])
				niku = 1;
			else
				niku = 0;
		}
		else
		{
			if (abs(nodeVector[i + k] - nodeVector[i]) < PL_Length)
				coef1 = 0;
			else
				coef1 = (u - nodeVector[i]) / (nodeVector[i + k] - nodeVector[i]);
			if (abs(nodeVector[i + k + 1] - nodeVector[i + 1]) < PL_Length)
				coef2 = 0;
			else
				coef2 = (nodeVector[i + k + 1] - u) / (nodeVector[i + k + 1] - nodeVector[i + 1]);
			niku = coef1 * splineBaseFunction(i, k - 1, u, nodeVector) + coef2 * splineBaseFunction(i + 1, k - 1, u, nodeVector);
		}
		return niku;
	}

	vector<BPParaVec> createSplineCurveQuasi(const vector<BPParaVec>& ctrlPoints, int disNum /*=0*/, int k /*=2*/)
	{   //migration generation algorithm
		vector<BPParaVec> points;
		if (ctrlPoints.size() <= k)
			k = ctrlPoints.size() - 1;
		if (disNum == 0)
		{
			double length = getPerimeterOfPolygon(ctrlPoints);
			disNum = int(length / double(10)) >= 10 ? int(length / double(10)) : 10;
		}
		if (disNum <= ctrlPoints.size()) //
			return points;
		if (disNum - 1 < ctrlPoints.size())
			disNum = ctrlPoints.size() + 2;
		disNum = disNum - 1;
		int n = ctrlPoints.size() - 1;
		vector<vector<double>> P(3); //3D array
		vector<double> node;
		vector<double> Ni_k; // nik(n + 1, 0);
		double x, y, z;
		for (int i = 0; i < n + 1; i++)
		{
			P[0].push_back(ctrlPoints[i].x());
			P[1].push_back(ctrlPoints[i].y());
			P[2].push_back(ctrlPoints[i].z());
		}
		//准均匀B样条
		node = splineQuasiNodeVector(n, int(k));
		vector<double> lins = linspace(0.0, 1 - 1 / double(disNum), int(disNum));
		for (const auto& u : lins)
		{
			for (int i = 0; i < n + 1; i++)
				Ni_k.push_back(splineBaseFunction(i, k, u, node));
			x = 0;
			y = 0;
			z = 0;
			for (int j = 0; j < n + 1; j++)
			{
				x += P[0][j] * Ni_k[j];
				y += P[1][j] * Ni_k[j];
				z += P[2][j] * Ni_k[j];
			}
			points.push_back(BPParaVec(x, y, z));
			Ni_k.clear();
		}
		points.push_back(ctrlPoints.back());
		return points;
	}

	//bool isSplineOnSamePlane(const PGSplineCurve& spline)
	//{
	//	vector<BPParaVec> points = spline.m_points;
	//	return isPointsOnSamePlane(points);
	//}

	double getPerimeterOfSpline(const PGSplineCurve& spline) //长度(近似值)
	{
		vector<BPParaVec> points = getDiscretePointsFromSpline(spline); // spline.getDiscretePoints();
		return getPerimeterOfPolygon(points);
	}

	vector<BPParaVec> getDiscretePointsFromSpline(const PGSplineCurve& spline, int disNum/*=0*/, bool withEnd /*= true*/) //离散点
	{
		if (0 == disNum)
			disNum = (int)spline.m_num;
		if (!withEnd)
			disNum = disNum + 1;
		vector<BPParaVec> points = createSplineCurveQuasi(spline.m_points, disNum, spline.m_k);
		if (!withEnd)
			points.pop_back();
		return spline.m_transform * points;
	}
	 

	//--------------------------------------------------------------------------------------------------------------
	//                                             discrete
	//--------------------------------------------------------------------------------------------------------------
	vector<double> linspace(double a, double b, int n)
	{
		vector<double> plist;
		double d = (n == 1) ? (a) : ((b - a) / (n - 1));
		for (int i = 0; i < n; i++)
			plist.push_back(a + d * i);
		return plist;
	}
	double getPerimeterOfSegments(const std::vector<PGSegment>& segments, bool isClose /*= false*/) //total length
	{
		double length = 0;
		for (const auto& segm : segments)
			length += norm(segm.end() - segm.start());
		if (isClose)
			length += norm(segments.back().end() - segments.front().start());
		return length;
	}

	double getPerimeterOfPolygon(const std::vector<BPParaVec>& points, bool isClose /*= false*/) //
	{
		double length = 0;
		int lenP = points.size();
		for (int i = 0; i < lenP - 1; i++)
			length += norm(points[i + 1] - points[i]);
		if (isClose)
			length += norm(points.back() - points.front());
		return length;
	}

	vector<BPParaVec> getDiscretePointsFromSegment(const PGSegment& segm, int disNum, bool withEnd /*= true*/)
	{
		vector<BPParaVec> pointList;
		int disNumO = disNum;
		if (withEnd) // defualt
			disNumO = disNumO - 1;
		if (disNumO == 0)
			disNumO = 1;
		for (int i = 0; i < disNum; i++)
		{
			pointList.push_back(segm.start() + double(i) / disNumO * (segm.end() - segm.start()));
		}
		return pointList;
	}
	vector<BPParaVec> getDiscretePointsFromArc(const PGArc& arc, int disNum, bool withEnd/* = true*/)
	{
		if (disNum == 0)
		{
			double length = getPerimeterOfArc(arc);
			(int(length / 20) >= 10) ? disNum = int(length / 20) : disNum = 10;
		}
		vector<BPParaVec> pointList;
		int disNumO = disNum; //record origin disNum
		if (withEnd) // 
			disNumO = disNumO - 1;
		if (disNumO == 0)
			disNumO = 1;
		for (int i = 0; i < disNum; i++)// i / disNum -> 1
		{
			BPParaVec pointI = BPParaVec(cos(double(i) / disNumO * arc.m_scope), sin(double(i) / disNumO * arc.m_scope), 0);
			pointList.push_back(arc.m_mat * pointI);
		}
		return pointList;
	}
	//discrete line
	vector<BPParaVec> getDiscretePointsFromLine(const PGLine& line, int disNumO /*= 0*/, bool onlyCurve /*= true*/, bool isClose /*= false*/) // 离散化line为BPParaVec列表
	{
		return getDiscretePointsFromLine(line.get(), disNumO, onlyCurve, isClose);
	}

	//--------------------------------------------------------------------------------------------------------------
	//                                             polygon
	//--------------------------------------------------------------------------------------------------------------

	double getSurfaceOfPolygon(std::vector<BPParaVec> points) 
	{
		int lenP = points.size();
		double Surfacex2 = 0.0;
		for (int i = 0; i < lenP - 1; i++)
		{
			Surfacex2 += points[i].x() * points[i + 1].y() - points[i + 1].x() * points[i].y();
		}
		Surfacex2 += points[lenP - 1].x() * points[0].y() - points[0].x() * points[lenP - 1].y();
		return Surfacex2 / 2;
	}

	vector<double> getRangeOfPolygon(std::vector<BPParaVec> points)
	{
		if (points.size() == 0)
			return { 0,0,0,0,0,0 };
		double xMin = points[0].x(), xMax = points[0].x();
		double yMin = points[0].y(), yMax = points[0].y();
		double zMin = points[0].z(), zMax = points[0].z();
		for (const auto& i : points)
		{
			if (i.x() > xMax)
				xMax = i.x();
			if (i.x() < xMin)
				xMin = i.x();
			if (i.y() > yMax)
				yMax = i.y();
			if (i.y() < yMin)
				yMin = i.y();
			if (i.z() > zMax)
				zMax = i.z();
			if (i.z() < zMin)
				zMin = i.z();
		}
		return { xMin, xMax, yMin, yMax, zMin, zMax };
	}

	//--------------------------------------------------------------------------------------------------------------
	//                                             math
	//--------------------------------------------------------------------------------------------------------------


	// quation of line
	std::vector<double> getABCFromTwoPoints(BPParaVec p1, BPParaVec p2)
	{
		//(y2 - y1)* x - (x2 - x1) * y + (x2 * y1 - x1 * y2) = 0
		double x1 = p1.x();
		double y1 = p1.y();
		double x2 = p2.x();
		double y2 = p2.y();
		double A = y2 - y1;
		double B = -(x2 - x1);
		double C = x2 * y1 - x1 * y2;
		return vector<double>({ A, B, C });
	}
	vector<double> quadraticEquationOfOneVariable(double a, double b, double c, bool realRoot /*= true*/)
	{
		vector<double> roots;
		if (abs(a) < PL_Length)// isnot quadraticEquation
		{
			return (abs(b) < PL_Length) ? roots : vector<double>{ -c / b };
		}
		double delta = b * b - 4 * a * c;
		if (abs(delta) < PL_Length)
			return vector<double>{-b / (2 * a)};
		else if (delta < 0)
			return roots;
		else
		{
			double d = sqrt(delta);
			roots.push_back((-b + d) / (2 * a));
			roots.push_back((-b - d) / (2 * a));
			return roots;
		}
	}
	std::vector<complex<double>> quadraticEquationOfOneVariableComplex(double a, double b, double c)
	{
		complex<double> a1(1, 2);
		complex<double> b1 = 2.0 + 3i;
		return vector<complex<double>>({ a1, b1 });
	}


}