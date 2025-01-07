#pragma once
namespace para
{
	//----------------------------------------------------------------------------------------------
	// 	                                        Arc
	//----------------------------------------------------------------------------------------------
	PGArc getArcFromThreePoints(const std::vector<BPParaVec>& points);
	PGArc getArcFromThreePoints(const BPParaVec& point1, const BPParaVec& point2, const BPParaVec& point3);

	//----------------------------------------------------------------------------------------------
	// 	                                        Spline
	//----------------------------------------------------------------------------------------------
	std::vector<double> splineQuasiNodeVector(int  n, int  k); //node function
	double splineBaseFunction(int i, int k, double u, const std::vector<double>& nodeVector); //standard iterative function
	std::vector<BPParaVec> createSplineCurveQuasi(const std::vector<BPParaVec>& ctrlPoints, int disNum = 0, int k = 2);
	//bool isSplineOnSamePlane(const PGSplineCurve& spline); is control points on same plane

	//----------------------------------------------------------------------------------------------
	// 	                                        Discrete
	//----------------------------------------------------------------------------------------------
	size_t getAmountOfLineSegment(const PGLine& line, bool isClose = false);
	double getPerimeterOfSegments(const std::vector<PGSegment>& segments, bool isClose = false); 
	double getPerimeterOfArc(const PGArc& arc);
	double getPerimeterOfSpline(const PGSplineCurve& spline);
	double getPerimeterOfPolygon(const std::vector<BPParaVec>& points, bool isClose = false); 
	double getPerimeterOfLine(const PGLine& line, bool onlyCurve = false, bool isClose = false); //for curve discrete
	std::vector<double> linspace(double a, double b, int n); 
	std::vector<BPParaVec> getDiscretePointsFromSegment(const PGSegment& segm, int disNum, bool withEnd = true); 
	std::vector<BPParaVec> getDiscretePointsFromArc(const PGArc& arc, int disNum = 0, bool withEnd = true);
	std::vector<BPParaVec> getDiscretePointsFromSpline(const PGSplineCurve& spline, int disNum = 0, bool withEnd = true);
	std::vector<BPParaVec> getDiscretePointsFromLine(const PGLine& line, int disNum = 0, bool onlyCurve = true, bool isClose = false);

	//----------------------------------------------------------------------------------------------
	// 	                                        Polygon
	//----------------------------------------------------------------------------------------------
	//surface
	double getSurfaceOfPolygon(const std::vector<BPParaVec>& points); //2D, to judge polygon direction
	//bounding box
	std::vector<double> getRangeOfPolygon(const std::vector<BPParaVec>& points);

	//----------------------------------------------------------------------------------------------
	// 	                                        Math
	//----------------------------------------------------------------------------------------------
	double mathCublicRealRoot();
	int mathSign(double x);
	std::vector<double> getABCFromTwoPoints(const BPParaVec& p1, const BPParaVec& p2);
	std::vector<double> quadraticEquationOfOneVariable(double a, double b, double c, bool realRoot = true);
	std::vector<std::complex<double>> quadraticEquationOfOneVariableComplex(double a, double b, double c);

}

