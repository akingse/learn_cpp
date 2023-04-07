#pragma once
#pragma once
namespace para
{
	enum class GRTwoVectorsDirection // geometry relation
	{
		Direction_None, // atypia
		Direction_Any, //due to zero
		Direction_Same,
		Direction_Opposite,
		Direction_Vertical,
	};
	class PGSegment;

	//point-line-plane
	BPParaVec getNearestPointOfPointLine(const BPParaVec& point, const PGSegment& line);
	double getDistanceOfPointLine(const BPParaVec& point, const PGSegment& line);

	//relation-on
	bool isPointOnArc(const BPParaVec& point, const BPParaTransform& arc);
	bool isPoint0nArcCconst(BPParaVec& point, const BPParaTransform& arc);
	bool isPointOnLine(const BPParaVec& point, const PGSegment& line);
	bool isPointOnPLane(const BPParaVec& point, BPParaTransform& pLane);
	GRTwoVectorsDirection isTwoVectorsSameDirection(const BPParaVec& vecA, const BPParaVec& vecB);

	//para driven 
	//bool isPointOnSphere(const BPParaVec& point, double R);
	//bool isPointOnCylinder(const BPParaVec& point, double R); //default axis is axisZ
	//bool isPointOnSphereSetableCconst(BPParaVec& point, double R, const std::set<BPParaParameterIndex>& paraDriven);
	//BPParaVec makePointOnSphere(const BPParaVec& point, double R, const std::set<BPParaParameterIndex>& paraDriven = {});
	//bool isPointOnCylinderSetabLe(const BPParaVec& point, double R, const std::set<BPParaParameterIndex>& paraDriven);
	//BPParaVec makePointOnCylinder(const BPParaVec& point, double R, const std::set<BPParaParameterIndex>& paraDriven = {});
	//BPParaVec makePointOnCylinder(const PGSegment& base, const BPParaVec& point, double R);
	// support arbitrary line to keep distance, in BPGSegment member function

	// line-property
	//bool isPartLocateOnPlane(const BPGeometricPrimitive& part, const BPGeometricPrimitive& PGLine);
	//ParaLineType getLineType(const PGLine& line);
	//bool isLineOnSamePlane(const PGLine& line); // 
	//bool isLineInTwoDimensional(const PGLine& line); //local on XoY plane
	//PGLine multiplyOfMatrixAndLine(const BPParaTransform& mat, const PGLine& part); //instead operator*
	//std::vector<BPGeometricPrimitive> getPartsFromPGLine(const PGLine& line);
	//std::vector<const BPParaVec&> getPointsOnUnifiedPlane(const std::vector<const BPParaVec&>& points);

	//----------------------------------------------------------------------------------------------
	// 	                                        Relationship
	//----------------------------------------------------------------------------------------------
			//点与线段
	enum class PPCPointAndSegmentRalation
	{
		POINT_IN, //共线-线段内
		POINT_END, //共线-线段端点
		POINT_EXTEND, //共线-线段外
		POINT_OUT, //线段外
	};

	//点与圆
	enum class PPCPointAndCircleRalation
	{
		//共面
		POINT_CIRCLE_OUT, //点在圆外
		POINT_CIRCLE_IN, //点在圆内
		POINT_CIRCLE_ON, //点在圆上
		//不共面
		POINT_NONCOPLANAR，
	};

	//点与圆弧
	enum class PPCPointAndArcRalation
	{
		//共面
		POINT_ARC_OUT, //点在圆弧（圆弧所在圆）外
		POINT_ARC_IN, //点在圆弧（圆弧所在圆）内
		POINT_ARC_ON, //点在圆弧上
		POINT_ARC_END, //点在圆弧端点
		//不共面
		POINT_NONCOPLANAR，
	};

	//点与多边形
	enum class PPCPointAndPolygonRalation
	{
		//共面
		POINT_POLYGON_OUT, //点在多边形外
		POINT_POLYGON_IN, //点在多边形内
		POINT_POLYGON_ON, //点在多边形边上
		POINT_POLYGON_VERTEX, //点在多边形顶点
		//不共面
		POINT_NONCOPLANAR，
	};
	//矢量方向
	enum class PPCTwoVectorsDirection
	{
		VECTORS_SAME,
		VECTORS_OPPOSITE,
		VECTORS_VERTICAL,
		VECTORS_NONE,
		VECTORS_ANY, //零矢量导致任意方向
	};

	//----------------------------------------------------------

	//两条直线的的关系
	enum class PPCTwoLinesRalation
	{
		LINES_ZERO, //零向量
		LINES_NONCOP, //不共面
		LINES_PARA,// 平行
		LINES_COLLINE,// 共线
		LINES_INTER,// 相交

		PARALLEL_H, //平行-H形（平行相离）
		PARALLEL_I, //平行-I形（共线相接）
		PARALLEL_C, //平行-C形（共线相交）
		PARALLEL_i, //平行-i形（共线相离）
		INTERSECT_V, //相交-V形（相接/垂直）
		INTERSECT_T, //相交-T形
		INTERSECT_X, //相交-X形
		SEPARATE,    //共面相离

		SEGMENT_COLLINE, //两线共线
		SEGMENT_END, //直线过线段端点
		SEGMENT_INTER, //直线穿过线段
		SEGMENT_SEPA, //相离
	};


	//两个线段之间的关系
	enum class PPCTwoSegmentsRelation
	{
		SEGMENTS_PARALLEL_H, //平行-H形（相离）
		SEGMENTS_PARALLEL_I, //平行-I形（共线相接）
		SEGMENTS_PARALLEL_C, //平行-C形（共线相交）
		SEGMENTS_PARALLEL_i, //平行-i形（共线相离）

		SEGMENTS_INTERSECT_V, //相交-V形（相接）（垂直）
		SEGMENTS_INTERSECT_T, //相交-T形（回头线）
		SEGMENTS_INTERSECT_X, //相交-X形
		//SEGMENTS_INTERSECT_L, //相交-L形
		SEGMENTS_SEPARATE, //共面相离
		SEGMENTS_NONCOPLANAR, //不共面
	};
	//直线与平面的关系
	enum class PPCLineAndPlaneRelation
	{
		LINE_PLANE_ON, //直线在平面上
		LINE_PLANE_PARALLEL, //直线平行于平面（相离）
		LINE_PLANE_INTERSECT, //相交（不垂直）
		LINE_PLANE_VERTICAL, //垂直
	};

	//直线与圆之间的关系
	enum class PPCLineAndArcRelation
	{
		//共面
		LINE_CIRCLE_INTERSECT, //相交
		LINE_CIRCLE_TANGENT, //相切
		LINE_CIRCLE_SEPARATE, //相离
		//不共面
		LINE_CIRCLE_VERTICAL_IN, //垂直，line-plane交点在圆内
		LINE_CIRCLE_VERTICAL_ON, //垂直，line-plane交点在圆上
		LINE_CIRCLE_VERTICAL_OUT, //垂直，line-plane交点在圆外
		LINE_CIRCLE_POINT_IN, //不垂直，line-plane交点在圆内
		LINE_CIRCLE_POINT_ON, //不垂直，line-plane交点在圆上
		LINE_CIRCLE_POINT_OUT, //不垂直，line-plane交点在圆外
	};
	//直线与圆之间的关系
	enum class PPCTwoArcsRelation
	{
		TWO_ARCS_NONCOP,//不共面
		TWO_ARCS_INTER,// 相交
		TWO_ARCS_TANGEN,// 相切
		TWO_ARCS_SEPA,// 相离
	};

	//线段与圆弧之间的关系
	enum class GRSegmentAndArcRelation
	{	//共面
		SEGMENT_ARC_INTERSECT_1, //相交-1个交点
		SEGMENT_ARC_INTERSECT_2, //相交-2个交点
		SEGMENT_ARC_JOINED_1, //相接-1个交点（连续）
		SEGMENT_ARC_JOINED_2, //相接-2个交点（闭合）
		SEGMENT_ARC_TANGENT_K, //相切-K形
		SEGMENT_ARC_TANGENT_P, //相切-P形
		SEGMENT_ARC_TANGENT_J, //相切-J形
		SEGMENT_ARC_SEPARATE_K, //相离-K形（离->离）
		SEGMENT_ARC_SEPARATE_P, //相离-P形（切->离）
		SEGMENT_ARC_SEPARATE_J, //相离-J形（交->离）
		//不共面
		SEGMENT_ARC_NONCOPLANAR_INTERSECT, //	不共面相交
		SEGMENT_ARC_NONCOPLANAR_JOINED, //	不共面相接
		SEGMENT_ARC_NONCOPLANAR_SEPARATE, //	不共面相离
	};

	// 平面关系
	enum class PPCTwoPlaneRelation
	{
		PLANES_PARALLEL, //平面平行
		PLANES_COPLANAR, //平面共面
		PLANES_INTERSECT, //平面相交（不垂直）
		PLANES_VERTICAL, //平面垂直（z轴夹角pi/2）
	};

	//圆与平面的关系
	enum class GRArcAndPlaneRelation
	{
		ARC_PLANE_COPLANAR, //圆在平面上
		ARC_PLANE_INTERSECT, //圆与平面相交
		ARC_PLANE_PARALLEL, //圆与平面平行
		ARC_PLANE_SEPARATE, //圆与平面相离（不平行）
		ARC_PLANE_VERTICAL, //圆（所在平面）与平面垂直
		ARC_PLANE_TANGENT, //圆与平面相切
	};

	//两个圆的关系
	enum class PPCTwoCirclesRelation
	{
		//共面
		CIRCLES_INTERSECT, //相交
		CIRCLES_TANGENT_IN, //内切
		CIRCLES_TANGENT_OUT, //外切
		CIRCLES_SEPARATE_IN, //相离-包含
		CIRCLES_SEPARATE_OUT, //相离
		//不共面
		CIRCLES_NONCOPLANAR,
	};
	//两段圆弧的关系
	//enum class PPCTwoArcsRelation
	//{	//共面
	//	ARCS_IS_FULL_1, //有1个满圆
	//	ARCS_IS_FULL_2, //有2个满圆
	//	ARCS_JOINED_1, //相接（1个交点）
	//	ARCS_JOINED_2, //相接（2个交点，闭合）
	//	ARCS_INTERSECT_1, //相交（1个交点）
	//	ARCS_INTERSECT_2, //相交（2个交点）
	//	ARCS_INTERSECT_3, //相交（有一段圆弧重合）
	//	ARCS_SEPARATE, //相离
	//	//不共面
	//	ARCS_NONCOPLANAR_SEPARATE, //相离
	//	ARCS_NONCOPLANAR_JOINED_1, //相接（1个交点）
	//	ARCS_NONCOPLANAR_JOINED_2, //相接（2个交点，闭合）
	//};

	//多边形
	enum class PPCPointAndContourlineRalation //点在多边形内（包含在轮廓线上）
	{
		POINT_CONTOUR_OUT, //多边形外
		POINT_CONTOUR_IN, //多边形内
		POINT_CONTOUR_ON, //多边形边上
		POINT_PLANE_OUT, //与多边形不共面
	};


	//-----------------------------------------------------------------
	//矢量
	//PPCTwoVectorsDirection isTwoVectorsSameDirection(const BPParaVec& vec1, const BPParaVec& vec2);
	//double getAngleOfTwoVectors(const BPParaVec& vec1, const BPParaVec& vec2);

	//点
	//line
	bool isPointOnLine(const BPParaVec& point, const PGSegment& line, bool isLine = true);//判断点是否在直线上
	PPCPointAndSegmentRalation isPointOnSegment(const BPParaVec& point, const PGSegment& segm);//判断点是否在线段上
	double getDistanceOfPointLine(const BPParaVec& point, const PGSegment& line);
	BPParaVec getNearestPointOfPointLine(const BPParaVec& point, const PGSegment& line); //获取其最近的点
	PPCPointAndArcRalation isPointOnArc(const BPParaVec& point, const PGArc& arc, bool isCircle = false);
	//Plane
	bool isPointLocateOnPlane(const BPParaVec& point, const BPParaTransform& mat);//点是否在平面上（平面的局部坐标系）
	double getDistanceOfPointPlane(const BPParaVec& point, const BPParaTransform& mat); //点到平面的距离
	//line
	PPCTwoLinesRalation isTwoLinesIntersect(const PGSegment& lineA, const PGSegment& lineB, bool isLineA = false, bool isLineB = false);
	bool isTwoSegmentsIntersect2D(const PGSegment& segmA, const PGSegment& segmB, bool ignoreZero = true);
	std::vector<const BPParaVec&> getIntersectPointOfTwoLines(const PGSegment& lineA, const PGSegment& lineB, bool isLineA = false, bool isLineB = false); //获取两条直线的交点
	PPCTwoLinesRalation getIntersectPointOfTwoLines(const PGSegment& lineA, const PGSegment& lineB, std::vector<const BPParaVec&>& point); //获取两条直线的交点
	double getDistanceOfTwoLines(const PGSegment& line1, const PGSegment& line2); //空间直线的距离
	double getAngleOfTwoLines2D(const PGSegment& line1, const PGSegment& line2); //平面直线的夹角
	//Arc
	PPCLineAndArcRelation isLineArcIntersect(const PGSegment& line, const PGSegment& arc, bool isLine = false, bool isCircle = false);
	std::vector<const BPParaVec&> getIntersectPointOfLineArc(const PGSegment& line, const PGArc& arc, bool isLine = false, bool isCircle = false);  //如果有交点，获得其交点
	PPCTwoArcsRelation isTwoArcsIntersect(const PGArc& arcA, const PGArc& arcB, bool isBothCircle = false);
	std::vector<const BPParaVec&> getIntersectPointOfTwoArcs(const PGArc& arcA, const PGArc& arcB, bool isBothCircle = false);  //如果有交点，获得其交点
	//spline

	//plane
	PPCLineAndPlaneRelation isLineOnPlane(const PGSegment& line, const BPParaTransform& plane);
	BPParaVec getPointOfLinePlane(const PGSegment& segm, const BPParaTransform& plane);  //如果有交点，获得其交点

	//面
	PPCTwoPlaneRelation isTwoPlanesIntersect(const BPParaTransform& planeO, const BPParaTransform& planeA);
	double getDistanceOfTwoPlanes(const BPParaTransform& planeA, const BPParaTransform& planeB); //如果平行面，获得其距离
	PGSegment getIntersectLineOfTwoPlanes(const BPParaTransform& planeA, const BPParaTransform& planeB);  //如果有交线，获得其交线
	//Line
	bool isLineLocateOnPlane(const PGSegment& line, const BPParaTransform& plane, bool isLine = false);
	std::vector<const BPParaVec&> getIntersectPointOfLinePlane(const PGSegment& line, const BPParaTransform& plane, bool isLine = false);
	bool isArcLocateOnPlane(const PGArc& arc, const BPParaTransform& plane);
	GRArcAndPlaneRelation isArcLocateOnPlane(const PGArc& arc, const BPParaTransform& plane, bool isCircle /*= false*/);
	std::vector<const BPParaVec&> getIntersectPointOfArcPlane(const PGArc& arc, const BPParaTransform& plane, bool isCircle = false);
	//spline
	std::vector<const BPParaVec&> getIntersectPointsOfFragments(Gnrc para1, Gnrc para2);
	//polygon
	PPCPointAndContourlineRalation isPointInContourline(const BPParaVec& point, para::PGLine contour);
	PPCPointAndContourlineRalation isPointInContourline(const BPParaVec& point, std::vector<const BPParaVec&> polyline);
	bool isPointOnPolygon(const BPParaVec& point, std::vector<const BPParaVec&> polygon); //点在轮廓线上
	bool isPointInPolygon(const BPParaVec& point, std::vector<const BPParaVec&> polygon); //点在轮廓线上或多边形内（射线法）
	bool isPolygonSelfIntersect(std::vector<const BPParaVec&> points); //多边形自相交
	bool isContourlineSelfIntersect(std::vector<const BPParaVec&> points); //多边形自相交
	bool isTwoSectionsIntersect(std::vector<const BPParaVec&> points); //截面轮廓线相交判定



}
