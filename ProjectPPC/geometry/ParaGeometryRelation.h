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
			//�����߶�
	enum class PPCPointAndSegmentRalation
	{
		POINT_IN, //����-�߶���
		POINT_END, //����-�߶ζ˵�
		POINT_EXTEND, //����-�߶���
		POINT_OUT, //�߶���
	};

	//����Բ
	enum class PPCPointAndCircleRalation
	{
		//����
		POINT_CIRCLE_OUT, //����Բ��
		POINT_CIRCLE_IN, //����Բ��
		POINT_CIRCLE_ON, //����Բ��
		//������
		POINT_NONCOPLANAR��
	};

	//����Բ��
	enum class PPCPointAndArcRalation
	{
		//����
		POINT_ARC_OUT, //����Բ����Բ������Բ����
		POINT_ARC_IN, //����Բ����Բ������Բ����
		POINT_ARC_ON, //����Բ����
		POINT_ARC_END, //����Բ���˵�
		//������
		POINT_NONCOPLANAR��
	};

	//��������
	enum class PPCPointAndPolygonRalation
	{
		//����
		POINT_POLYGON_OUT, //���ڶ������
		POINT_POLYGON_IN, //���ڶ������
		POINT_POLYGON_ON, //���ڶ���α���
		POINT_POLYGON_VERTEX, //���ڶ���ζ���
		//������
		POINT_NONCOPLANAR��
	};
	//ʸ������
	enum class PPCTwoVectorsDirection
	{
		VECTORS_SAME,
		VECTORS_OPPOSITE,
		VECTORS_VERTICAL,
		VECTORS_NONE,
		VECTORS_ANY, //��ʸ���������ⷽ��
	};

	//----------------------------------------------------------

	//����ֱ�ߵĵĹ�ϵ
	enum class PPCTwoLinesRalation
	{
		LINES_ZERO, //������
		LINES_NONCOP, //������
		LINES_PARA,// ƽ��
		LINES_COLLINE,// ����
		LINES_INTER,// �ཻ

		PARALLEL_H, //ƽ��-H�Σ�ƽ�����룩
		PARALLEL_I, //ƽ��-I�Σ�������ӣ�
		PARALLEL_C, //ƽ��-C�Σ������ཻ��
		PARALLEL_i, //ƽ��-i�Σ��������룩
		INTERSECT_V, //�ཻ-V�Σ����/��ֱ��
		INTERSECT_T, //�ཻ-T��
		INTERSECT_X, //�ཻ-X��
		SEPARATE,    //��������

		SEGMENT_COLLINE, //���߹���
		SEGMENT_END, //ֱ�߹��߶ζ˵�
		SEGMENT_INTER, //ֱ�ߴ����߶�
		SEGMENT_SEPA, //����
	};


	//�����߶�֮��Ĺ�ϵ
	enum class PPCTwoSegmentsRelation
	{
		SEGMENTS_PARALLEL_H, //ƽ��-H�Σ����룩
		SEGMENTS_PARALLEL_I, //ƽ��-I�Σ�������ӣ�
		SEGMENTS_PARALLEL_C, //ƽ��-C�Σ������ཻ��
		SEGMENTS_PARALLEL_i, //ƽ��-i�Σ��������룩

		SEGMENTS_INTERSECT_V, //�ཻ-V�Σ���ӣ�����ֱ��
		SEGMENTS_INTERSECT_T, //�ཻ-T�Σ���ͷ�ߣ�
		SEGMENTS_INTERSECT_X, //�ཻ-X��
		//SEGMENTS_INTERSECT_L, //�ཻ-L��
		SEGMENTS_SEPARATE, //��������
		SEGMENTS_NONCOPLANAR, //������
	};
	//ֱ����ƽ��Ĺ�ϵ
	enum class PPCLineAndPlaneRelation
	{
		LINE_PLANE_ON, //ֱ����ƽ����
		LINE_PLANE_PARALLEL, //ֱ��ƽ����ƽ�棨���룩
		LINE_PLANE_INTERSECT, //�ཻ������ֱ��
		LINE_PLANE_VERTICAL, //��ֱ
	};

	//ֱ����Բ֮��Ĺ�ϵ
	enum class PPCLineAndArcRelation
	{
		//����
		LINE_CIRCLE_INTERSECT, //�ཻ
		LINE_CIRCLE_TANGENT, //����
		LINE_CIRCLE_SEPARATE, //����
		//������
		LINE_CIRCLE_VERTICAL_IN, //��ֱ��line-plane������Բ��
		LINE_CIRCLE_VERTICAL_ON, //��ֱ��line-plane������Բ��
		LINE_CIRCLE_VERTICAL_OUT, //��ֱ��line-plane������Բ��
		LINE_CIRCLE_POINT_IN, //����ֱ��line-plane������Բ��
		LINE_CIRCLE_POINT_ON, //����ֱ��line-plane������Բ��
		LINE_CIRCLE_POINT_OUT, //����ֱ��line-plane������Բ��
	};
	//ֱ����Բ֮��Ĺ�ϵ
	enum class PPCTwoArcsRelation
	{
		TWO_ARCS_NONCOP,//������
		TWO_ARCS_INTER,// �ཻ
		TWO_ARCS_TANGEN,// ����
		TWO_ARCS_SEPA,// ����
	};

	//�߶���Բ��֮��Ĺ�ϵ
	enum class GRSegmentAndArcRelation
	{	//����
		SEGMENT_ARC_INTERSECT_1, //�ཻ-1������
		SEGMENT_ARC_INTERSECT_2, //�ཻ-2������
		SEGMENT_ARC_JOINED_1, //���-1�����㣨������
		SEGMENT_ARC_JOINED_2, //���-2�����㣨�պϣ�
		SEGMENT_ARC_TANGENT_K, //����-K��
		SEGMENT_ARC_TANGENT_P, //����-P��
		SEGMENT_ARC_TANGENT_J, //����-J��
		SEGMENT_ARC_SEPARATE_K, //����-K�Σ���->�룩
		SEGMENT_ARC_SEPARATE_P, //����-P�Σ���->�룩
		SEGMENT_ARC_SEPARATE_J, //����-J�Σ���->�룩
		//������
		SEGMENT_ARC_NONCOPLANAR_INTERSECT, //	�������ཻ
		SEGMENT_ARC_NONCOPLANAR_JOINED, //	���������
		SEGMENT_ARC_NONCOPLANAR_SEPARATE, //	����������
	};

	// ƽ���ϵ
	enum class PPCTwoPlaneRelation
	{
		PLANES_PARALLEL, //ƽ��ƽ��
		PLANES_COPLANAR, //ƽ�湲��
		PLANES_INTERSECT, //ƽ���ཻ������ֱ��
		PLANES_VERTICAL, //ƽ�洹ֱ��z��н�pi/2��
	};

	//Բ��ƽ��Ĺ�ϵ
	enum class GRArcAndPlaneRelation
	{
		ARC_PLANE_COPLANAR, //Բ��ƽ����
		ARC_PLANE_INTERSECT, //Բ��ƽ���ཻ
		ARC_PLANE_PARALLEL, //Բ��ƽ��ƽ��
		ARC_PLANE_SEPARATE, //Բ��ƽ�����루��ƽ�У�
		ARC_PLANE_VERTICAL, //Բ������ƽ�棩��ƽ�洹ֱ
		ARC_PLANE_TANGENT, //Բ��ƽ������
	};

	//����Բ�Ĺ�ϵ
	enum class PPCTwoCirclesRelation
	{
		//����
		CIRCLES_INTERSECT, //�ཻ
		CIRCLES_TANGENT_IN, //����
		CIRCLES_TANGENT_OUT, //����
		CIRCLES_SEPARATE_IN, //����-����
		CIRCLES_SEPARATE_OUT, //����
		//������
		CIRCLES_NONCOPLANAR,
	};
	//����Բ���Ĺ�ϵ
	//enum class PPCTwoArcsRelation
	//{	//����
	//	ARCS_IS_FULL_1, //��1����Բ
	//	ARCS_IS_FULL_2, //��2����Բ
	//	ARCS_JOINED_1, //��ӣ�1�����㣩
	//	ARCS_JOINED_2, //��ӣ�2�����㣬�պϣ�
	//	ARCS_INTERSECT_1, //�ཻ��1�����㣩
	//	ARCS_INTERSECT_2, //�ཻ��2�����㣩
	//	ARCS_INTERSECT_3, //�ཻ����һ��Բ���غϣ�
	//	ARCS_SEPARATE, //����
	//	//������
	//	ARCS_NONCOPLANAR_SEPARATE, //����
	//	ARCS_NONCOPLANAR_JOINED_1, //��ӣ�1�����㣩
	//	ARCS_NONCOPLANAR_JOINED_2, //��ӣ�2�����㣬�պϣ�
	//};

	//�����
	enum class PPCPointAndContourlineRalation //���ڶ�����ڣ��������������ϣ�
	{
		POINT_CONTOUR_OUT, //�������
		POINT_CONTOUR_IN, //�������
		POINT_CONTOUR_ON, //����α���
		POINT_PLANE_OUT, //�����β�����
	};


	//-----------------------------------------------------------------
	//ʸ��
	//PPCTwoVectorsDirection isTwoVectorsSameDirection(const BPParaVec& vec1, const BPParaVec& vec2);
	//double getAngleOfTwoVectors(const BPParaVec& vec1, const BPParaVec& vec2);

	//��
	//line
	bool isPointOnLine(const BPParaVec& point, const PGSegment& line, bool isLine = true);//�жϵ��Ƿ���ֱ����
	PPCPointAndSegmentRalation isPointOnSegment(const BPParaVec& point, const PGSegment& segm);//�жϵ��Ƿ����߶���
	double getDistanceOfPointLine(const BPParaVec& point, const PGSegment& line);
	BPParaVec getNearestPointOfPointLine(const BPParaVec& point, const PGSegment& line); //��ȡ������ĵ�
	PPCPointAndArcRalation isPointOnArc(const BPParaVec& point, const PGArc& arc, bool isCircle = false);
	//Plane
	bool isPointLocateOnPlane(const BPParaVec& point, const BPParaTransform& mat);//���Ƿ���ƽ���ϣ�ƽ��ľֲ�����ϵ��
	double getDistanceOfPointPlane(const BPParaVec& point, const BPParaTransform& mat); //�㵽ƽ��ľ���
	//line
	PPCTwoLinesRalation isTwoLinesIntersect(const PGSegment& lineA, const PGSegment& lineB, bool isLineA = false, bool isLineB = false);
	bool isTwoSegmentsIntersect2D(const PGSegment& segmA, const PGSegment& segmB, bool ignoreZero = true);
	std::vector<const BPParaVec&> getIntersectPointOfTwoLines(const PGSegment& lineA, const PGSegment& lineB, bool isLineA = false, bool isLineB = false); //��ȡ����ֱ�ߵĽ���
	PPCTwoLinesRalation getIntersectPointOfTwoLines(const PGSegment& lineA, const PGSegment& lineB, std::vector<const BPParaVec&>& point); //��ȡ����ֱ�ߵĽ���
	double getDistanceOfTwoLines(const PGSegment& line1, const PGSegment& line2); //�ռ�ֱ�ߵľ���
	double getAngleOfTwoLines2D(const PGSegment& line1, const PGSegment& line2); //ƽ��ֱ�ߵļн�
	//Arc
	PPCLineAndArcRelation isLineArcIntersect(const PGSegment& line, const PGSegment& arc, bool isLine = false, bool isCircle = false);
	std::vector<const BPParaVec&> getIntersectPointOfLineArc(const PGSegment& line, const PGArc& arc, bool isLine = false, bool isCircle = false);  //����н��㣬����佻��
	PPCTwoArcsRelation isTwoArcsIntersect(const PGArc& arcA, const PGArc& arcB, bool isBothCircle = false);
	std::vector<const BPParaVec&> getIntersectPointOfTwoArcs(const PGArc& arcA, const PGArc& arcB, bool isBothCircle = false);  //����н��㣬����佻��
	//spline

	//plane
	PPCLineAndPlaneRelation isLineOnPlane(const PGSegment& line, const BPParaTransform& plane);
	BPParaVec getPointOfLinePlane(const PGSegment& segm, const BPParaTransform& plane);  //����н��㣬����佻��

	//��
	PPCTwoPlaneRelation isTwoPlanesIntersect(const BPParaTransform& planeO, const BPParaTransform& planeA);
	double getDistanceOfTwoPlanes(const BPParaTransform& planeA, const BPParaTransform& planeB); //���ƽ���棬��������
	PGSegment getIntersectLineOfTwoPlanes(const BPParaTransform& planeA, const BPParaTransform& planeB);  //����н��ߣ�����佻��
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
	bool isPointOnPolygon(const BPParaVec& point, std::vector<const BPParaVec&> polygon); //������������
	bool isPointInPolygon(const BPParaVec& point, std::vector<const BPParaVec&> polygon); //�����������ϻ������ڣ����߷���
	bool isPolygonSelfIntersect(std::vector<const BPParaVec&> points); //��������ཻ
	bool isContourlineSelfIntersect(std::vector<const BPParaVec&> points); //��������ཻ
	bool isTwoSectionsIntersect(std::vector<const BPParaVec&> points); //�����������ཻ�ж�



}
