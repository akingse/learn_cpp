#include "pch.h"
using namespace std;
namespace para
{
	////plane
	//PGPlane::PGPlane(const BPParaTransform& mat)
	//{
	//	BPParaVec vecZ = getMatrixsAxisZ(mat);
	//	BPParaVec point = getMatrixsPosition(mat);
	//	m_A = vecZ.x();
	//	m_B = vecZ.y();
	//	m_C = vecZ.z();
	//	m_D = -(m_A * point.x() + m_B * point.y() + m_C * point.z());
	//}
	//BPParaTransform PGPlane::getMatrix() const
	//{
	//	if (!isValid())
	//		return g_MatrixO;
	//	BPParaVec vecZ(m_A, m_D, m_C);
	//	BPParaVec point;
	//	if (!isFloatZero(m_C))
	//		point = BPParaVec(0, 0, -m_D / m_C);
	//	else if (!isFloatZero(m_B))
	//		point = BPParaVec(0, -m_D / m_B, 0);
	//	else
	//		point = BPParaVec(-m_D / m_A, 0, 0);
	//	return getMatrixFromTwoPoints(point, point + vecZ, true);
	//}

	//BPParaPosVec declare before BPParaSegment
	PGPosVec::PGPosVec(const PGSegment& segm) :m_pos(segm.start()), m_vec(segm.vector())
	{
	}

	//Arc
	PGArc::PGArc():m_mat(BPParaTransform()),m_scope(2*M_PI)
	{
	}
	PGArc::PGArc(const BPParaTransform& transformation, double scope) :
		m_mat(transformation),
		m_scope(scope)
	{
	}
	PGArc::~PGArc()
	{
	}
	//namespace para
	//{
	//	PGArc::PGArc(const GeoArc& arc)
	//	{
	//		m_mat = arc.getTransform();
	//		m_scope = arc.getScope();
	//	}
	//}

	//PGPosVec PGArc::start()
	//{
	//	if (isZeroMatrix(m_transform))
	//		return PGPosVec();
	//	return PGPosVec(m_transform * BPParaVec(1, 0, 0), unitize(m_transform * BPParaVec(0, 1, 0)));
	//}
	//PGPosVec PGArc::end()
	//{
	//	if (isZeroMatrix(m_transform))
	//		return PGPosVec();
	//	BPParaVec v_end(cos(m_scope), sin(m_scope), 0);
	//	return PGPosVec(m_transform * v_end, unitize(m_transform * (v_end ^ BPParaVec(0, 0, -1))));
	//}

	PGSplineCurve::PGSplineCurve()
	{
	}
	PGSplineCurve::PGSplineCurve(const std::vector<BPParaVec>& points, long long discNum, int k, const std::string& type) :
		m_points(points),
		m_num(discNum),
		m_k(k),
		m_type(type)
		//m_transform(transform)
	{
	}
	PGSplineCurve::~PGSplineCurve()
	{
	}
	//PGPosVec PGSplineCurve::start() const
	//{
	//	if (m_points.size() == 0)
	//		return PGPosVec();
	//	else if (m_points.size() == 1)
	//		return PGPosVec(m_transform * m_points[0], BPParaVec());
	//	else
	//		return PGPosVec(m_transform * m_points[0], m_transform * unitize(m_points[1] - m_points[0]));
	//}
	//PGPosVec PGSplineCurve::end() const
	//{
	//	if (m_points.size() == 0)
	//		return PGPosVec();
	//	else if (m_points.size() == 1)
	//		return PGPosVec(m_transform * m_points[0], BPParaVec());
	//	else
	//		return PGPosVec(m_transform * m_points[m_points.size() - 1], m_transform * unitize(m_points[m_points.size() - 1] - m_points[m_points.size() - 2]));
	//}

	//vector<BPParaVec> PGSplineCurve::getDiscretePoints() const
	//{
	//	vector<BPParaVec> points = createSplineCurveQuasi(m_points, m_num, m_k);
	//	return m_transform * points;
	//}

}