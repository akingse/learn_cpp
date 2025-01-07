#pragma once
//using namespace std::rel_ops;
class GeoArc;
namespace para
{
	//----------------------------------------------------------------------------------------------
	// 	                                        PGPlane
	//----------------------------------------------------------------------------------------------
	//using BPParaPlane
	//typedef BPParaPlane PGPlane;
	//class PGPlane  
	//{
	//private:
	//	double m_A;
	//	double m_B;
	//	double m_C;
	//	double m_D;
	//public:
	//	PGPlane():
	//		m_A(0), m_B(0), m_C(0), m_D(0){}
	//	PGPlane(double A, double B, double C, double D)
	//		:m_A(A), m_B(B), m_C(C), m_D(D) {}
	//	PGPlane(const BPParaTransform& mat);
	//	~PGPlane() {}
	//	bool isValid() const 
	//	{
	//		return !(isFloatZero(m_A) && isFloatZero(m_B) && isFloatZero(m_C));
	//	}
	//	/// <summary>
	//	/// using matrix XoY plane
	//	/// </summary>
	//	/// <returns></returns>
	//	BPParaTransform getMatrix() const;
	//	inline operator BPParaTransform() const
	//	{
	//		return getMatrix();
	//	}
	//};

	class PGFragment //virtual interface class
	{
	public:
		PGFragment() {};
		virtual ~PGFragment() {};
		virtual BPParaVec start() const { return g_axisNaN; };
		virtual BPParaVec end() const { return g_axisNaN; };
		virtual BPParaVec startVec() const /*= 0*/ { return BPParaVec(); };
		virtual BPParaVec endVec() const /*= 0*/ { return BPParaVec(); };
		virtual double length() const /*= 0*/ { return 0; };
	};

//----------------------------------------------------------------------------------------------
// 	                                        PGPosVec
//----------------------------------------------------------------------------------------------

	class PGSegment;
	// (parametric geometry) the PosVec class only for calculate, include position and vector 
	class PGPosVec
	{
	private:
		BPParaVec m_pos;
		BPParaVec m_vec;
	public:
		PGPosVec() :m_pos(BPParaVec()), m_vec(BPParaVec()) {};
		PGPosVec(const BPParaVec& pos, const BPParaVec& vec) :m_pos(pos), m_vec(vec) {}
		PGPosVec(const PGSegment& segm);
		PGPosVec getUnitize() const
		{
			return PGPosVec(m_pos, unitize(m_vec));
		}
		BPParaVec pos() const
		{
			return m_pos;
		}
		BPParaVec vec() const
		{
			return m_vec;
		}
		BPParaVec start() const
		{
			return m_pos;
		}
		BPParaVec end() const
		{
			return m_pos + m_vec;
		}
		BPParaVec operator[](bool i) const //using index get
		{
			return i ? m_vec : m_pos;
		}
		inline bool operator<(const PGPosVec& rhs) const
		{
			return m_pos < rhs.pos() || m_vec < rhs.vec();
		}
		inline bool operator==(const PGPosVec& rhs) const
		{
			return m_pos == rhs.pos() && m_vec == rhs.vec();
		}
		
	};
	inline PGPosVec operator*(const BPParaTransform& mat, const PGPosVec& pv)
	{
		return PGPosVec(mat * pv.start(), getMatrixsRotationPart(mat) * pv.vec());
	}


//----------------------------------------------------------------------------------------------
// 	                                        PGSegment
//----------------------------------------------------------------------------------------------
	class PGSegment :public PGFragment
	{
	private:
		BPParaVec m_start;
		BPParaVec m_end;
	public:
		__declspec(dllexport) PGSegment() :m_start(g_axisNaN), m_end(g_axisNaN) {};// = delete;
		__declspec(dllexport) PGSegment(const BPParaVec& pStart, const BPParaVec& pEnd) :m_start(pStart), m_end(pEnd) {}
		__declspec(dllexport) ~PGSegment() {};
		//geometry attribute
		bool isCoincident() const
		{
			return para::isCoincident(end(), start());
		}
		virtual BPParaVec start() const override
		{
			return m_start;
		}
		virtual BPParaVec end() const override
		{
			return m_end;
		}
		virtual BPParaVec startVec() const override
		{
			return unitize(m_end - m_start);
		}
		virtual BPParaVec endVec() const override
		{
			return unitize(m_end - m_start);
		}
		virtual double length() const override
		{
			return norm(end() - start());
		}
		BPParaVec vector() const //the direciton vector from start to end
		{
			return end() - start();
		}
		BPParaVec vectorU() const//the unitize vector from start to end
		{
			return unitize(end() - start());
		}
		inline bool operator<(const PGSegment& rhs) const
		{
			return m_start < rhs.start() || m_end < rhs.end();
		}
		inline bool operator==(const PGSegment& rhs) const
		{
			return m_start == rhs.start() && m_end == rhs.end();
		}
		inline BPParaVec operator[](bool i) const //using index get
		{
			return i ? end() : start();
		}
	};
	inline PGSegment operator*(const BPParaTransform& mat, const PGSegment& segm)
	{
		return PGSegment(mat * segm.start(), mat * segm.end());
	}
//----------------------------------------------------------------------------------------------
// 	                                        PGArc
//----------------------------------------------------------------------------------------------

	class PGArc :public PGFragment
	{
	public:
		BPParaTransform m_mat;
		double m_scope;
	public:
		__declspec(dllexport) PGArc();
		__declspec(dllexport) PGArc(const BPParaTransform& transformation, double scope = 2 * M_PI);
		__declspec(dllexport) ~PGArc();
		virtual BPParaVec start() const override
		{
			return m_mat * BPParaVec(1, 0, 0);

		}
		virtual BPParaVec end() const override
		{
			BPParaVec pEnd(cos(m_scope), sin(m_scope), 0);
			return m_mat * pEnd;
		}
		virtual BPParaVec startVec() const override //vectorTangentStart
		{
			BPParaVec vectorZ = getMatrixsAxisZ(getOrthogonalMatrix(m_mat));
			BPParaVec vectorStart = start() - center();
			return mathSign(m_scope) * unitize((vectorZ ^ vectorStart));
		}
		virtual BPParaVec endVec() const override //vectorTangentEnd
		{
			BPParaVec vectorZ = getMatrixsAxisZ(getOrthogonalMatrix(m_mat));
			BPParaVec vectorEnd = end() - center();
			return mathSign(m_scope) * unitize((vectorZ ^ vectorEnd));
		}
		virtual double length() const override
		{
			return norm(end() - start());
		}
		BPParaVec center() const
		{
			return getMatrixsPosition(m_mat);
		}
		BPParaVec vectorNormal() const
		{
			BPParaVec axisx = getMatrixsAxisX(m_mat);
			BPParaVec axisy = getMatrixsAxisY(m_mat);
			return unitize(axisx ^ axisy);
		}
		bool isFull()
		{
			return false;
		}
		bool isCircle()
		{
			return false;
		}
	};
//----------------------------------------------------------------------------------------------
// 	                                        PGSplineCurve
//----------------------------------------------------------------------------------------------

	class PGSplineCurve :public PGFragment
	{
	public:
		std::vector<BPParaVec> m_points;
		long long m_num;
		int m_k;
		std::string m_type;
		BPParaTransform m_transform = BPParaTransform();
	public:
		__declspec(dllexport) PGSplineCurve();
		__declspec(dllexport) PGSplineCurve(const std::vector<BPParaVec>& points, long long discNum = 0, int k = 2, const std::string& type = "quasi");
		__declspec(dllexport) ~PGSplineCurve();
		virtual BPParaVec start() const override
		{
			return (m_points.size() > 0) ? *m_points.begin() : g_axisNaN;
		}
		virtual BPParaVec end() const override
		{
			return (m_points.size() > 0) ? *m_points.end() : g_axisNaN;
		}
		virtual BPParaVec startVec() const override
		{
			return BPParaVec();
		}
		virtual BPParaVec endVec() const override
		{
			return BPParaVec();
		}
		virtual double length() const override
		{
			return 0;
		}
		//std::vector<BPParaVec> getDiscretePoints() const { return createSplineCurveQuasi(m_points, m_num, m_k); }
	};

	//----------------------------------------------------------------------------------------------
	// 	                                        broad line
	//----------------------------------------------------------------------------------------------


	//----------------------------------------------------------------------------------------------
	// 	                                        broad line
	//----------------------------------------------------------------------------------------------

	enum class ParaLineType
	{
		LINE_NONE,
		LINE_XOY, // on XoY plane
		LINE_SECTION, // on same plane
		LINE_STERE, // not on same plane
	};
	class PGLine
	{
		std::vector<PGFragment> m_parts;
	public:
		__declspec(dllexport) PGLine() {}
		__declspec(dllexport) PGLine(const std::vector<PGFragment>& parts) :m_parts(parts)
		{}
		__declspec(dllexport) ~PGLine() {}
		bool isClose() const;
		std::vector<PGFragment> get() const { return m_parts; }
		std::vector<PGFragment> getNestedParts() const;
		ParaLineType getType() const; //getLineType
		BPParaVec start() const;
		BPParaVec startVector() const;
		BPParaVec end() const;
		BPParaVec endVector() const;
		PGLine multipyMatrixLeft(const BPParaTransform& mat) const;
	};
	PGLine operator*(const BPParaTransform& mat, const PGLine& line);

}