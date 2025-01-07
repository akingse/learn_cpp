#pragma once
namespace para
{

	// using GB
	static const double PL_Length = 1e-10;// AccuracyLinearDimensions::value();
	static const double PL_Surface = 1e-6; //AccuracyLinearDimensions::square();
	static const double PL_Angle = 1e-10; //AccuracyAngularDimensions::value();
	extern "C"
	{
		struct CVec3 /*__declspec(dllexport) */
		{
			double x;
			double y;
			double z;
		};// vec = { 0,0,0 };

		//member
		__declspec(dllexport) CVec3 CVec3add(CVec3 a, CVec3 b); //+
		__declspec(dllexport) CVec3 CVec3sub(CVec3 a, CVec3 b); //-
		__declspec(dllexport) double CVec3dot(CVec3 a, CVec3 b); //*
		__declspec(dllexport) CVec3 CVec3cross(CVec3 a, CVec3 b); //^
		__declspec(dllexport) bool CVec3equal(CVec3 a, CVec3 b); //=
		__declspec(dllexport) bool CVec3less(CVec3 a, CVec3 b); //< (compare norm)
		__declspec(dllexport) double CVec3norm(CVec3 a); //||
		__declspec(dllexport) CVec3 CVec3unitize(CVec3 a); //v||

		// float deviation
		__declspec(dllexport) bool _isFloatZero(double num, double eps = 1e-10);//default 0.0, using abs percision
		__declspec(dllexport) bool _isFloatEqual(double numA, double numB, double eps = 0.0); //default 0.0, using auto percision
		//__declspec(dllexport) bool isParallel2d(CVec2 vecA, CVec2 vecB);
		__declspec(dllexport) bool isParallel3d(CVec3 vecA, CVec3 vecB);
		//__declspec(dllexport) bool isPerpendi2d(CVec2 vecA, CVec2 vecB);
		__declspec(dllexport) bool isPerpendi3d(CVec3 vecA, CVec3 vecB);
		//__declspec(dllexport) bool isCoincident2d(CVec2 vecA, CVec2 vecB);
		__declspec(dllexport) bool isCoincident3d(CVec3 vecA, CVec3 vecB);
		//__declspec(dllexport) double getAngleOfTwoVectors2d(CVec2 vecA, CVec2 vecB, bool isAbs = false);
		__declspec(dllexport) double getAngleOfTwoVectors3d(CVec3 vecA, CVec3 vecB, bool isAbs = false);
	}

	class BPParaVec2
	{
	public:
		double x;
		double y;
		__declspec(dllexport) BPParaVec2() :x(0.0), y(0.0) {}
		__declspec(dllexport) BPParaVec2(double _x, double _y) : x(_x), y(_y) {}
		__declspec(dllexport) ~BPParaVec2() {}
		__declspec(dllexport) BPParaVec2(const BPParaVec2& rhs) : x(rhs.x), y(rhs.y) {}
		//__declspec(dllexport) BPParaVec2 operator^(const BPParaVec2& rhs) const;
		__declspec(dllexport) double operator*(const BPParaVec2& rhs) const { return x * rhs.x, y* rhs.y; }
		__declspec(dllexport) BPParaVec2 operator+(const BPParaVec2& rhs) const { return BPParaVec2(x + rhs.x, y + rhs.y); }
		__declspec(dllexport) BPParaVec2 operator-(const BPParaVec2& rhs) const { return BPParaVec2(x - rhs.x, y - rhs.y); }
		__declspec(dllexport) bool operator<(const BPParaVec2& rhs) const;
		__declspec(dllexport) bool operator==(const BPParaVec2& rhs) const;
		__declspec(dllexport) double operator[](int i) const;
		inline BPParaVec2 operator+() { return *this; }
		inline BPParaVec2 operator-() { return BPParaVec2(-x, -y); }
	};

	class BPParaTransform;
	class BPParaVec
	{
	public:
		CVec3 m_imp;
		__declspec(dllexport) BPParaVec();
		__declspec(dllexport) BPParaVec(double x, double y /*= 0*/, double z = 0);
		__declspec(dllexport) ~BPParaVec();
		__declspec(dllexport) BPParaVec(const CVec3& src);
		__declspec(dllexport) BPParaVec(const BPParaVec& other);
		__declspec(dllexport) BPParaVec(BPParaVec&& other) = default;
		__declspec(dllexport) BPParaVec& operator=(const BPParaVec& other);
		__declspec(dllexport) BPParaVec operator^(const BPParaVec& other) const;
		__declspec(dllexport) BPParaVec operator^=(const BPParaVec& other);
		__declspec(dllexport) double operator*(const BPParaVec& other) const;
		__declspec(dllexport) BPParaVec operator+(const BPParaVec& other) const;
		__declspec(dllexport) BPParaVec operator+=(const BPParaVec& other);
		__declspec(dllexport) BPParaVec operator-(const BPParaVec& other) const;
		__declspec(dllexport) BPParaVec operator-=(const BPParaVec& other);
		__declspec(dllexport) bool operator<(const BPParaVec& other) const;
		__declspec(dllexport) bool operator==(const BPParaVec& other) const;
		__declspec(dllexport) double operator[](int i) const;
		inline BPParaVec operator+() { return *this; }
		inline BPParaVec operator-() { return BPParaVec(-m_imp.x, -m_imp.y, -m_imp.z); }
		inline double x() const { return m_imp.x; };
		inline double y() const { return m_imp.y; };
		inline double z() const { return m_imp.z; };
		__declspec(dllexport) double norm() const;
		__declspec(dllexport) BPParaVec& unitize();
		__declspec(dllexport) BPParaVec unitize() const { return BPParaVec(CVec3unitize(m_imp)); };
		__declspec(dllexport) bool isValid() const;
		__declspec(dllexport) bool isOrigin() const;
		__declspec(dllexport) bool isUnitize() const;
		__declspec(dllexport) bool isAcuteAngle(const BPParaVec& other) const; //exclusive right-angle
		__declspec(dllexport) bool isSameDireciton(const BPParaVec& other) const; //angle 0 rad
		__declspec(dllexport) bool isParallel(const BPParaVec& other) const; //angle 0 or pi
		__declspec(dllexport) bool isPerpendi(const BPParaVec& other) const; //angle pi/2 

#ifdef PARA2P3D
	public:
		inline BPParaVec(const p3d::GeVec3d& rhs) :m_imp({ rhs.x, rhs.y, rhs.z })
		{
		}
		inline operator p3d::GeVec3d() const
		{
			return p3d::GeVec3d::create(m_imp.x, m_imp.y, m_imp.z);
		}
		inline BPParaVec(const p3d::GePoint3d& rhs) :m_imp({ rhs.x, rhs.y, rhs.z })
		{
		}
		inline operator p3d::GePoint3d() const
		{
			return p3d::GePoint3d::create(m_imp.x, m_imp.y, m_imp.z);
		}
		inline BPParaVec(const p3d::GeVec2d& rhs) :m_imp({ rhs.x, rhs.y, 0.0 })
		{
		}
		inline operator p3d::GeVec2d() const
		{
			return p3d::GeVec2d::create(m_imp.x, m_imp.y);
		}
		inline BPParaVec(const p3d::GePoint2d& rhs) :m_imp({ rhs.x, rhs.y, 0.0 })
		{
		}
		inline operator p3d::GePoint2d() const
		{
			return p3d::GePoint2d::create(m_imp.x, m_imp.y);
		}
		//__declspec(dllexport) operator GeoPoint() const;

#endif // PARA2P3D

	};

	//global static variable
	static const BPParaVec g_axisO = BPParaVec(0.0, 0.0, 0.0);
	static const BPParaVec g_axisX = BPParaVec(1.0, 0.0, 0.0);
	static const BPParaVec g_axisY = BPParaVec(0.0, 1.0, 0.0);
	static const BPParaVec g_axisZ = BPParaVec(0.0, 0.0, 1.0);
	static const BPParaVec g_axisNaN = BPParaVec(std::nan("0"), std::nan("0"), std::nan("0")); //in <cmath>

	//inline
	inline bool isFloatZero(double num) { return _isFloatZero(num, PL_Length); }
	inline bool isFloatZero(double num, double eps) { return _isFloatZero(num, eps); }
	inline bool isFloatEqual(double numA, double numB) { return _isFloatEqual(numA, numB, 0.0); }
	inline bool isFloatEqualOrLess(double numA, double numB) { return _isFloatEqual(numA, numB, 0.0) || numA < numB; }
	inline bool isFloatEqualOrMore(double numA, double numB) { return _isFloatEqual(numA, numB, 0.0) || numA > numB; }
	inline bool isFloatEqual(double numA, double numB, double eps) { return _isFloatEqual(numA, numB, eps); }
	inline double norm(const BPParaVec& other) { return other.norm(); }
	inline BPParaVec unitize(const BPParaVec& other) { return BPParaVec(other).unitize(); }
	//inline BPParaVec toVec2(const BPParaVec& vec3) { return BPParaVec(vec3.m_imp.x, vec3.m_imp.y, 0); }
	inline BPParaVec toVec2XoY(const BPParaVec& vec3) { return BPParaVec(vec3.m_imp.x, vec3.m_imp.y, 0); }
	inline BPParaVec toVec2XoZ(const BPParaVec& vec3) { return BPParaVec(vec3.m_imp.x, 0, vec3.m_imp.z); }
	inline BPParaVec toVec2YoZ(const BPParaVec& vec3) { return BPParaVec(0, vec3.m_imp.y, vec3.m_imp.y); }
	inline BPParaVec operator*(double n, const BPParaVec& other) { return BPParaVec(n * other.m_imp.x, n * other.m_imp.y, n * other.m_imp.z); }
	inline BPParaVec2 _toVec2(const BPParaVec& vec) { return BPParaVec2(vec.x(), vec.y()); }

	//geometry relation
	__declspec(dllexport) bool isParallel(const BPParaVec& vecA, const BPParaVec& vecB);
	__declspec(dllexport) bool isPerpendi(const BPParaVec& vecA, const BPParaVec& vecB);
	__declspec(dllexport) bool isCoincident(const BPParaVec& vecA, const BPParaVec& vecB);
	__declspec(dllexport) std::vector<BPParaVec> operator*(const BPParaTransform& mat, const std::vector<BPParaVec>& vecs);
	__declspec(dllexport) double getAngleOfTwoVectors(const BPParaVec& vecA, const BPParaVec& vecB, bool isAbs = false);
	__declspec(dllexport) bool _getGnrcDouble(const Gnrc& gnrc, double& outNum);
	__declspec(dllexport) int mathSign(double x);

}