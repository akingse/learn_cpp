#include "pch.h"
//common operator
CVec3 CVec3add(CVec3 a, CVec3 b) //+
{
	CVec3 c;
	c.x = a.x + b.x;
	c.y = a.y + b.y;
	c.z = a.z + b.z;
	return c;
}

CVec3 CVec3sub(CVec3 a, CVec3 b) //-
{
	CVec3 c;
	c.x = a.x - b.x;
	c.y = a.y - b.y;
	c.z = a.z - b.z;
	return c;
}

double CVec3dot(CVec3 a, CVec3 b) //*
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

CVec3 CVec3cross(CVec3 a, CVec3 b) //^
{
	CVec3 c;
	c.x = a.y * b.z - a.z * b.y;
	c.y = a.z * b.x - a.x * b.z;
	c.z = a.x * b.y - a.y * b.x;
	return c;
}

bool CVec3equal(CVec3 a, CVec3 b) //=
{
	return CVec3norm(CVec3sub(a, b)) < PL_Length;
}

bool CVec3notequal(CVec3 a, CVec3 b) //=
{
	return !CVec3equal(a, b);
}

double CVec3norm(CVec3 a) //||
{
	return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}
bool CVec3less(CVec3 a, CVec3 b) //<
{
	return CVec3norm(a) < CVec3norm(b);
}
CVec3 CVec3unitize(CVec3 a) //v||
{
	double len = CVec3norm(a);
	if (len < PL_Length)
		return a;
	CVec3 c;
	c.x = a.x / len;
	c.y = a.y / len;
	c.z = a.z / len;
	return c;
}
//---------------------------------------------------------------------------------------------
//                                    function
//---------------------------------------------------------------------------------------------
bool _isFloatZero(double num, double eps /*= PL_A*/)
{
	if (!eps)
		return !bool(num);
	return abs(num) < eps;
}
bool _isFloatEqual(double numA, double numB, double eps /*= 0.0*/)
{
	if (!eps) //default
	{
		double maxl = abs(numA) > abs(numB) ? abs(numA) : abs(numB);
		return abs(numA - numB) < (maxl + 1) * PL_Length;
	}
	return abs(numA - numB) < eps;
}

bool isParallel3d(CVec3 vecA, CVec3 vecB)
{
	double maxl = (CVec3norm(vecA) >= CVec3norm(vecB)) ? CVec3norm(vecA) : CVec3norm(vecB);
	return (CVec3norm(CVec3cross(vecA, vecB)) < (maxl + 1) * PL_Length);
}

bool isPerpendi3d(CVec3 vecA, CVec3 vecB)
{
	double maxl = (CVec3norm(vecA) >= CVec3norm(vecB)) ? CVec3norm(vecA) : CVec3norm(vecB);
	return (abs(CVec3dot(vecA, vecB)) < (maxl + 1) * PL_Surface);
}

bool isCoincident3d(CVec3 vecA, CVec3 vecB)
{
	double maxl = (CVec3norm(vecA) >= CVec3norm(vecB)) ? CVec3norm(vecA) : CVec3norm(vecB);
	return (CVec3norm(CVec3sub(vecA, vecB)) < (maxl + 1) * PL_Length);
}

double getAngleOfTwoVectors3d(CVec3 vecA, CVec3 vecB, bool isAbs /*= false*/)
{
	if (CVec3norm(vecA) * CVec3norm(vecB) < PL_Angle) //exist zero vector
		return 0.0;
	double theta = acos(CVec3dot(vecA, vecB) / (CVec3norm(vecA) * CVec3norm(vecB))); //acos range 0->pi
	if (isAbs)
		return theta;
	if (CVec3cross(vecA, vecB).z < 0) //using in 2D
		theta = -theta;
	return theta;
}
//---------------------------------------------------------------------------------------------
//                                       BPParaVec
//---------------------------------------------------------------------------------------------
BPParaVec::BPParaVec() : m_imp({ 0, 0, 0 })
{
}

BPParaVec::BPParaVec(const CVec3& src) : m_imp(src)
{
}

BPParaVec::BPParaVec(double x, double y, double z) : m_imp({ x, y, z })
{
}

BPParaVec::~BPParaVec()
{
}

BPParaVec::BPParaVec(const BPParaVec& src) : m_imp(src.m_imp)
{
}

//BPParaVec::BPParaVec(BPParaVec&& src) : m_imp(src.m_imp)
//{
//}

BPParaVec& BPParaVec::operator=(const BPParaVec& src)
{
	m_imp = src.m_imp;
	return *this;
}

BPParaVec BPParaVec::operator^(const BPParaVec& other) const
{
	return BPParaVec(CVec3cross(m_imp, other.m_imp));
}
BPParaVec BPParaVec::operator^=(const BPParaVec& other)
{
	m_imp = CVec3cross(m_imp, other.m_imp);
	return BPParaVec(m_imp);
}
double BPParaVec::operator*(const BPParaVec& other) const
{
	return CVec3dot(m_imp, other.m_imp);
}
BPParaVec BPParaVec::operator-(const BPParaVec& other) const
{
	return BPParaVec(CVec3sub(m_imp, other.m_imp));
}
BPParaVec BPParaVec::operator-=(const BPParaVec& other)
{
	m_imp = CVec3sub(m_imp, other.m_imp);
	return BPParaVec(m_imp);
}
BPParaVec BPParaVec::operator+(const BPParaVec& other) const
{
	return BPParaVec(CVec3add(m_imp, other.m_imp));
}
BPParaVec BPParaVec::operator+=(const BPParaVec& other)
{
	m_imp = CVec3add(m_imp, other.m_imp);
	return BPParaVec(m_imp);
}
double BPParaVec::operator[](int i) const
{
	if (0 == i)
		return m_imp.x;
	else if (1 == i)
		return m_imp.y;
	else if (2 == i)
		return m_imp.z;
	else
		return std::nan("0");
		//throw std::range_error("out of index");
}
//compare
bool BPParaVec::operator<(const BPParaVec& other) const
{
	bool res = memcmp(this, &other, sizeof(CVec3)) == -1;
	return res;
	//return CVec3less(m_imp, other.m_imp);
}
bool BPParaVec::operator==(const BPParaVec& other) const
{
	return isFloatEqual(m_imp.x, other.m_imp.x) && isFloatEqual(m_imp.y, other.m_imp.y) && isFloatEqual(m_imp.z, other.m_imp.z);
	//return CVec3equal(m_imp, other.m_imp);
}

//bool isFloatZero(double num, double eps/* = PL_A*/)
//{
//	return _isFloatZero(num, eps);
//}

//bool isFloatEqual(double numA, double numB, double eps /*= 0.0*/)
//{
//	return _isFloatEqual(numA, numB, eps);
//}

double BPParaVec::norm() const
{
	return CVec3norm(m_imp);
}
BPParaVec& BPParaVec::unitize()
{
	m_imp = CVec3unitize(m_imp);
	return *this;
}
bool BPParaVec::isValid() const
{
	if (isnan(m_imp.x) || isnan(m_imp.y) || isnan(m_imp.z)) //not a number
		return false;
	if (isinf(m_imp.x) || isinf(m_imp.y) || isinf(m_imp.z)) //infinite
		return false;
	return true;
}
bool BPParaVec::isOrigin() const
{
	return norm() < PL_Length;
}
bool BPParaVec::isUnitize() const
{
	return abs(norm() - 1) < PL_Length;
}
bool BPParaVec::isAcuteAngle(const BPParaVec& other) const
{
	return *this * other > 0;
}
bool BPParaVec::isSameDireciton(const BPParaVec& other) const
{
	if (isOrigin() || other.isOrigin())
		return true;
	return (isParallel(other) && (*this * other) > 0);
}
bool BPParaVec::isParallel(const BPParaVec& other) const
{
	return ::isParallel(*this, other);
}
bool BPParaVec::isPerpendi(const BPParaVec& other) const
{
	return ::isPerpendi(*this, other);
}

bool isPerpendi(const BPParaVec& vecA, const BPParaVec& vecB)
{
	return isPerpendi3d(vecA.m_imp, vecB.m_imp);
}

bool isParallel(const BPParaVec& vecA, const BPParaVec& vecB)
{
	return isParallel3d(vecA.m_imp, vecB.m_imp);
}

bool isCoincident(const BPParaVec& vecA, const BPParaVec& vecB)
{
	return isCoincident3d(vecA.m_imp, vecB.m_imp);
}

double getAngleOfTwoVectors(const BPParaVec& vecA, const BPParaVec& vecB, bool isAbs /*= false*/)
{
	return getAngleOfTwoVectors3d(vecA.m_imp, vecB.m_imp, isAbs);
}

int mathSign(double x)
{
	if (abs(x) < PL_Length)
		return 0;
	return (x > 0) ? (1) : (-1);
}

std::vector<BPParaVec> operator*(const BPParaTransform& mat, const std::vector<BPParaVec>& vecs)
{
	std::vector<BPParaVec> retres;
	for (auto& iter : vecs)
		retres.push_back(mat * (iter));
	return retres;
}

bool _getGnrcDouble(const Gnrc& gnrc, double& outNum)
{
	if (gnrc.is<double>())
		outNum = gnrc.as<double>();
	else if (gnrc.is<int>())
		outNum = double(gnrc.as<int>());
	else if (gnrc.is<long long>())
		outNum = double(gnrc.as<long long>());
	else
		return false;
	return true;
}


