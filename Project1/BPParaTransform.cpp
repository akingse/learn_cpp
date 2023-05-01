#include "pch.h"
#define _USE_MATH_DEFINES
#include <math.h>
//--------------------------------------------------------------------------------------------------------------
//                                             cons
//--------------------------------------------------------------------------------------------------------------
bool BPParaTransform::isValid() const
{
	if (isZeroMatrix(*this, false))
		return false;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			if (isnan(m_matrix[i][j]) || isinf(m_matrix[i][j]))
			{
				return false;
			}
		}
	}
	return true;
}

BPParaTransform::BPParaTransform()
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			if (i == j)
				m_matrix[i][j] = 1.0;
			else
				m_matrix[i][j] = 0.0;
		}
	}
};

BPParaTransform::BPParaTransform(double a)
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			m_matrix[i][j] = a;
		}
	}
};

BPParaTransform::~BPParaTransform() {};


BPParaTransform BPParaTransform::operator+(const BPParaTransform& other) const
{
	BPParaTransform c;
	for (int iC = 0; iC < 4; iC++)
	{
		for (int iR = 0; iR < 3; iR++)
		{
			c.m_matrix[iR][iC] = m_matrix[iR][iC] + other.m_matrix[iR][iC];
		}
	}
	return c;
}

BPParaTransform BPParaTransform::operator-(const BPParaTransform& other) const
{
	BPParaTransform c;
	for (int iC = 0; iC < 4; iC++)
	{
		for (int iR = 0; iR < 3; iR++)
		{
			c.m_matrix[iR][iC] = m_matrix[iR][iC] - other.m_matrix[iR][iC];
		}
	}
	return c;
}

BPParaTransform BPParaTransform::operator*(const BPParaTransform& other) const //
{
	BPParaTransform c;
	for (int iC = 0; iC < 4; iC++)
	{
		for (int iR = 0; iR < 3; iR++)
		{
			c.m_matrix[iR][iC] = 0.0;
			for (int iS = 0; iS < 3; iS++)
				c.m_matrix[iR][iC] += m_matrix[iR][iS] * other.m_matrix[iS][iC];
			if (iC == 3)
				c.m_matrix[iR][iC] += m_matrix[iR][iC];
		}
	}
	return c;
}
BPParaVec BPParaTransform::operator*(const BPParaVec& other) const
{
	CVec3 c;
	CVec3 o = other.m_imp;
	c.x = m_matrix[0][0] * o.x + m_matrix[0][1] * o.y + m_matrix[0][2] * o.z + m_matrix[0][3];
	c.y = m_matrix[1][0] * o.x + m_matrix[1][1] * o.y + m_matrix[1][2] * o.z + m_matrix[1][3];
	c.z = m_matrix[2][0] * o.x + m_matrix[2][1] * o.y + m_matrix[2][2] * o.z + m_matrix[2][3];
	return BPParaVec(c);
}
BPParaTransform BPParaTransform::operator*(const double& other) const // number right mutiply
{
	BPParaTransform c;
	for (int iC = 0; iC < 4; iC++)
	{
		for (int iR = 0; iR < 3; iR++)
		{
			c.m_matrix[iR][iC] = c.m_matrix[iR][iC] * other;
		}
	}
	return c;
}

bool BPParaTransform::operator<(const BPParaTransform& other) const
{
	//for (int iC = 0; iC < 4; iC++)
	//{
	//	for (int iR = 0; iR < 3; iR++)
	//	{
	//		if (m_matrix[iR][iC]<other.m_matrix[iR][iC])
	//			return true;
	//	}
	//}
	//return false;
	return memcmp(this, &other, sizeof(BPParaTransform)) == -1;
}
bool BPParaTransform::operator==(const BPParaTransform& other) const
{
	return isZeroMatrix(*this - other);
}

/*BPParaVec2 BPParaTransform::operator*(const BPParaVec2& other) const
{
	BPParaVec2 c;
	c.x = m_matrix[0][0] * other.x + m_matrix[0][1] * other.y + m_matrix[0][3];
	c.y = m_matrix[1][0] * other.x + m_matrix[1][1] * other.y + m_matrix[1][3];
	return c;
}*/
//BPParaTransform BPParaTransform::inverse() const
//{
//	return this->inverse();
//}
BPParaTransform BPParaTransform::inverse() const
{
	BPParaTransform invT;
	invT.m_matrix[0][3] = -m_matrix[0][3];
	invT.m_matrix[1][3] = -m_matrix[1][3];
	invT.m_matrix[2][3] = -m_matrix[2][3];
	double	a11 = m_matrix[0][0];
	double	a12 = m_matrix[0][1];
	double	a13 = m_matrix[0][2];
	double	a21 = m_matrix[1][0];
	double	a22 = m_matrix[1][1];
	double	a23 = m_matrix[1][2];
	double	a31 = m_matrix[2][0];
	double	a32 = m_matrix[2][1];
	double	a33 = m_matrix[2][2];
	double M_det = a11 * a22 * a33 + a12 * a23 * a31 + a13 * a32 * a21 - a13 * a22 * a31 - a12 * a21 * a33 - a11 * a32 * a23;
	if (abs(M_det) < PL_Surface)
	{
		std::string message = "M_det<0, there is no inverse matrix!\n";
		//				BPWorkToUIMessageEventListenerCenter::getInstance().notifyListeners(L"PBBim.Message.ToolPrompt", ::p3d::PString(message.c_str()).c_str());
		return g_MatrixO;
	}
	double b11 = a22 * a33 - a23 * a32;
	double b12 = a21 * a33 - a23 * a31;
	double b13 = a21 * a32 - a22 * a31;
	double b21 = a12 * a33 - a13 * a32;
	double b22 = a11 * a33 - a13 * a31;
	double b23 = a11 * a32 - a12 * a31;
	double b31 = a12 * a23 - a13 * a22;
	double b32 = a11 * a23 - a13 * a21;
	double b33 = a11 * a22 - a12 * a21;
	BPParaTransform M_adj = setMatrixByValueList(//transpose T
		b11, -b21, b31, 0,
		-b12, b22, -b32, 0,
		b13, -b23, b33, 0);
	return (1 / M_det) * M_adj * invT;
}
//BPParaTransform BPParaTransform::inverse_orth() const
//{
//	return this->inverse_orth();
//}
BPParaTransform BPParaTransform::inverseOrth() const
{
	BPParaTransform rotM = getMatrixsRotationPart(*this);
	BPParaVec transP = BPParaVec(m_matrix[0][3], m_matrix[1][3], m_matrix[2][3]);//getMatrixsPosition(T);
	return transpose(rotM) * translate((-1.0) * transP);
}

//--------------------------------------------------------------------------------------------------------------
//                                             common
//--------------------------------------------------------------------------------------------------------------

BPParaTransform scale(double n)
{
	BPParaTransform transform;
	transform.m_matrix[0][0] = n;
	transform.m_matrix[1][1] = n;
	transform.m_matrix[2][2] = n;
	return transform;
}
BPParaTransform scale(double x, double y, double z /*= 1.0*/)
{
	BPParaTransform transform;
	transform.m_matrix[0][0] = x;
	transform.m_matrix[1][1] = y;
	transform.m_matrix[2][2] = z;
	return transform;
}

BPParaTransform scale(const BPParaVec& n)
{
	BPParaTransform transform;
	transform.m_matrix[0][0] = n.m_imp.x;
	transform.m_matrix[1][1] = n.m_imp.y;
	transform.m_matrix[2][2] = n.m_imp.z;
	return transform;
}

BPParaTransform trans(double x, double y, double z /*= 0.0*/)
{
	return translate(x, y, z);
}
BPParaTransform trans(const BPParaVec& point)
{
	return translate(point);
}


BPParaTransform translate(double x, double y, double z /*= 0.0*/)
{
	BPParaTransform transform;
	transform.m_matrix[0][3] = x;
	transform.m_matrix[1][3] = y;
	transform.m_matrix[2][3] = z;
	return transform;
}
BPParaTransform translate(const BPParaVec& point)
{
	BPParaTransform transform;
	transform.m_matrix[0][3] = point.m_imp.x;
	transform.m_matrix[1][3] = point.m_imp.y;
	transform.m_matrix[2][3] = point.m_imp.z;
	return transform;
}
BPParaTransform transx(double x)
{
	return translate(x, 0, 0);
}
BPParaTransform transy(double y)
{
	return translate(0, y, 0);
}
BPParaTransform transz(double z)
{
	return translate(0, 0, z);
}

BPParaTransform rotate(double angle, const BPParaVec& axis /*= PPCVec3()*/)
{
	// Rodrigues' rotation formula
	CVec3 nv = unitize(axis).m_imp;
	double c = 1.0 - cos(angle);
	double s = sin(angle);
	BPParaTransform A = setMatrixByValueList(
		0.0, -nv.z * c, nv.y * c, 0.0,
		nv.z * c, 0.0, -nv.x * c, 0.0,
		-nv.y * c, nv.x * c, 0.0, 0.0);
	BPParaTransform B = setMatrixByValueList(
		0.0, -nv.z, nv.y, 0.0,
		nv.z, 0.0, -nv.x, 0.0,
		-nv.y, nv.x, 0.0, 0.0);
	BPParaTransform T = A * B;
	BPParaTransform R = setMatrixByValueList(
		T.m_matrix[0][0] + 1.0, T.m_matrix[0][1] - nv.z * s, T.m_matrix[0][2] + nv.y * s, 0.0,
		T.m_matrix[1][0] + nv.z * s, T.m_matrix[1][1] + 1.0, T.m_matrix[1][2] - nv.x * s, 0.0,
		T.m_matrix[2][0] - nv.y * s, T.m_matrix[2][1] + nv.x * s, T.m_matrix[2][2] + 1.0, 0.0);
	return R;
}

BPParaTransform rotate(const BPParaVec& axis, double angle) //compat
{
	return rotate(angle, axis);
}

BPParaTransform rotateArbitrary(const BPParaVec& point, const BPParaVec& vector, double theta)
{
	return translate(point) * rotate(theta, vector) * translate((-1.0) * point);
}

BPParaTransform rotx(double theta) //基础旋转矩阵
{
	BPParaTransform mat = setMatrixByValueList(
		1, 0, 0, 0,
		0, cos(theta), -sin(theta), 0,
		0, sin(theta), cos(theta), 0);
	return mat;
}

BPParaTransform roty(double theta)
{
	BPParaTransform mat = setMatrixByValueList(
		cos(theta), 0, sin(theta), 0,
		0, 1, 0, 0,
		-sin(theta), 0, cos(theta), 0);
	return mat;
}

BPParaTransform rotz(double theta)
{
	BPParaTransform mat = setMatrixByValueList(
		cos(theta), -sin(theta), 0, 0,
		sin(theta), cos(theta), 0, 0,
		0, 0, 1, 0);
	return mat;
}
BPParaTransform getMatrixByEulerRPY(const BPParaVec& rpy)
{
	double r = rpy.m_imp.x;
	double p = rpy.m_imp.y;
	double y = rpy.m_imp.z;
	double y_cos = cos(y), y_sin = sin(y);
	double p_cos = cos(p), p_sin = sin(p);
	double r_cos = cos(r), r_sin = sin(r);
	return setMatrixByValueList(
		y_cos * p_cos, -(y_sin * r_cos + y_cos * p_sin * r_sin), (y_sin * r_sin - y_cos * p_sin * r_cos), 0,
		y_sin * p_cos, (y_cos * r_cos - y_sin * p_sin * r_sin), -(y_cos * r_sin + y_sin * p_sin * r_cos), 0,
		p_sin, p_cos * r_sin, p_cos * r_cos, 0
	);
}
BPParaVec getRPYByMatrix(const BPParaTransform& mat)
{
	BPParaVec angles;
	double s1 = mat.m_matrix[2][0];
	double c1 = sqrt(mat.m_matrix[2][1] * mat.m_matrix[2][1] + mat.m_matrix[2][2] * mat.m_matrix[2][2]);

	double pitchA = atan2(s1, c1);
	double pitchB = atan2(s1, -c1);
	if (c1 < PL_Length)
	{
		angles = BPParaVec(atan2(-mat.m_matrix[0][1], mat.m_matrix[1][1]), pitchA, 0);
		return angles;
	}
	else
	{
		double yawA = atan2(mat.m_matrix[1][0], mat.m_matrix[0][0]);
		double rollA = atan2(mat.m_matrix[2][1], mat.m_matrix[2][2]);
		double yawB = atan2(-mat.m_matrix[1][0], -mat.m_matrix[0][0]);
		double rollB = atan2(-mat.m_matrix[2][1], -mat.m_matrix[2][2]);

		BPParaVec yprA(rollA, pitchA, yawA);
		BPParaVec yprB(rollB, pitchB, yawB);
		static double s_absFactor = 0.95;
		//maxAbsDegrees
		double radiansA = max(max(yprA.m_imp.x, yprA.m_imp.y), yprA.m_imp.z);
		double radiansB = max(max(yprB.m_imp.x, yprB.m_imp.y), yprB.m_imp.z);
		if (radiansA < s_absFactor * radiansB)
			angles = yprA;
		else if (radiansB < s_absFactor * radiansA)
			angles = yprB;
		else
		{
			//SumSquaredDegrees
			double sumA = yprA.m_imp.x * yprA.m_imp.x + yprA.m_imp.y * yprA.m_imp.y + yprA.m_imp.z * yprA.m_imp.z;
			double sumB = yprB.m_imp.x * yprB.m_imp.x + yprB.m_imp.y * yprB.m_imp.y + yprB.m_imp.z * yprB.m_imp.z;
			if (sumA <= sumB)
				angles = yprA;
			else
				angles = yprB;
		}
		return angles;
	}
}

BPParaTransform mirror(const COORD_SYS& sys) // 关于坐标系轴的镜像
{
	switch (sys)
	{
	case COORD_SYS::AXIS_O:
	{
		BPParaTransform mat = setMatrixByValueList(
			-1, 0, 0, 0,
			0, -1, 0, 0,
			0, 0, -1, 0);
		return mat;
		break;
	}
	case COORD_SYS::AXIS_X:
	{
		BPParaTransform mat = setMatrixByValueList(
			1, 0, 0, 0,
			0, -1, 0, 0,
			0, 0, -1, 0);
		return mat;
		break;
	}
	case COORD_SYS::AXIS_Y:
	{
		BPParaTransform mat = setMatrixByValueList(
			-1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, -1, 0);
		return mat;
		break;
	}
	case COORD_SYS::AXIS_Z:
	{
		BPParaTransform mat = setMatrixByValueList(
			-1, 0, 0, 0,
			0, -1, 0, 0,
			0, 0, 1, 0);
		return mat;
		break;

	}
	case COORD_SYS::PLANE_XOY:
	{
		BPParaTransform mat = setMatrixByValueList(
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, -1, 0);
		return mat;
		break;

	}
	case COORD_SYS::PLANE_XOZ:
	{
		BPParaTransform mat = setMatrixByValueList(
			1, 0, 0, 0,
			0, -1, 0, 0,
			0, 0, 1, 0);
		return mat;
		break;
	}
	case COORD_SYS::PLANE_YOZ:
	{
		BPParaTransform mat = setMatrixByValueList(
			-1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0);
		return mat;
		break;

	}
	default:
	{
		return g_MatrixO;
	}
	}
}

//BPParaTransform mirror(const BPParaVec & axisA, const BPParaVec & axisB)
//{
//	BPParaTransform forwM = setMatrixByTwoVectors(axisA, axisA);
//	BPParaTransform invM = inverse_orth(forwM);
//	BPParaTransform mirXoY(p3d::GeTransform::createByValue(
//		1, 0, 0, 0,
//		0, 1, 0, 0,
//		0, 0, -1, 0
//	));
//	return forwM * mirXoY * invM;
//}

BPParaTransform mirror(const BPParaTransform& mat, const COORD_SYS& sys) //关于面的镜像;;
{
	BPParaTransform mirXOY = setMatrixByValueList(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, 0);
	BPParaTransform mirXOZ = setMatrixByValueList(1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0);
	BPParaTransform	mirYOZ = setMatrixByValueList(-1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
	switch (sys)
	{
	case COORD_SYS::PLANE_XOY:
	{
		return mat * mirXOY * inverse(mat);
		break;
	}
	case COORD_SYS::PLANE_XOZ:
	{
		return mat * mirXOZ * inverse(mat);
		break;
	}
	case COORD_SYS::PLANE_YOZ:
	{
		return mat * mirYOZ * inverse(mat);
		break;
	}
	default:
	{
		return g_MatrixO;
		break;
	}
	};
}

BPParaTransform shear(const COORD_SYS& axis, double a, double b) //指定单位基准坐标轴的偏移距离
{

	switch (axis)
	{
	case COORD_SYS::AXIS_X:
	{
		BPParaTransform mat = setMatrixByValueList(
			1, 0, 0, 0,
			a, 1, 0, 0,
			b, 0, 1, 0);
		return mat;
		break;
	}
	case COORD_SYS::AXIS_Y:
	{

		BPParaTransform mat = setMatrixByValueList(
			1, a, 0, 0,
			0, 1, 0, 0,
			0, b, 1, 0);
		return mat;
		break;
	}
	case COORD_SYS::AXIS_Z:
	{
		BPParaTransform mat = setMatrixByValueList(
			1, 0, a, 0,
			0, 1, b, 0,
			0, 0, 1, 0);
		return mat;
		break;
	}
	default:
	{
		return g_MatrixO;
		break;
	}
	}
}
//--------------------------------------------------------------------------------------------------------------
//                                             attribute
//--------------------------------------------------------------------------------------------------------------

BPParaTransform transpose(const BPParaTransform& T)
{
	BPParaTransform M = setMatrixByValueList(
		T.m_matrix[0][0], T.m_matrix[1][0], T.m_matrix[2][0], 0.0,
		T.m_matrix[0][1], T.m_matrix[1][1], T.m_matrix[2][1], 0.0,
		T.m_matrix[0][2], T.m_matrix[1][2], T.m_matrix[2][2], 0.0);
	return M;
}
BPParaTransform inverse(const BPParaTransform& T)
{
	return T.inverse();
}

BPParaTransform inverseOrth(const BPParaTransform& T)
{
	BPParaTransform rotM = getMatrixsRotationPart(T);
	BPParaVec transP = getMatrixsPosition(T);
	return transpose(rotM) * translate((-1.0) * transP);
}

BPParaTransform operator*(double a, const BPParaTransform& b) //数字左乘
{
	BPParaTransform c;
	for (int iC = 0; iC < 4; iC++)
	{
		for (int iR = 0; iR < 3; iR++)
		{
			c.m_matrix[iR][iC] = b.m_matrix[iR][iC] * a;
		}
	}
	return c;
}

//--------------------------------------------------------------------------------------------------------------
//                                             set&get
//--------------------------------------------------------------------------------------------------------------

BPParaTransform setMatrixByColumnVectors(const BPParaVec& n, const BPParaVec& o, const BPParaVec& a, const BPParaVec& p /*= BPParaVec()*/)
{
	BPParaTransform M = setMatrixByValueList(
		n.m_imp.x, o.m_imp.x, a.m_imp.x, p.m_imp.x,
		n.m_imp.y, o.m_imp.y, a.m_imp.y, p.m_imp.y,
		n.m_imp.z, o.m_imp.z, a.m_imp.z, p.m_imp.z);
	return M;
}
//BPParaTransform setMatrixByArray(double* mat[3][4])
//{
//	BPParaTransform matR;
//	matR.m_matrix = *mat;
//	return matR;
//}

//BPParaTransform setMatrixByRowVectors(const BPParaVec& n, const BPParaVec& o, const BPParaVec& a, BPParaVec p /*= BPParaVec()*/)
//{
//}
BPParaTransform setMatrixByTwoVectors(const BPParaVec& vecX, const BPParaVec& vecY, bool isOrth /*= true*/)
{
	if (norm(vecX) + norm(vecY) < PL_Length) //both zero vector
	{
		return BPParaTransform();
	}
	if (norm(vecX) * norm(vecY) < PL_Length || norm((vecX ^ vecY)) < PL_Length)//模长为零 || 方向相同
	{
		return getMatrixFromOneVector(vecX + vecY, false);
	}
	if (isOrth) // unit orthogonal
	{
		BPParaVec vectorX = unitize(vecX);
		BPParaVec vectorZ = unitize(vecX ^ vecY); // been collinear judge
		BPParaVec vectorY = unitize(vectorZ ^ vectorX);
		return setMatrixByColumnVectors(vectorX, vectorY, vectorZ);
	}
	else
	{
		BPParaVec vecZ = unitize(vecX ^ vecY);
		return setMatrixByColumnVectors(vecX, vecY, vecZ);
	}
}
BPParaTransform setMatrixByRotAndPosition(const BPParaTransform& rot, const BPParaVec& position)
{
	return trans(position) * rot;
}
BPParaTransform setMatrixByRotAndPosition(const BPParaTransform& rot, const BPParaTransform& position)
{
	return position * rot;
}

BPParaTransform setMatrixByValueList(
	double nx, double ox, double ax, double px,
	double ny, double oy, double ay, double py,
	double nz, double oz, double az, double pz) //
{
	BPParaTransform mat;
	mat.m_matrix[0][0] = nx;
	mat.m_matrix[0][1] = ox;
	mat.m_matrix[0][2] = ax;
	mat.m_matrix[0][3] = px;
	mat.m_matrix[1][0] = ny;
	mat.m_matrix[1][1] = oy;
	mat.m_matrix[1][2] = ay;
	mat.m_matrix[1][3] = py;
	mat.m_matrix[2][0] = nz;
	mat.m_matrix[2][1] = oz;
	mat.m_matrix[2][2] = az;
	mat.m_matrix[2][3] = pz;
	return mat;
}

BPParaTransform setMatrixByValueList(double vars[12], bool isRow /*= true*/) //
{
	BPParaTransform mat;
	if (isRow)
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				mat.m_matrix[i][j] = vars[j + 4 * i];
			}
		}
	}
	else
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				mat.m_matrix[j][i] = vars[j + 3 * i];
			}
		}
	}
	return mat;
}

//double* getListFromMatrix(const BPParaTransform& mat, bool isRow /*= true*/) //
//{
//	double alist[12];
//	if (isRow)
//	{
//		for (int i = 0; i < 3; i++)
//		{
//			for (int j = 0; j < 4; j++)
//				alist[j + 4 * i] = mat.m_matrix[i][j];
//		}
//	}
//	else
//	{
//		for (int i = 0; i < 4; i++)
//		{
//			for (int j = 0; j < 3; j++)
//				alist[j + 3 * i] = mat.m_matrix[j][i];
//		}
//	}
//	return alist;
//}

BPParaVec getMatrixsAxisX(const BPParaTransform& T)
{
	return BPParaVec(T.m_matrix[0][0], T.m_matrix[1][0], T.m_matrix[2][0]);
}

BPParaVec getMatrixsAxisY(const BPParaTransform& T)
{
	return BPParaVec(T.m_matrix[0][1], T.m_matrix[1][1], T.m_matrix[2][1]);
}

BPParaVec getMatrixsAxisZ(const BPParaTransform& T)
{
	return BPParaVec(T.m_matrix[0][2], T.m_matrix[1][2], T.m_matrix[2][2]);
}

BPParaVec getMatrixsPosition(const BPParaTransform& T)
{
	return BPParaVec(T.m_matrix[0][3], T.m_matrix[1][3], T.m_matrix[2][3]);
}

BPParaTransform getMatrixsRotationPart(const BPParaTransform& T)
{
	BPParaTransform M = setMatrixByValueList(
		T.m_matrix[0][0], T.m_matrix[0][1], T.m_matrix[0][2], 0.0,
		T.m_matrix[1][0], T.m_matrix[1][1], T.m_matrix[1][2], 0.0,
		T.m_matrix[2][0], T.m_matrix[2][1], T.m_matrix[2][2], 0.0);
	return M;
}
BPParaTransform getMatrixsPositionPart(const BPParaTransform& T)
{
	return trans(getMatrixsPosition(T));
}

BPParaVec getMatrixsScale(const BPParaTransform& T)
{
	return BPParaVec(getMatrixsAxisX(T).norm(), getMatrixsAxisY(T).norm(), getMatrixsAxisZ(T).norm());
}
BPParaTransform getMatrixsScalePart(const BPParaTransform& T)
{
	BPParaVec vec = getMatrixsScale(T);
	return scale(vec);
}

//--------------------------------------------------------------------------------------------------------------
//                                             judge
//--------------------------------------------------------------------------------------------------------------

bool isIdentifyMatrix(const BPParaTransform& M)
{
	BPParaVec posi = getMatrixsPosition(M);
	if (norm(posi) > PL_Length)
		return false;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (i == j && abs(M.m_matrix[i][j] - 1) > PL_Length)
				return false;
			if (i != j && abs(M.m_matrix[i][j]) > PL_Length)
				return false;
		}
	}
	return true;
}
bool isZeroMatrix(const BPParaTransform& M, bool isAll /*= true*/)
{
	for (int i = 0; i < 3; i++)
	{
		if (isAll)
		{
			for (int j = 0; j < 4; j++)
			{
				if (abs(M.m_matrix[i][j]) > PL_Length)
					return false;
			}
		}
		else
		{
			for (int j = 0; j < 3; j++)
			{
				if (abs(M.m_matrix[i][j]) > PL_Length)
					return false;
			}
		}
	}
	return true;
}
bool isOrthogonalMatrix(const BPParaTransform& T, bool onlyRot /*= true*/)
{
	//onlyRot = true, only judge rotation part
	if (!onlyRot)
	{
		BPParaVec posi = getMatrixsPosition(T);
		if (norm(posi) > PL_Length)
			return false;
	}
	BPParaTransform M = getMatrixsRotationPart(T);
	return isIdentifyMatrix(transpose(M) * M);
}
BPParaTransform getOrthogonalMatrix(const BPParaTransform& T, bool withPosition /*= true*/)
{
	BPParaVec axisX = getMatrixsAxisX(T);
	BPParaVec axisY = getMatrixsAxisY(T);
	BPParaVec point = getMatrixsPosition(T);
	BPParaVec axisZ = unitize(axisX ^ axisY);
	axisY = axisZ ^ axisX;
	BPParaTransform rotM = setMatrixByColumnVectors(unitize(axisX), unitize(axisY), axisZ);
	return (withPosition) ? translate(point) * rotM : rotM;
}
bool isTwoDimensionalMatrix(const BPParaTransform& M)
{
	BPParaVec vecx = getMatrixsAxisX(M);
	BPParaVec vecy = getMatrixsAxisY(M);
	BPParaVec pos = getMatrixsPosition(M);
	if (abs(pos.m_imp.z) > PL_Length)
		return false;
	if (abs((vecx * g_axisZ)) > PL_Surface || abs((vecy * g_axisZ)) > PL_Surface)
		return false;
	return true;
}
//--------------------------------------------------------------------------------------------------------------
//                                             shadaow
//--------------------------------------------------------------------------------------------------------------

BPParaTransform shadowVectorMatrix2D(const BPParaVec& n)
{
	BPParaTransform M = setMatrixByValueList(
		1 - n.m_imp.x * n.m_imp.x, -n.m_imp.x * n.m_imp.y, -n.m_imp.x * n.m_imp.z, 0,
		-n.m_imp.x * n.m_imp.y, 1 - n.m_imp.y * n.m_imp.y, -n.m_imp.y * n.m_imp.z, 0,
		-n.m_imp.x * n.m_imp.z, -n.m_imp.y * n.m_imp.z, 1 - n.m_imp.z * n.m_imp.z, 0);
	return M;
}

BPParaTransform shadowScaleMatrix(double intAngle /*= 0*/) // enlarge surround z 
{
	if (isFloatEqual(intAngle,M_PI_2))
		return g_MatrixO;
	return scale(1 / cos(intAngle), 1 / cos(intAngle), 1) * rotz(intAngle);
}


//--------------------------------------------------------------------------------------------------------------
//                                             get-local
//--------------------------------------------------------------------------------------------------------------

BPParaTransform getMatrixFromTwoPoints(const BPParaVec& pointStart, const BPParaVec& pointEnd, bool isAxisZ /*= true*/)
{
	if (isCoincident(pointEnd, pointStart))
		return translate(pointStart);
	//default the two points vector is axisZ, or tobe axisX
	BPParaVec axisZ = unitize(pointEnd - pointStart);
	BPParaVec axisX;
	if (isParallel(axisZ, g_axisZ))
		axisX = g_axisX;
	else if (abs(axisZ.m_imp.z) < PL_Length) //axisZ on XoY plane
		axisX = g_axisZ;
	else
		axisX = BPParaVec(axisZ.m_imp.x, axisZ.m_imp.y, 0);
	BPParaVec axisY = unitize((axisZ ^ axisX));
	axisX = unitize((axisY ^ axisZ));
	if (!isAxisZ)
	{
		BPParaVec temp = axisZ;
		axisZ = axisX;
		axisX = temp;
		axisY = (-1.0) * axisY;
	}
	return setMatrixByColumnVectors(axisX, axisY, axisZ, pointStart);
}

BPParaTransform getMatrixFromTwoPoints(const BPParaVec& pointStart, const BPParaVec& pointEnd, bool withScale, bool isAxisZ)
{
	if (isCoincident(pointEnd, pointStart))
		return translate(pointStart);
	//default the two points vector is axisZ, or tobe axisX
	BPParaVec axisZ = (withScale) ? (pointEnd - pointStart) : unitize(pointEnd - pointStart);
	BPParaVec axisX;
	if (isParallel(axisZ, g_axisZ))
		axisX = g_axisX;
	else if (abs(axisZ.m_imp.z) < PL_Length) //axisZ on XoY plane
		axisX = g_axisZ;
	else
		axisX = BPParaVec(axisZ.m_imp.x, axisZ.m_imp.y, 0);
	BPParaVec axisY = unitize((axisZ ^ axisX));
	axisX = unitize((axisY ^ axisZ));
	if (!isAxisZ)
	{
		BPParaVec temp = axisZ;
		axisZ = axisX;
		axisX = temp;
		axisY = (-1.0) * axisY;
	}
	return setMatrixByColumnVectors(axisX, axisY, axisZ, pointStart);
}

BPParaTransform getMatrixFromOneVector(const BPParaVec& vector, bool isAxisZ/*=true*/) //one vector
{
	return getMatrixFromTwoPoints(g_axisO, vector, false);
}

BPParaTransform getMatrixFromTwoVector(const BPParaVec& vecA, const BPParaVec& vecB) //two vector
{
	return setMatrixByTwoVectors(vecA, vecB, true);
}

BPParaTransform getMatrixFromThreePoints(const std::vector<BPParaVec>& points, bool is2D /*= false*/)
{
	if (points.size() == 0)
		return BPParaTransform();
	else if (points.size() == 1)
		return translate(points[1]);
	else if (points.size() == 2)
		return getMatrixFromTwoPoints(points[0], points[1]);
	else
	{
		BPParaVec vecA = points[1] - points[0];
		BPParaVec vecB = points[2] - points[1];
		return translate(points[0]) * setMatrixByTwoVectors(vecA, vecB);
	}
}

BPParaTransform getMatrixFromThreePoints(const BPParaVec& point1, const BPParaVec& point2, const BPParaVec& point3)
{
	std::vector<BPParaVec> points = { point1 , point2 , point3 };
	return getMatrixFromThreePoints(points);
}

BPParaTransform getMatrixFromPoints(const std::vector<BPParaVec>& points, bool is2D /*= true*/)
{
	if (points.size() == 0)
		return BPParaTransform();
	BPParaVec vecA = BPParaVec();
	BPParaVec vecB = BPParaVec();
	int indexA = 0;
	for (int i = 1; i < points.size(); i++)
	{
		if (norm(points[i] - points[0]) > PL_Length)
		{
			vecA = points[i] - points[0];
			indexA = i;
			break;
		}
	}
	for (int j = indexA + 1; j < points.size(); j++)
	{
		if (norm(points[j] - points[indexA]) > PL_Length && norm((vecA ^ points[j] - points[indexA])) > PL_Length)
		{
			vecB = points[j] - points[indexA];
			break;
		}
	}
	BPParaTransform forwM = translate(points[0]) * setMatrixByTwoVectors(vecA, vecB);
	return (isTwoDimensionalMatrix(forwM) && is2D) ? BPParaTransform() : forwM;
}

//--------------------------------------------------------------------------------------------------------------
//                                             Invariance
//--------------------------------------------------------------------------------------------------------------

bool hasMatrixTranslate(const BPParaTransform& M)
{
	BPParaVec pos = getMatrixsPosition(M);
	return !pos.isOrigin();
}
bool hasMatrixRotate(const BPParaTransform& M)
{
	BPParaVec axisx = getMatrixsAxisX(M);
	BPParaVec axisy = getMatrixsAxisY(M);
	BPParaVec axisz = getMatrixsAxisZ(M);
	// strict judge, any axis shear is regard as rotate, iow any not-orth
	return !(axisx.isSameDireciton(g_axisX) && axisy.isSameDireciton(g_axisY) && axisz.isSameDireciton(g_axisZ));
}
bool hasMatrixScale(const BPParaTransform& M)
{
	BPParaVec axisx = getMatrixsAxisX(M);
	BPParaVec axisy = getMatrixsAxisY(M);
	BPParaVec axisz = getMatrixsAxisZ(M);
	return !(axisx.isUnitize() && axisy.isUnitize() && axisz.isUnitize());
}
bool hasMatrixScaleNotEqual(const BPParaTransform& M)	//neS
{
	if (hasMatrixShear(M))// non support shear
		return true;
	BPParaVec axisx = getMatrixsAxisX(M);
	BPParaVec axisy = getMatrixsAxisY(M);
	BPParaVec axisz = getMatrixsAxisZ(M);
	return !(isFloatEqual(axisx.norm(), axisy.norm()) && isFloatEqual(axisy.norm(), axisz.norm()));
}

bool hasMatrixMirror(const BPParaTransform& M)
{
	BPParaVec axisx = getMatrixsAxisX(M);
	BPParaVec axisy = getMatrixsAxisY(M);
	BPParaVec axisz = getMatrixsAxisZ(M);
	// support shear, on the basis of the dierction range of coord-axis
	return !(axisx.isAcuteAngle(axisy ^ axisz) && axisy.isAcuteAngle(axisz ^ axisx) && axisz.isAcuteAngle(axisx ^ axisy));
}
bool hasMatrixShear(const BPParaTransform& M)
{
	BPParaVec axisx = getMatrixsAxisX(M);
	BPParaVec axisy = getMatrixsAxisY(M);
	BPParaVec axisz = getMatrixsAxisZ(M);
	return !(isParallel(axisx, axisy ^ axisz) && isParallel(axisy, axisz ^ axisx) && isParallel(axisz, axisx ^ axisy));
}
bool hasMatrixTransform(const BPParaTransform& M, const MAT_TRANS_BAN& type)
{
	if (type == MAT_TRANS_BAN::Translate)
		return hasMatrixTranslate(M);
	else if (type == MAT_TRANS_BAN::Rotate)
		return hasMatrixRotate(M);
	else if (type == MAT_TRANS_BAN::Scale)
		return hasMatrixScale(M);
	else if (type == MAT_TRANS_BAN::ScaleNotEqual)
		return hasMatrixScaleNotEqual(M);
	else if (type == MAT_TRANS_BAN::Mirror)
		return hasMatrixMirror(M);
	else if (type == MAT_TRANS_BAN::Shear)
		return hasMatrixShear(M);
	else
		throw std::out_of_range("enum value error!");
}
std::vector<bool> hasMatrixTransform(const BPParaTransform& M)
{
	bool b0 = hasMatrixTranslate(M);
	bool b1 = hasMatrixRotate(M);
	bool b2 = hasMatrixScale(M);
	bool b3 = hasMatrixScaleNotEqual(M);
	bool b4 = hasMatrixMirror(M);
	bool b5 = hasMatrixShear(M);
	//return int(b0) + 2 * int(b1) + 4 * int(b2) + 8 * int(b3) + 16 * int(b4);
	return std::vector<bool>{b0, b1, b2, b3, b4, b5};
}

//bool getMatrixInvariance(const BPParaTransform& M, const BPParaInvariance& type)
//{
//	bool bT = hasMatrixTranslate(M);
//	bool bR = hasMatrixRotate(M);
//	bool bS = hasMatrixScale(M);
//	bool bSNE = hasMatrixScaleNotEqual(M);
//	bool bM = hasMatrixMirror(M);
//	bool bH = hasMatrixShear(M);
//	size_t PositionInvariance = (1ull << 0) * (isIdentifyMatrix(M));
//	size_t OrientationInvariance = (1ull << 1) * (!bR & !bM & !bH);
//	size_t ScaleInvariance = (1ull << 2) * (!bS & !bH);
//	size_t ChiralInvariance = (1ull << 3) * (!bM);
//	size_t AngularInvariance = (1ull << 4) * (!bH);
//	return (PositionInvariance | OrientationInvariance | ScaleInvariance | ChiralInvariance | AngularInvariance) & type;
//}
// special
// cone   => no scale shear 
// sphere => no nescale shear 

