#pragma once

enum class COORD_SYS :int
{
	AXIS_O = 0,
	AXIS_X = 1,
	AXIS_Y = 2,
	AXIS_Z = 3,
	PLANE_XOY = 4,
	PLANE_XOZ = 5,
	PLANE_YOZ = 6,
};
enum class MAT_TRANS_BAN :int
{
	Translate = 1,
	Rotate = 2,
	Scale = 3,
	ScaleNotEqual = 4,
	Mirror = 5,
	Shear = 6
};
enum class MAT_INVARIANCE :int
{
	None = 0,			//any matrix
	Position = 1,		// local  invariance: no T R S M H
	Orientation = 2,	// direct invariance: no R M H
	Scale = 3,			// length invariance: no S H
	Chiral = 4,			// hand   invariance: no M
	Orthogonal = 5,		// orthog invariance: no H
};


class BPParaTransform
{
public:
	double m_matrix[3][4];
	__declspec(dllexport) BPParaTransform();
	__declspec(dllexport) ~BPParaTransform();
	__declspec(dllexport) BPParaTransform inverse() const;
	__declspec(dllexport) BPParaTransform operator*(const BPParaTransform& other) const;
	__declspec(dllexport) bool isValid() const; //isValidMatrix, except allZero NaN Inf
	__declspec(dllexport) explicit BPParaTransform(double a);
	__declspec(dllexport) BPParaTransform inverseOrth() const;
	__declspec(dllexport) BPParaTransform operator+(const BPParaTransform& other) const;
	__declspec(dllexport) BPParaTransform operator-(const BPParaTransform& other) const;
	__declspec(dllexport) BPParaVec operator*(const BPParaVec& other) const;
	__declspec(dllexport) BPParaTransform operator*(const double& other) const;
	__declspec(dllexport) bool operator<(const BPParaTransform& other) const; //std::less<void>
	__declspec(dllexport) bool operator==(const BPParaTransform& other) const;//std::equal_to<void>

#ifdef PARA2P3D

	inline BPParaTransform(p3d::GeTransform _right)
	{
		memcpy(m_matrix, _right.array3d, sizeof(m_matrix));
	}
	inline BPParaTransform(p3d::GeRotMatrix _right)
	{
		for (int i = 0; i < 3; i++)
		{
			for (int ii = 0; ii < 3; ii++)
				m_matrix[i][ii] = _right.array3d[i][ii];
		}
		m_matrix[0][3] = 0.0;
		m_matrix[1][3] = 0.0;
		m_matrix[2][3] = 0.0;
	}
	inline operator p3d::GeTransform() const
	{
		p3d::GeTransform temp;
		memcpy(temp.array3d, m_matrix, sizeof(m_matrix));
		return temp;
	}
	inline operator p3d::GeRotMatrix() const
	{
		p3d::GeRotMatrix temp;
		temp.createByTransform(*this);
		return temp;
	}
#endif // PARA2P3D

};

// build-in matrix
static const BPParaTransform g_MatrixE = BPParaTransform();
static const BPParaTransform g_MatrixI = BPParaTransform();
static const BPParaTransform g_MatrixO = BPParaTransform(0.0);
//scale
__declspec(dllexport) BPParaTransform scale(double n);
__declspec(dllexport) BPParaTransform scale(double x, double y, double z = 1.0);
__declspec(dllexport) BPParaTransform scale(const BPParaVec& n);
inline BPParaTransform scalex(double x) { return scale(x, 1, 1); }
inline BPParaTransform scaley(double y) { return scale(1, y, 1); }
inline BPParaTransform scalez(double z) { return scale(1, 1, z); }
inline BPParaTransform scaleXoY(double s) { return scale(s, s, 1); }
// __declspec(dllexport) inline BPParaTransform scalePoint(double z);
//translate
__declspec(dllexport) BPParaTransform trans(double x, double y, double z = 0.0); //no implicit convert
__declspec(dllexport) BPParaTransform trans(const BPParaVec& point);
__declspec(dllexport) BPParaTransform transx(double x);
__declspec(dllexport) BPParaTransform transy(double y);
__declspec(dllexport) BPParaTransform transz(double z);
__declspec(dllexport) BPParaTransform translate(double x, double y, double z = 0.0);
__declspec(dllexport) BPParaTransform translate(const BPParaVec& point);
//rotate
__declspec(dllexport) BPParaTransform rotate(double angle, const BPParaVec& axis = BPParaVec(0, 0, 1));
__declspec(dllexport) BPParaTransform rotate(const BPParaVec& axis = BPParaVec(0, 0, 1), double angle = 0.0);
__declspec(dllexport) BPParaTransform rotateArbitrary(const BPParaVec& point = BPParaVec(0, 0, 0), const BPParaVec& vector = BPParaVec(0, 0, 1), double theta = 0.0);
__declspec(dllexport) BPParaTransform rotx(double theta);
__declspec(dllexport) BPParaTransform roty(double theta);
__declspec(dllexport) BPParaTransform rotz(double theta);
__declspec(dllexport) BPParaTransform getMatrixByEulerRPY(const BPParaVec& rpy);
__declspec(dllexport) BPParaVec getRPYByMatrix(const BPParaTransform& mat);
//liner matrix
// __declspec(dllexport) BPParaTransform mirror(const BPParaVec& axis);
// __declspec(dllexport) BPParaTransform mirror(const BPParaVec& axisA, const BPParaVec& axisB);
__declspec(dllexport) BPParaTransform mirror(const BPParaTransform& mat, const COORD_SYS& sys = COORD_SYS::PLANE_XOY/*const std::string& plane = "YOZ"*/); //关于面的镜像
inline BPParaTransform mirrorXoY(const BPParaTransform& mat = g_MatrixE) { return mirror(mat, COORD_SYS::PLANE_XOY); }
inline BPParaTransform mirrorXoZ(const BPParaTransform& mat = g_MatrixE) { return mirror(mat, COORD_SYS::PLANE_XOZ); }
inline BPParaTransform mirrorYoZ(const BPParaTransform& mat = g_MatrixE) { return mirror(mat, COORD_SYS::PLANE_YOZ); }
__declspec(dllexport) BPParaTransform shear(const COORD_SYS& axis, double a, double b);
inline BPParaTransform shearx(double y, double z) { return shear(COORD_SYS::AXIS_X, y, z); }
inline BPParaTransform sheary(double x, double z) { return shear(COORD_SYS::AXIS_Y, x, z); }
inline BPParaTransform shearz(double x, double y) { return shear(COORD_SYS::AXIS_Z, x, y); }
//matrix property
__declspec(dllexport) BPParaTransform transpose(const BPParaTransform& T);
__declspec(dllexport) BPParaTransform inverse(const BPParaTransform& T);
__declspec(dllexport) BPParaTransform inverseOrth(const BPParaTransform& T);
__declspec(dllexport) BPParaTransform operator*(double a, const BPParaTransform& b); //number*matrix
//operate-set
__declspec(dllexport) BPParaTransform setMatrixByColumnVectors(const BPParaVec& n, const BPParaVec& o, const BPParaVec& a, const BPParaVec& p = BPParaVec());
// __declspec(dllexport) BPParaTransform setMatrixByRowVectors(double row0[4], double row1[4], double row2[4]);
// __declspec(dllexport) BPParaTransform setMatrixByArray(double* mat[3][4]);
__declspec(dllexport) BPParaTransform setMatrixByRotAndPosition(const BPParaTransform& rot, const BPParaVec& position);
__declspec(dllexport) BPParaTransform setMatrixByRotAndPosition(const BPParaTransform& rot, const BPParaTransform& position);
__declspec(dllexport) BPParaTransform setMatrixByTwoVectors(const BPParaVec& vecX, const BPParaVec& vecY, bool isOrth = true); //
__declspec(dllexport) BPParaTransform setMatrixByValueList(double vars[12], bool isRow = true); //
__declspec(dllexport) BPParaTransform setMatrixByValueList(double nx, double ox, double ax, double px, double ny, double oy, double ay, double py, double nz, double oz, double az, double pz);
// __declspec(dllexport) double* getListFromMatrix(const BPParaTransform& mat, bool isRow = true); //
//operate-get
__declspec(dllexport) BPParaVec getMatrixsAxisX(const BPParaTransform& T);
__declspec(dllexport) BPParaVec getMatrixsAxisY(const BPParaTransform& T);
__declspec(dllexport) BPParaVec getMatrixsAxisZ(const BPParaTransform& T);
__declspec(dllexport) BPParaVec getMatrixsPosition(const BPParaTransform& T); // get position vector
__declspec(dllexport) BPParaTransform getMatrixsRotationPart(const BPParaTransform& T); // only rot, position is zero获
__declspec(dllexport) BPParaTransform getMatrixsPositionPart(const BPParaTransform& T); // only position matrix
__declspec(dllexport) BPParaVec getMatrixsScale(const BPParaTransform& T); // get scale vector
__declspec(dllexport) BPParaTransform getMatrixsScalePart(const BPParaTransform& T); // get scale matrix
//operate-judge
__declspec(dllexport) bool isIdentifyMatrix(const BPParaTransform& M); 
__declspec(dllexport) bool isZeroMatrix(const BPParaTransform& M, bool isAll = true); // all zero
__declspec(dllexport) bool isOrthogonalMatrix(const BPParaTransform& T, bool onlyRot = true); // 
__declspec(dllexport) BPParaTransform getOrthogonalMatrix(const BPParaTransform& T, bool withPosition = true);
// __declspec(dllexport) bool isAttitudeMatrix(const BPParaTransform& T); // 
__declspec(dllexport) bool isTwoDimensionalMatrix(const BPParaTransform& M);// only rotz&trans(x,y)
//matrix shadow
__declspec(dllexport) BPParaTransform shadowVectorMatrix2D(const BPParaVec& n); //arbitrary_shadow vector front shadow
// __declspec(dllexport) BPParaTransform shadowVectorMatrix3D(const BPParaVec& vec); // 3 dimensional shadow
__declspec(dllexport) BPParaTransform shadowScaleMatrix(double intAngle = 0); // enlarge shadow about axiaZ
//matrix genetate
__declspec(dllexport) BPParaTransform getMatrixFromTwoPoints(const BPParaVec& pointStart, const BPParaVec& pointEnd, bool isAxisZ = true);
__declspec(dllexport) BPParaTransform getMatrixFromTwoPoints(const BPParaVec& pointStart, const BPParaVec& pointEnd, bool withScale, bool isAxisZ);//overload
__declspec(dllexport) BPParaTransform getMatrixFromOneVector(const BPParaVec& vector, bool isAxisZ = true);
__declspec(dllexport) BPParaTransform getMatrixFromTwoVector(const BPParaVec& vecA, const BPParaVec& vecB);
__declspec(dllexport) BPParaTransform getMatrixFromThreePoints(const std::vector<BPParaVec>& points, bool is2D = false);
__declspec(dllexport) BPParaTransform getMatrixFromThreePoints(const BPParaVec& point1, const BPParaVec& point2, const BPParaVec& point3);
__declspec(dllexport) BPParaTransform getMatrixFromPoints(const std::vector<BPParaVec>& points, bool is2D = true);
//matrix invariance
__declspec(dllexport) bool hasMatrixTranslate(const BPParaTransform& M);		//T
__declspec(dllexport) bool hasMatrixRotate(const BPParaTransform& M);			//R
__declspec(dllexport) bool hasMatrixScale(const BPParaTransform& M);			//S
__declspec(dllexport) bool hasMatrixScaleNotEqual(const BPParaTransform& M);	//Sne
__declspec(dllexport) bool hasMatrixMirror(const BPParaTransform& M);			//M
__declspec(dllexport) bool hasMatrixShear(const BPParaTransform& M);			//H
__declspec(dllexport) bool hasMatrixTransform(const BPParaTransform& M, const MAT_TRANS_BAN& type);
__declspec(dllexport) std::vector<bool> hasMatrixTransform(const BPParaTransform& M); //combine transform
//__declspec(dllexport) bool getMatrixInvariance(const BPParaTransform& M, const BPParaInvariance& type);
