#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <type_traits>
#include <ctime>
#include <cassert>
#include <thread>
#include <functional>
#include <cmath>
#include <complex>
#include <vector>
#include <map>
#include <unordered_map>
#include <string>
#include <set>
#include <queue>
#include <stack>
#include <unordered_set>
#include <regex>
#include <cassert>
#include <typeinfo>
#include <typeindex>
#include <type_traits>
#include <utility>
#include <any>
#include <mutex>
#include <string_view>
#include <functional>
#include <memory_resource>

/* Warning, change the file to script automatically.
Please modify the source file under source. 
Source file: "Gnrc.h".*/

//custom template meta 
class None {};
template<typename T>
struct _GeneHasLessOperator
{
    template<typename U> static void test(...) {};
    template<typename U> static auto test(int)->decltype(std::declval<const U>() < std::declval<const U>()) {};
    enum { value = std::is_same<decltype(test<T>(0)), bool > ::value };
};

template<typename T>
struct _GeneHasEqualOperator
{
    template<typename U> static void test(...) {};
    template<typename U> static auto test(int)->decltype(std::declval<const U>() == std::declval<const T>()) {};
    enum { value = std::is_same<decltype(test<T>(0)), bool > ::value };
};

class _GeneBaseClass
{
public:
    //allow copy
    template<typename T, bool>
    class AbleCopy
    {
    public:
        //std::is_copy_assignable
        static void copy_assignable(T& lhs, const T& rhs);
    };
    template<typename T>
    class AbleCopy<T, true>
    {
    public:
        static void copy_assignable(T& lhs, const T& rhs)
        {
            lhs = rhs;
        }
    };
    template<typename T>
    class AbleCopy<T, false>
    {
    public:
        static void copy_assignable(T& lhs, const T& rhs)
        {
#ifdef _DEBUG
            throw std::logic_error("T nonsupport opetaror=");
#endif // DEBUG
        }
    };

    //allow compare less
    template<typename T, bool>
    class AbleLess
    {
    public:
        static bool operator_less(const T& lhs, const T& rhs);
    };
    template<typename T>
    class AbleLess<T, true>
    {
    public:
        static bool operator_less(const T& lhs, const T& rhs)
        {
            return lhs < rhs;
        }
    };
    template<typename T>
    class AbleLess<T, false>
    {
    public:
        static bool operator_less(const T& lhs, const T& rhs)
        {
#ifdef _DEBUG
            throw std::logic_error("T nonsupport opetaror<");
#else
            return false;
#endif // DEBUG
        }
    };

    //allow compare equal
    template<typename T, bool>
    class AbleEqual
    {
    public:
        static bool operator_equal(const T& lhs, const T& rhs);
    };
    template<typename T>
    class AbleEqual<T, true>
    {
    public:
        static bool operator_equal(const T& lhs, const T& rhs)
        {
            return lhs == rhs;
        }
    };
    template<typename T>
    class AbleEqual<T, false>
    {
    public:
        static bool operator_equal(const T& lhs, const T& rhs)
        {
#ifdef _DEBUG
            throw std::logic_error("T nonsupport opetaror==");
#else
            return false;
#endif // DEBUG
        }
    };

public:
    virtual ~_GeneBaseClass()
    {
    }
    virtual _GeneBaseClass* _constructor() const = 0; //make_constructor
    virtual bool _is(const std::type_index& type) const = 0;
    virtual bool _is_equal(const _GeneBaseClass& other) const = 0;
    virtual bool _is_less(const _GeneBaseClass& other) const = 0;
    virtual std::type_index _get_id() const = 0;
    virtual void* _get_imp() const = 0;
protected:
    virtual bool _copy_constructor(const _GeneBaseClass* src) = 0;
};

// case 1: value
template <typename T>
class _GeneTemplateClass :public _GeneBaseClass
{
public:
    T m_imp;
    _GeneTemplateClass(T src) : m_imp(src)
    {
    }
    ~_GeneTemplateClass()
    {
    }
    inline _GeneBaseClass* _constructor() const override
    {
        return new _GeneTemplateClass<T>(m_imp);
    }

protected:
    bool _copy_constructor(const _GeneBaseClass* src) override
    {
        if (!src->_is(typeid(T)))
            return false;
        const _GeneTemplateClass<T>* ptr = dynamic_cast<const _GeneTemplateClass<T>*>(src);
        if (nullptr == ptr)
            return false;
        //m_imp = ptr->m_imp;
        AbleCopy<T, std::is_copy_assignable<T>::value>::copy_assignable(m_imp, ptr->m_imp);
        return true;
    }
    inline bool _is(const std::type_index& type) const override
    {
        return type == typeid(T);
    }

    bool _is_equal(const _GeneBaseClass& other) const override
    {
        const _GeneTemplateClass<T>* ptr = dynamic_cast<const _GeneTemplateClass<T>*>(&other);
        if (!ptr) //(ptr == nullptr)
            return false;
        //return ptr->m_imp == this->m_imp;
        return AbleEqual<T, _GeneHasEqualOperator<T>::value>::operator_equal(ptr->m_imp, this->m_imp);
    }
    bool _is_less(const _GeneBaseClass& other) const override
    {
        if (!(&other))
            return false;
        const _GeneTemplateClass<T>* ptr = dynamic_cast<const _GeneTemplateClass<T>*>(&other);
        if (!ptr)
        {
            return typeid(*this).name() < typeid(other).name(); //compare string
            //return typeid(*this) < typeid(other); //compare hashcode
        }
        //return this->m_imp < ptr->m_imp;
        return AbleLess<T, _GeneHasLessOperator<T>::value>::operator_less(this->m_imp, ptr->m_imp);
    }

    std::type_index _get_id() const
    {
        return typeid(T);
    }

    virtual void* _get_imp() const override
    {
        return (void*)(&m_imp);
    }


};

// case 2: ref true, ref false
template <typename T>
class _GeneTemplateClassRef :public _GeneBaseClass
{
public:
    T* m_imp;
    bool m_ref;
    _GeneTemplateClassRef() :
        m_imp(nullptr),
        m_ref(false)
    {
    }
    _GeneTemplateClassRef(T* src, bool isRef = false) //: m_imp(src), m_ref(isRef)
    {
        m_ref = isRef;
        m_imp = new T(*src);
    }
    ~_GeneTemplateClassRef()
    {
        if (m_ref)
            return;
        else
        {
            delete m_imp;
            m_imp = nullptr;
        }
    }
    inline _GeneBaseClass* _constructor() const override
    {
        return new _GeneTemplateClassRef<T>(m_imp);
    }

protected:
    bool _copy_constructor(const _GeneBaseClass* src) override
    {
        if (!src->_is(typeid(T)))
            return false;
        const _GeneTemplateClassRef<T>* ptr = dynamic_cast<const _GeneTemplateClassRef<T>*>(src);
        if (nullptr == ptr)
            return false;
        //m_imp = ptr->m_imp;
        AbleCopy<T, std::is_copy_assignable<T>::value>::copy_assignable(*m_imp, *ptr->m_imp);
        return true;
    }
    inline bool _is(const std::type_index& type) const override
    {
        return type == typeid(T);
    }

    bool _is_equal(const _GeneBaseClass& other) const override
    {
        const _GeneTemplateClassRef<T>* ptr = dynamic_cast<const _GeneTemplateClassRef<T>*>(&other);
        if (!ptr) //(ptr == nullptr)
            return false;
        //return ptr->m_imp == this->m_imp;
        return AbleEqual<T, _GeneHasEqualOperator<T>::value>::operator_equal(*ptr->m_imp, *this->m_imp);
    }
    bool _is_less(const _GeneBaseClass& other) const override
    {
        if (!(&other))
            return false;
        const _GeneTemplateClassRef<T>* ptr = dynamic_cast<const _GeneTemplateClassRef<T>*>(&other);
        if (!ptr)
        {
            return typeid(*this).name() < typeid(other).name(); //compare string
            //return typeid(*this) < typeid(other); //compare hashcode
        }
        //return this->m_imp < ptr->m_imp;
        return AbleLess<T, _GeneHasLessOperator<T>::value>::operator_less(*this->m_imp, *ptr->m_imp);
    }

    std::type_index _get_id() const
    {
        return typeid(T);
    }
    virtual void* _get_imp() const override
    {
        return (void*)m_imp;
    }

};

class _GeneFactoryClassBase
{
public:
    virtual ~_GeneFactoryClassBase() {};
    virtual _GeneBaseClass* create(void* ptr, bool isRef) = 0;
};

template<typename T>
class _GeneFactoryClass :public _GeneFactoryClassBase
{
public:
    ~_GeneFactoryClass() {};
    virtual _GeneBaseClass* create(void* ptr, bool isRef)
    {
        _GeneTemplateClassRef<T>* tempRef = new _GeneTemplateClassRef<T>();
        if (ptr == nullptr)
        {
            tempRef->m_imp = new T;
        }
        else
        {
            tempRef->m_imp = static_cast<T*>(ptr);
            if (!(tempRef->m_imp))
            {
                throw std::runtime_error("static_cast T* fail");
            }
        }
        tempRef->m_ref = isRef;
        return tempRef;
    }
};

class Gnrc
{
    _GeneBaseClass* m_imp;
    static std::map<std::type_index, _GeneFactoryClassBase*> s_map;
    __declspec(dllimport) static std::map<std::type_index, _GeneFactoryClassBase*>& _getFactoryMap();
public:
    template<typename T>
    static void enrol()
    {
        if (_getFactoryMap().find(typeid(T)) == _getFactoryMap().end())//avoid mem-leak
        {
            _getFactoryMap()[typeid(T)] = new _GeneFactoryClass<T>;
        }
    }
    __declspec(dllimport) Gnrc();
    __declspec(dllimport) Gnrc(std::type_index id, void* src = nullptr, bool isRef = false);
    template <typename T> //
    Gnrc(const T& src) : m_imp(new _GeneTemplateClass<T>(src))
    {
        static bool _once = false;
        if (_once)
            return;
        _once = true;
        enrol<T>();
    }
    template <typename T> //
    Gnrc(T*) = delete;
    template <typename T> //
    Gnrc(const T*) = delete;
    __declspec(dllimport) Gnrc(const Gnrc& lhs); //
    __declspec(dllimport) Gnrc& operator=(const Gnrc& lhs) noexcept; //
    __declspec(dllimport) Gnrc(Gnrc&& rhs) noexcept;  //
    __declspec(dllimport) Gnrc& operator=(Gnrc&& src) noexcept; //
    __declspec(dllimport) Gnrc* operator&(); //
    __declspec(dllimport) const Gnrc* operator&() const; //
    //using namespace rel_ops;
    __declspec(dllimport) bool operator==(const Gnrc& other) const;
    __declspec(dllimport) bool operator<(const Gnrc& other) const;
    __declspec(dllimport) const std::type_index _id() const; //compat
    __declspec(dllimport) void* _imp() const;
    __declspec(dllimport) ~Gnrc();
    template <typename T>
    inline bool is() const
    {
        return m_imp->_is(typeid(T));
    }
    template <typename T>
    inline T& as()
    {
        return *(T*)(m_imp->_get_imp());
    }
    template <typename T>
    inline const T& as() const
    {
        return *(T*)(m_imp->_get_imp());
    }
    inline bool isValid() const
    {
        return _id() != typeid(None);
    }
};

using GnrcList = std::vector<Gnrc>;
using GnrcDict = std::map<Gnrc, Gnrc>;


/* Warning, change the file to script automatically.
Please modify the source file under source. 
Source file: "BPParaVec.h".*/
// using GB
static const double PL_Length = 1e-10;// AccuracyLinearDimensions::value();
static const double PL_Surface = 1e-6; //AccuracyLinearDimensions::square();
static const double PL_Angle = 1e-10; //AccuracyAngularDimensions::value();
extern "C"
{
	struct CVec3 /*__declspec(dllimport) */
	{
		double x;
		double y;
		double z;
	};// vec = { 0,0,0 };

	//member
	__declspec(dllimport) CVec3 CVec3add(CVec3 a, CVec3 b); //+
	__declspec(dllimport) CVec3 CVec3sub(CVec3 a, CVec3 b); //-
	__declspec(dllimport) double CVec3dot(CVec3 a, CVec3 b); //*
	__declspec(dllimport) CVec3 CVec3cross(CVec3 a, CVec3 b); //^
	__declspec(dllimport) bool CVec3equal(CVec3 a, CVec3 b); //=
	__declspec(dllimport) bool CVec3less(CVec3 a, CVec3 b); //< (compare norm)
	__declspec(dllimport) double CVec3norm(CVec3 a); //||
	__declspec(dllimport) CVec3 CVec3unitize(CVec3 a); //v||

	// float deviation
	__declspec(dllimport) bool _isFloatZero(double num, double eps = 1e-10);//default 0.0, using abs percision
	__declspec(dllimport) bool _isFloatEqual(double numA, double numB, double eps = 0.0); //default 0.0, using auto percision
	//__declspec(dllimport) bool isParallel2d(CVec2 vecA, CVec2 vecB);
	__declspec(dllimport) bool isParallel3d(CVec3 vecA, CVec3 vecB);
	//__declspec(dllimport) bool isPerpendi2d(CVec2 vecA, CVec2 vecB);
	__declspec(dllimport) bool isPerpendi3d(CVec3 vecA, CVec3 vecB);
	//__declspec(dllimport) bool isCoincident2d(CVec2 vecA, CVec2 vecB);
	__declspec(dllimport) bool isCoincident3d(CVec3 vecA, CVec3 vecB);
	//__declspec(dllimport) double getAngleOfTwoVectors2d(CVec2 vecA, CVec2 vecB, bool isAbs = false);
	__declspec(dllimport) double getAngleOfTwoVectors3d(CVec3 vecA, CVec3 vecB, bool isAbs = false);
}

class BPParaVec2
{
public:
	double x;
	double y;
	__declspec(dllimport) BPParaVec2() :x(0.0), y(0.0) {}
	__declspec(dllimport) BPParaVec2(double _x, double _y) : x(_x), y(_y) {}
	__declspec(dllimport) ~BPParaVec2() {}
	__declspec(dllimport) BPParaVec2(const BPParaVec2& rhs) : x(rhs.x), y(rhs.y) {}
	//__declspec(dllimport) BPParaVec2 operator^(const BPParaVec2& rhs) const;
	__declspec(dllimport) double operator*(const BPParaVec2& rhs) const { return x * rhs.x, y * rhs.y; }
	__declspec(dllimport) BPParaVec2 operator+(const BPParaVec2& rhs) const { return BPParaVec2(x + rhs.x, y + rhs.y); }
	__declspec(dllimport) BPParaVec2 operator-(const BPParaVec2& rhs) const { return BPParaVec2(x - rhs.x, y - rhs.y); }
	__declspec(dllimport) bool operator<(const BPParaVec2& rhs) const;
	__declspec(dllimport) bool operator==(const BPParaVec2& rhs) const;
	__declspec(dllimport) double operator[](int i) const;
	inline BPParaVec2 operator+() { return *this; }
	inline BPParaVec2 operator-() { return BPParaVec2(-x, -y); }
};

class BPParaTransform;
class BPParaVec
{
public:
	CVec3 m_imp;
	__declspec(dllimport) BPParaVec();
	__declspec(dllimport) BPParaVec(double x, double y /*= 0*/, double z = 0);
	__declspec(dllimport) ~BPParaVec();
		__declspec(dllimport) BPParaVec(const CVec3& src);
	__declspec(dllimport) BPParaVec(const BPParaVec& other);
	__declspec(dllimport) BPParaVec(BPParaVec&& other) = default;
	__declspec(dllimport) BPParaVec& operator=(const BPParaVec& other);
	__declspec(dllimport) BPParaVec operator^(const BPParaVec& other) const;
	__declspec(dllimport) BPParaVec operator^=(const BPParaVec& other);
	__declspec(dllimport) double operator*(const BPParaVec& other) const;
	__declspec(dllimport) BPParaVec operator+(const BPParaVec& other) const;
	__declspec(dllimport) BPParaVec operator+=(const BPParaVec& other);
	__declspec(dllimport) BPParaVec operator-(const BPParaVec& other) const;
	__declspec(dllimport) BPParaVec operator-=(const BPParaVec& other);
	__declspec(dllimport) bool operator<(const BPParaVec& other) const;
	__declspec(dllimport) bool operator==(const BPParaVec& other) const;
	__declspec(dllimport) double operator[](int i) const;
	inline BPParaVec operator+() { return *this; }
	inline BPParaVec operator-() { return BPParaVec(-m_imp.x, -m_imp.y, -m_imp.z); }
	inline double x() const { return m_imp.x; };
	inline double y() const { return m_imp.y; };
	inline double z() const { return m_imp.z; };
	__declspec(dllimport) double norm() const;
	__declspec(dllimport) BPParaVec& unitize();
	__declspec(dllimport) BPParaVec unitize() const { return BPParaVec(CVec3unitize(m_imp)); };
	__declspec(dllimport) bool isValid() const;
	__declspec(dllimport) bool isOrigin() const;
	__declspec(dllimport) bool isUnitize() const;
	__declspec(dllimport) bool isAcuteAngle(const BPParaVec& other) const; //exclusive right-angle
	__declspec(dllimport) bool isSameDireciton(const BPParaVec& other) const; //angle 0 rad
	__declspec(dllimport) bool isParallel(const BPParaVec& other) const; //angle 0 or pi
	__declspec(dllimport) bool isPerpendi(const BPParaVec& other) const; //angle pi/2 

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
	//__declspec(dllimport) operator GeoPoint() const;

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
__declspec(dllimport) bool isParallel(const BPParaVec& vecA, const BPParaVec& vecB);
__declspec(dllimport) bool isPerpendi(const BPParaVec& vecA, const BPParaVec& vecB);
__declspec(dllimport) bool isCoincident(const BPParaVec& vecA, const BPParaVec& vecB);
__declspec(dllimport) std::vector<BPParaVec> operator*(const BPParaTransform& mat, const std::vector<BPParaVec>& vecs);
__declspec(dllimport) double getAngleOfTwoVectors(const BPParaVec& vecA, const BPParaVec& vecB, bool isAbs = false);
__declspec(dllimport) bool _getGnrcDouble(const Gnrc& gnrc, double& outNum);
__declspec(dllimport) int mathSign(double x);


/* Warning, change the file to script automatically.
Please modify the source file under source. 
Source file: "BPParaTransform.h".*/

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
	__declspec(dllimport) BPParaTransform();
	__declspec(dllimport) ~BPParaTransform();
	__declspec(dllimport) BPParaTransform inverse() const;
	__declspec(dllimport) BPParaTransform operator*(const BPParaTransform& other) const;
	__declspec(dllimport) bool isValid() const; //isValidMatrix, except allZero NaN Inf
	__declspec(dllimport) explicit BPParaTransform(double a);
	__declspec(dllimport) BPParaTransform inverseOrth() const;
	__declspec(dllimport) BPParaTransform operator+(const BPParaTransform& other) const;
	__declspec(dllimport) BPParaTransform operator-(const BPParaTransform& other) const;
	__declspec(dllimport) BPParaVec operator*(const BPParaVec& other) const;
	__declspec(dllimport) BPParaTransform operator*(const double& other) const;
	__declspec(dllimport) bool operator<(const BPParaTransform& other) const; //std::less<void>
	__declspec(dllimport) bool operator==(const BPParaTransform& other) const;//std::equal_to<void>

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
__declspec(dllimport) BPParaTransform scale(double n);
__declspec(dllimport) BPParaTransform scale(double x, double y, double z = 1.0);
__declspec(dllimport) BPParaTransform scale(const BPParaVec& n);
inline BPParaTransform scalex(double x) { return scale(x, 1, 1); }
inline BPParaTransform scaley(double y) { return scale(1, y, 1); }
inline BPParaTransform scalez(double z) { return scale(1, 1, z); }
inline BPParaTransform scaleXoY(double s) { return scale(s, s, 1); }
// __declspec(dllimport) inline BPParaTransform scalePoint(double z);
//translate
__declspec(dllimport) BPParaTransform trans(double x, double y, double z = 0.0); //no implicit convert
__declspec(dllimport) BPParaTransform trans(const BPParaVec& point);
__declspec(dllimport) BPParaTransform transx(double x);
__declspec(dllimport) BPParaTransform transy(double y);
__declspec(dllimport) BPParaTransform transz(double z);
__declspec(dllimport) BPParaTransform translate(double x, double y, double z = 0.0);
__declspec(dllimport) BPParaTransform translate(const BPParaVec& point);
//rotate
__declspec(dllimport) BPParaTransform rotate(double angle, const BPParaVec& axis = BPParaVec(0, 0, 1));
__declspec(dllimport) BPParaTransform rotate(const BPParaVec& axis = BPParaVec(0, 0, 1), double angle = 0.0);
__declspec(dllimport) BPParaTransform rotateArbitrary(const BPParaVec& point = BPParaVec(0, 0, 0), const BPParaVec& vector = BPParaVec(0, 0, 1), double theta = 0.0);
__declspec(dllimport) BPParaTransform rotx(double theta);
__declspec(dllimport) BPParaTransform roty(double theta);
__declspec(dllimport) BPParaTransform rotz(double theta);
__declspec(dllimport) BPParaTransform getMatrixByEulerRPY(const BPParaVec& rpy);
__declspec(dllimport) BPParaVec getRPYByMatrix(const BPParaTransform& mat);
//liner matrix
// __declspec(dllimport) BPParaTransform mirror(const BPParaVec& axis);
// __declspec(dllimport) BPParaTransform mirror(const BPParaVec& axisA, const BPParaVec& axisB);
__declspec(dllimport) BPParaTransform mirror(const BPParaTransform& mat, const COORD_SYS& sys = COORD_SYS::PLANE_XOY/*const std::string& plane = "YOZ"*/); //关于面的镜像
inline BPParaTransform mirrorXoY(const BPParaTransform& mat = g_MatrixE) { return mirror(mat, COORD_SYS::PLANE_XOY); }
inline BPParaTransform mirrorXoZ(const BPParaTransform& mat = g_MatrixE) { return mirror(mat, COORD_SYS::PLANE_XOZ); }
inline BPParaTransform mirrorYoZ(const BPParaTransform& mat = g_MatrixE) { return mirror(mat, COORD_SYS::PLANE_YOZ); }
__declspec(dllimport) BPParaTransform shear(const COORD_SYS& axis, double a, double b);
inline BPParaTransform shearx(double y, double z) { return shear(COORD_SYS::AXIS_X, y, z); }
inline BPParaTransform sheary(double x, double z) { return shear(COORD_SYS::AXIS_Y, x, z); }
inline BPParaTransform shearz(double x, double y) { return shear(COORD_SYS::AXIS_Z, x, y); }
//matrix property
__declspec(dllimport) BPParaTransform transpose(const BPParaTransform& T);
__declspec(dllimport) BPParaTransform inverse(const BPParaTransform& T);
__declspec(dllimport) BPParaTransform inverseOrth(const BPParaTransform& T);
__declspec(dllimport) BPParaTransform operator*(double a, const BPParaTransform& b); //number*matrix
//operate-set
__declspec(dllimport) BPParaTransform setMatrixByColumnVectors(const BPParaVec& n, const BPParaVec& o, const BPParaVec& a, const BPParaVec& p = BPParaVec());
// __declspec(dllimport) BPParaTransform setMatrixByRowVectors(double row0[4], double row1[4], double row2[4]);
// __declspec(dllimport) BPParaTransform setMatrixByArray(double* mat[3][4]);
__declspec(dllimport) BPParaTransform setMatrixByRotAndPosition(const BPParaTransform& rot, const BPParaVec& position);
__declspec(dllimport) BPParaTransform setMatrixByRotAndPosition(const BPParaTransform& rot, const BPParaTransform& position);
__declspec(dllimport) BPParaTransform setMatrixByTwoVectors(const BPParaVec& vecX, const BPParaVec& vecY, bool isOrth = true); //
__declspec(dllimport) BPParaTransform setMatrixByValueList(double vars[12], bool isRow = true); //
__declspec(dllimport) BPParaTransform setMatrixByValueList(double nx, double ox, double ax, double px, double ny, double oy, double ay, double py, double nz, double oz, double az, double pz);
// __declspec(dllimport) double* getListFromMatrix(const BPParaTransform& mat, bool isRow = true); //
//operate-get
__declspec(dllimport) BPParaVec getMatrixsAxisX(const BPParaTransform& T);
__declspec(dllimport) BPParaVec getMatrixsAxisY(const BPParaTransform& T);
__declspec(dllimport) BPParaVec getMatrixsAxisZ(const BPParaTransform& T);
__declspec(dllimport) BPParaVec getMatrixsPosition(const BPParaTransform& T); // get position vector
__declspec(dllimport) BPParaTransform getMatrixsRotationPart(const BPParaTransform& T); // only rot, position is zero获
__declspec(dllimport) BPParaTransform getMatrixsPositionPart(const BPParaTransform& T); // only position matrix
__declspec(dllimport) BPParaVec getMatrixsScale(const BPParaTransform& T); // get scale vector
__declspec(dllimport) BPParaTransform getMatrixsScalePart(const BPParaTransform& T); // get scale matrix
//operate-judge
__declspec(dllimport) bool isIdentifyMatrix(const BPParaTransform& M); 
__declspec(dllimport) bool isZeroMatrix(const BPParaTransform& M, bool isAll = true); // all zero
__declspec(dllimport) bool isOrthogonalMatrix(const BPParaTransform& T, bool onlyRot = true); // 
__declspec(dllimport) BPParaTransform getOrthogonalMatrix(const BPParaTransform& T, bool withPosition = true);
// __declspec(dllimport) bool isAttitudeMatrix(const BPParaTransform& T); // 
__declspec(dllimport) bool isTwoDimensionalMatrix(const BPParaTransform& M);// only rotz&trans(x,y)
//matrix shadow
__declspec(dllimport) BPParaTransform shadowVectorMatrix2D(const BPParaVec& n); //arbitrary_shadow vector front shadow
// __declspec(dllimport) BPParaTransform shadowVectorMatrix3D(const BPParaVec& vec); // 3 dimensional shadow
__declspec(dllimport) BPParaTransform shadowScaleMatrix(double intAngle = 0); // enlarge shadow about axiaZ
//matrix genetate
__declspec(dllimport) BPParaTransform getMatrixFromTwoPoints(const BPParaVec& pointStart, const BPParaVec& pointEnd, bool isAxisZ = true);
__declspec(dllimport) BPParaTransform getMatrixFromTwoPoints(const BPParaVec& pointStart, const BPParaVec& pointEnd, bool withScale, bool isAxisZ);//overload
__declspec(dllimport) BPParaTransform getMatrixFromOneVector(const BPParaVec& vector, bool isAxisZ = true);
__declspec(dllimport) BPParaTransform getMatrixFromTwoVector(const BPParaVec& vecA, const BPParaVec& vecB);
__declspec(dllimport) BPParaTransform getMatrixFromThreePoints(const std::vector<BPParaVec>& points, bool is2D = false);
__declspec(dllimport) BPParaTransform getMatrixFromThreePoints(const BPParaVec& point1, const BPParaVec& point2, const BPParaVec& point3);
__declspec(dllimport) BPParaTransform getMatrixFromPoints(const std::vector<BPParaVec>& points, bool is2D = true);
//matrix invariance
__declspec(dllimport) bool hasMatrixTranslate(const BPParaTransform& M);		//T
__declspec(dllimport) bool hasMatrixRotate(const BPParaTransform& M);			//R
__declspec(dllimport) bool hasMatrixScale(const BPParaTransform& M);			//S
__declspec(dllimport) bool hasMatrixScaleNotEqual(const BPParaTransform& M);	//Sne
__declspec(dllimport) bool hasMatrixMirror(const BPParaTransform& M);			//M
__declspec(dllimport) bool hasMatrixShear(const BPParaTransform& M);			//H
__declspec(dllimport) bool hasMatrixTransform(const BPParaTransform& M, const MAT_TRANS_BAN& type);
__declspec(dllimport) std::vector<bool> hasMatrixTransform(const BPParaTransform& M); //combine transform
//__declspec(dllimport) bool getMatrixInvariance(const BPParaTransform& M, const BPParaInvariance& type);


/* Warning, change the file to script automatically.
Please modify the source file under source. 
Source file: "my_vec.h".*/
//class ClassInt :public IClass
//{
//public:
//	int m_imp;
//	ClassInt(int src) :
//		m_imp(src)
//	{
//	}
//};
//class ClassDouble :public IClass
//{
//public:
//	double m_imp;
//	ClassDouble(double src) :
//		m_imp(src)
//	{
//	}
//};
//Gene(double src)
//{
//	m_imp = new ClassDouble(src);
//}
//bool is(int)
//{
//	ClassInt* ptr = dynamic_cast<ClassInt*>(m_imp);
//	return ptr != nullptr;
//}
//bool is(double)
//{
//	ClassDouble* ptr = dynamic_cast<ClassDouble*>(m_imp);
//	return ptr != nullptr;
//}
//int& as(int)
//{
//	ClassInt* ptr = dynamic_cast<ClassInt*>(m_imp);
//	return ptr->m_imp;
//}
//double& as(double)
//{
//	ClassDouble* ptr = dynamic_cast<ClassDouble*>(m_imp);
//	return ptr->m_imp;
//}
//template<typename T> 
//inline Gene(const T& src)
//
//template<typename T>
//

//#define PL_A 1e-10 
static const double PL_A = 1e-10;

class Vec1
{
public:
	double x;
};


class Vec2
{
public:
	double x;
	double y;
	//double transform[16];
	__declspec(dllimport) Vec2() = default;
	__declspec(dllimport) Vec2(double x, double y) : x(x), y(y) {}
};

class Vec3 : public Vec2
	//struct Vec3
{
public:
	class Vec3Wrap
	{
	public:
		Vec3* m_imp;
		Vec3Wrap() :m_imp(nullptr) {}
		Vec3Wrap(Vec3* imp) :m_imp(imp) {}
		~Vec3Wrap() {}
		//Vec3Wrap(const Vec3Wrap& rhs);// = delete;

		Vec3Wrap(const Vec3Wrap& rhs)
		{//（深）拷贝构造，指针成员必须支持拷贝构造
			Vec3* vec = new Vec3(*rhs.m_imp);
			m_imp = vec;
		}
		Vec3Wrap(Vec3Wrap&& rhs) :m_imp(rhs.m_imp) { rhs.m_imp = nullptr; }
		bool operator==(const Vec3Wrap& rhs)const
		{
			return m_imp->x == rhs.m_imp->x && m_imp->y == rhs.m_imp->y && m_imp->z == rhs.m_imp->z;
		}

	};


	double x;
	double y;
	double z;
	//double* pvalue = NULL; // 初始化为 null 的指针
	//在 C++11 新标准中，可以通过 =default 关键来声明构造函数，告诉编译器为该类生成一个默认的版本，由编译器自己生成的默认构造函数，性能上一般会比用户自己定义的更好，而且也更有标志性，便于代码的阅读。
	__declspec(dllimport) Vec3() = default; //C++11 new feature
	//Vec3() : x(0), y(0), z(0)
	//{
	//	std::cout << "default construct" << std::endl;
	//}

	__declspec(dllimport) Vec3(double x, double y, double z = 0) : x(x), y(y), z(z)
	{
		//pvalue = new double[5e2];
		std::cout << "param construct" << std::endl;
	}
	__declspec(dllimport) Vec3(const Vec3& vec) : x(vec.x), y(vec.y), z(vec.z) //拷贝构造
	{
		std::cout << "copy construct" << std::endl;
	}

	__declspec(dllimport) Vec3(Vec3&& rhs) //移动构造
	{
		//this = &rhs;
		//rhs.x = 0;
		//rhs.y = 0;
		//rhs.z = 0;
		this->x = rhs.x;
		this->y = rhs.y;
		this->z = rhs.z;
		std::cout << "Move Constructor" << std::endl;
	}

	__declspec(dllimport) Vec3 operator=(const Vec3& rhs)  //拷贝赋值
	{
		if (&rhs!=this)
		{
			x = rhs.x;
			y = rhs.y;
			z = rhs.z;
		}
		std::cout << "copy operator=" << std::endl;
		return *this;
	}

	__declspec(dllimport) Vec3 operator=(Vec3&& rhs)  //移动赋值
	{
		this->x = rhs.x;
		this->y = rhs.y;
		this->z = rhs.z;
		std::cout << "move operator=" << std::endl;
		return *this;
	}
	/*
	可以为任何类指定显式移动构造函数或移动运算符，无论它是否具有指针成员。确实，移动构造函数和运算符通常与具有某种指针的类一起使用。但这只是因为在这些情况下，有一种方法可以避免更昂贵的复制开销。

	平凡类型的默认移动操作等效于复制操作。类的默认移动操作相当于类的每个成员的移动操作。

	因此，在您的示例中，移动操作等效于复制操作，并且没有可观察到的差异。
	*/

	//compare
	__declspec(dllimport) bool operator==(const Vec3& vec) const
	{
		return abs(vec.x - x) + abs(vec.y - y) + abs(vec.z - z) < PL_A;
	}

	__declspec(dllimport) bool operator<(const Vec3& vec) const
	{
		return vec.x * vec.x + vec.y * vec.y + vec.z * vec.z < x* x + y * y + z * z;
	}

	__declspec(dllimport) operator bool() const
	{
		return (abs(x) + abs(y) + abs(z) < PL_A);
	}

	__declspec(dllimport) ~Vec3()
	{
		//delete[] pvalue;
		//std::cout << "vec destruct" << std::endl;
	}
	void test()
	{

	}
	void test() const
	{
		//前面使用const 表示返回值为const
		//后面加 const表示函数不可以修改class的成员
	}

};

class Segment
{
public:
	Vec3 m_start;
	Vec3 m_end;
	__declspec(dllimport) Segment(Vec3 start, Vec3 end):m_start(start),m_end(end)
	{}
	__declspec(dllimport) Vec3 operator[](bool i) const
	{
		return i ? m_end : m_start;
	}
};

namespace std
{
	template<>
	struct hash<Vec3> //: public __hash_base<size_t, MyClass>   //标准库中有这个继承，查看一下其实只是继承两个typedef而已，
	{
		size_t operator()(const Vec3& rhs) const noexcept    //这个const noexpect一定要写上去
		{
			return (std::hash<double>()(rhs.x)) ^ (std::hash<double>()(rhs.y) << 1) ^ (std::hash<double>()(rhs.z) << 2); //当然，可以使用其他的方式来组合这个哈希值,
																							//这里是cppreference里面的例子，产生的数够乱就行。
		}
	};

	//wrap class
	template<>
	struct hash<Vec3::Vec3Wrap>
	{
		size_t operator()(const Vec3::Vec3Wrap& rhs) const noexcept
		{
			return (std::hash<double>()(rhs.m_imp->x)) ^ (std::hash<double>()(rhs.m_imp->y) << 1) ^ (std::hash<double>()(rhs.m_imp->z) << 2);
		}
	};
}

//template<> struct hash<Vec3>
//{
//	size_t operator()(const Vec3& gene)
//	{
//		size_t hashn = 0;
//		return hashn;
//	}
//};






//----------------------------------------------------------------------------------------------

//class IC
//{
//public:
//	IC() {}
//	virtual ~IC() {}
//	virtual bool operator==(const IC& p) const = 0;
//	virtual bool operator>(const IC& p) const = 0;
//	virtual bool operator<(const IC& p) const = 0;
//	virtual std::type_index _type() const = 0;
//};
//
//template <typename T>
//class Templ :public IC //模板类
//{
//public:
//	T m_imp;
//	Templ(T src) :m_imp(src) {}
//	~Templ() {}
//	virtual bool operator==(const IC& p) const override
//	{
//		const Templ* ptr = dynamic_cast<const Templ*>(&p);
//		if (ptr == nullptr)
//			return false;
//		return ptr->m_imp == m_imp;
//	}
//
//	virtual bool operator>(const IC& p) const override
//	{
//		const Templ* ptr = dynamic_cast<const Templ*>(&p);
//		if (ptr == nullptr)
//			return p._type() > _type();
//		return ptr->m_imp > m_imp;
//	}
//	virtual bool operator<(const IC& p) const override
//	{
//		const Templ* ptr = dynamic_cast<const Templ*>(&p);
//		if (ptr == nullptr)
//			return p._type() < _type();
//		return ptr->m_imp < m_imp;
//	}
//	virtual std::type_index _type() const override
//	{
//		return typeid(T);
//	}
//};
//
////类继承
//class CA :public IC
//{
//public:
//	CA(int src = 0) :i(src) {}
//	~CA() {}
//	virtual bool operator==(const IC& p) const override
//	{
//		const CA* ptr = dynamic_cast<const CA*>(&p);
//		if (ptr == nullptr)
//			return false;
//		return ptr->i == i;
//	}
//	virtual bool operator>(const IC& p) const override
//	{
//		const CA* ptr = dynamic_cast<const CA*>(&p);
//		if (ptr == nullptr)
//			return p._type() > _type();
//		return ptr->i > i;
//	}
//	virtual bool operator<(const IC& p) const override
//	{
//		const CA* ptr = dynamic_cast<const CA*>(&p);
//		if (ptr == nullptr)
//			return p._type() < _type();
//		return ptr->i < i;
//	}
//	virtual std::type_index _type() const override
//	{
//		return typeid(int);
//	}
//	int i;
//};
//
//class CB :public IC
//{
//public:
//	CB(string src = "s") :s(src) {}
//	~CB() {}
//	virtual bool operator==(const IC& p) const override
//	{
//		const CB* ptr = dynamic_cast<const CB*>(&p);
//		if (ptr == nullptr)
//			return false;
//		return ptr->s == s;
//	}
//	virtual bool operator>(const IC& p) const override
//	{
//		const CB* ptr = dynamic_cast<const CB*>(&p);
//		if (ptr == nullptr)
//			return p._type() > _type();
//		return ptr->s > s;
//	}
//	virtual bool operator<(const IC& p) const override
//	{
//		const CB* ptr = dynamic_cast<const CB*>(&p);
//		if (ptr == nullptr)
//			return p._type() < _type();
//		return ptr->s < s;
//	}
//	virtual std::type_index _type() const override
//	{
//		return typeid(std::string);
//	}
//	std::string s;
//};
//
//


class GeTransform
{
public:
	double x = 0;
	double y = 0;
	double z = 0;
	GeTransform(){}
	GeTransform(double _x, double _y, double _z):x(_x), y(_y), z(_z){}
	GeTransform(const GeTransform& rhs) :x(rhs.x), y(rhs.y), z(rhs.z) {}
	~GeTransform(){}


};

class BPTransfrom
{
	double x = 0;
	double y = 0;
	double z = 0;
public:
	BPTransfrom(){}
	BPTransfrom(double _x, double _y, double _z) :x(_x), y(_y), z(_z) {}
	BPTransfrom(const BPTransfrom& rhs) :x(rhs.x), y(rhs.y), z(rhs.z) {}
	BPTransfrom(const GeTransform& rhs) :x(rhs.x), y(rhs.y), z(rhs.z) {}
	~BPTransfrom(){}

	inline operator GeTransform() const
	{
		return GeTransform(x, y, z);
	}

};

/* Warning, change the file to script automatically.
Please modify the source file under source. 
Source file: "my_md5.h".*//**
 * @file md5.h
 * @The header file of md5.
 * @author Jiewei Wei
 * @mail weijieweijerry@163.com
 * @github https://github.com/JieweiWei
 * @data Oct 19 2014
 *
 */


/* Define of btye.*/
typedef unsigned char Byte;
/* Define of Byte. */
typedef unsigned int bit32;

class MD5 {
public:
	/* Construct a MD5 object with a string. */
	__declspec(dllimport) MD5(const std::string& message);
	/* Generate md5 digest. */
	__declspec(dllimport) const Byte* getDigest();
	/* Convert digest to string value */
	__declspec(dllimport) std::string toStr();

private:
	/* Initialization the md5 object, processing another message block,
	 * and updating the context.*/
	void init(const Byte* input, size_t len);

	/* MD5 basic transformation. Transforms state based on block. */
	void transform(const Byte block[64]);

	/* Encodes input (usigned long) into output (Byte). */
	void encode(const bit32* input, Byte* output, size_t length);

	/* Decodes input (Byte) into output (usigned long). */
	void decode(const Byte* input, bit32* output, size_t length);

private:
	/* Flag for mark whether calculate finished. */
	bool finished;

	/* state (ABCD). */
	bit32 state[4];

	/* number of bits, low-order word first. */
	bit32 count[2];

	/* input buffer. */
	Byte buffer[64];

	/* message digest. */
	Byte digest[16];

	/* padding for calculate. */
	static const Byte PADDING[64];

	/* Hex numbers. */
	static const char HEX_NUMBERS[16];
};


namespace para
{
	__declspec(dllimport) std::string getMD5(const std::string& source);
}



/* Warning, change the file to script automatically.
Please modify the source file under source. 
Source file: "my_ref_count.h".*///
//#include <map>
//#include <unordered_map>
//#include <type_traits>

class Primitive
{
public:
	virtual ~Primitive() {}
};

#define sizeMax 100

// ------------------------------internal----------------------------------------

class ToGeCone :public Primitive
{

public:
	class WrapCone
	{
	public:
		WrapCone(const Primitive* ptr);
		WrapCone(const WrapCone& rhs);//copy
		WrapCone(WrapCone&& rhs); //move
		~WrapCone();
		//bool operator<(const WrapCone& wrap)const { return false; } //invalid
		bool operator==(const WrapCone& wrap)const;// { return false; }
		const ToGeCone* m_imp;
	};

	double m_r;
	double m_h;

	static const ToGeCone* create(double r, double h);
	static void countIncrease(const Primitive* imp);
	static void countDecrease(const Primitive* imp); //while count==0 delete
	static const Primitive* set_h(const Primitive* ptr, double h);
	static const double get_h(const Primitive* ptr);
	static size_t sm_totalCount;
	//static std::map<WrapCone, size_t> sm_coneMap;
	static std::unordered_map<ToGeCone::WrapCone, size_t> sm_coneMap;
};



namespace std
{
	template<>
	struct hash<ToGeCone>
	{
		size_t operator()(const ToGeCone& rhs) const noexcept
		{
			return (std::hash<double>()(rhs.m_r)) ^ (std::hash<double>()(rhs.m_h) << 1);
		}
	};

	template<>
	struct hash<ToGeCone::WrapCone>
	{
		size_t operator()(const ToGeCone::WrapCone& rhs) const noexcept
		{
			return (std::hash<double>()(rhs.m_imp->m_r)) ^ (std::hash<double>()(rhs.m_imp->m_h) << 1);
		}
	};
}



// ------------------------------external----------------------------------------


class Cone
{
public:
	__declspec(dllimport) Cone();
	__declspec(dllimport) Cone(double r, double h);
	__declspec(dllimport) ~Cone();
	__declspec(dllimport) void set_h(double h);
	__declspec(dllimport) double get_h() const;

	const Primitive* m_imp;
};





/* Warning, change the file to script automatically.
Please modify the source file under source. 
Source file: "my_handle.h".*/
class BPParaHandle
{
public:
    enum HandleType : unsigned char
    {
        enInvalid = 0,
        enPropertyHandle = 10,
    };
    size_t m_handle1 = 0;
    size_t m_handle2 = 0;
    unsigned char m_type = 0;

public:
    template<typename T>
    bool isHandleType() const
    {
        static_assert(std::is_base_of<BPParaHandle, T>::value, "Only judgement between handles is allowed.");
        //return m_handle[15] == T::getHandleType();
        return m_type == T::getHandleType();
    }
    template<typename T>
    inline T asHandleType() const
    {
        static_assert(std::is_base_of<BPParaHandle, T>::value, "Only judgement between handles is allowed.");
        T res;
        res.m_handle1 = m_handle1;
        res.m_handle2 = m_handle2;
        res.m_type = T::getHandleType();
        return res;
    }
    BPParaHandle()
    {
    }
    BPParaHandle(const BPParaHandle& rhs) :m_handle1(rhs.m_handle1), m_handle2(rhs.m_handle2), m_type(rhs.m_type)
    {
    }
    BPParaHandle(size_t handle1, size_t handle2, HandleType type):m_handle1(handle1), m_handle2(handle2), m_type(type)
    {
    }
    virtual ~BPParaHandle()
    {
    }
    inline bool operator<(const BPParaHandle& _r) const
    {
        //char* pOffset=(char*)(this);        pOffset += sizeof(void*); //vtptr
        //size_t* hd1 = (size_t*)(pOffset);   pOffset += sizeof(size_t);
        //size_t* hd2 = (size_t*)(pOffset);   pOffset += sizeof(size_t);

        size_t* pOffset = (size_t*)this;                   pOffset += 1;
        size_t* hd1 = (size_t*)(pOffset);       pOffset += 1;
        size_t* hd2 = (size_t*)(pOffset);       pOffset += 1;

        //BPParaHandle copy = *this;
        //memcpy(&copy, nullptr, sizeof(void*));

        //当一个类拥有两个以上的成员时，需要写统一的比较函数
		int res0 = memcmp((char*)(this) + sizeof(void*), (char*)(&_r) + sizeof(void*), sizeof(size_t) + sizeof(size_t) + sizeof(unsigned char)); //v
		int res1 = memcmp((size_t*)(this) + 1, (size_t*)(&_r) + 1, sizeof(size_t) + sizeof(size_t) + sizeof(unsigned char)); 
		int res2 = memcmp(this, &_r , sizeof(void*)+ sizeof(size_t) + sizeof(size_t) + sizeof(unsigned char)); //x
        return res2==-1;
        //this 是此时父类虚表指针，_r的首地址是子类的虚表指针

        if (!(m_handle1 < _r.m_handle1)) 
            return false;
        if (!(m_handle2 < _r.m_handle2))
            return false;
        return m_type < _r.m_type;


        //if (!m_handle1 < _r.m_handle1) //try_emplace将无法判断
        //    return false;
        //if (!m_handle2 < _r.m_handle2)
        //    return false;
        //return m_type < _r.m_type;

        //return m_handle1 < _r.m_handle1 || m_handle2 < _r.m_handle2 || m_type < _r.m_type;
        size_t mhash = (std::hash<size_t>()(m_handle1)) ^ (std::hash<size_t>()(m_handle2) << 1) ^ (std::hash<unsigned char>()(m_type) << 2);
        size_t rhash = (std::hash<size_t>()(_r.m_handle1)) ^ (std::hash<size_t>()(_r.m_handle2) << 1) ^ (std::hash<unsigned char>()(_r.m_type) << 2);
        //return mhash < rhash;
    }
    inline bool operator==(const BPParaHandle& _r) const
    {
        return m_handle1 == _r.m_handle1 && m_handle2 < _r.m_handle2 && m_type == _r.m_type;
    }
};


class BPPropertyHandle :
    public BPParaHandle
{
public:
    inline static HandleType getHandleType()
    {
        return BPParaHandle::enPropertyHandle;
    }
    __declspec(dllimport) BPPropertyHandle() {}
    __declspec(dllimport) BPPropertyHandle(size_t handle1, size_t handle2) //:m_handle(handle), m_type(type)
    {
        m_handle1 = handle1;
        m_handle2 = handle2;
		m_type = BPParaHandle::enPropertyHandle;
    }
    __declspec(dllimport) ~BPPropertyHandle() {}
};



