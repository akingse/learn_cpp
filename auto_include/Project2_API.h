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
Source file: "my_geometry.h".*/
class DpIn
{
public:
	DpIn() :
		m_funType(typeid(para::None)),
		m_fun(nullptr)
	{
	}
	std::type_index m_funType;
	std::string m_name;
	void* m_fun;

	template<typename T>
	bool is()
	{
		std::type_index tmp0 = typeid(T);
		std::string tmp1 = typeid(T).name();
		std::type_index tmp2 = typeid(para::None);
		std::string tmp3 = typeid(para::None).name();
		return m_funType == typeid(T);
	}

	template<typename T>
	T* as()
	{
		if (!is<T>())
			return nullptr;
		return (T*)m_fun;
	}
};

static std::map <std::string, std::map<std::string, DpIn*> > g_funMap;
template<typename T>
inline bool interface_enrol(std::string object, std::string propertyID, T t)
{
	if (t == nullptr)
		return false;
	DpIn* dp = new DpIn();
	dp->m_fun = &t;
	dp->m_funType = typeid(T);//Cube:setHigh(int)
	dp->m_name = typeid(T).name();
 	std::string name = typeid(T).name();//"void (__cdecl Cube::*)(int) __ptr64"
	if (g_funMap.find(object) == g_funMap.end())
		g_funMap[object] = { {propertyID, dp} };
	else
	{
		std::map<std::string, DpIn*>& funMap = g_funMap[object];
		funMap[propertyID] = dp;
	}
	return true;
}
template<typename T>
inline void interface_enrol(std::string object, std::string propertyID, T* t)
{
	if (t == nullptr)
		return;
	DpIn* dp = new DpIn();
	dp->m_fun = t;
	dp->m_funType = typeid(T); //cube_setlength(class BPObject *,int)
	dp->m_name = typeid(T).name();
	std::string name = typeid(T).name();//"bool __cdecl(class BPObject * __ptr64,int)"
	if (g_funMap.find(object) == g_funMap.end())
		g_funMap[object] = { {propertyID, dp} };
	else
	{
		std::map<std::string, DpIn*>& funMap = g_funMap[object];
		funMap[propertyID] = dp;
	}
}

inline DpIn* getFun(std::string object, std::string funName)
{
	if (g_funMap.find(object) == g_funMap.end())
		return nullptr;
	std::map<std::string, DpIn*>& funMap = g_funMap[object];
	if (funMap.find(funName) == funMap.end())
		return nullptr;
	return funMap[funName];
}


class BPObject
{
public:
	BPObject() {};
	virtual ~BPObject()
	{

	}
	virtual std::string getClassName() { return {}; };

	//bool operator<(const BPObject& rhs)
	//{
	//	//this
	//	return false;
	//}
};
class Cube : public BPObject
{
public:
	int m_l;
	int m_w;
	int m_h;
	Cube(int l, int w, int h) :
		m_l(l),
		m_w(w),
		m_h(h)
	{

	}
	virtual ~Cube()
	{

	}
	virtual std::string getClassName() override final
	{
		return "CUBE";
	}

	void setLength(int length)
	{
		m_l = length;
	}
	void setWidth(int width)
	{
		m_w = width;
	}
	void setHigh(int high)
	{
		m_h = high;
	}
	//实现需要独立写
	//int getLength()
	//{
	//	return m_l;
	//}
	int getLength() const 
	{
		return m_l;
	}
	int getWidth() const
	{
		return m_w;
	}
	int getHigh() const
	{
		return m_h;
	}
	bool setArea(int length, int width)
	{
		m_l = length;
		m_w = width;
		return true;
	}

};

inline bool cube_setLength(BPObject* a, int length)
{
	//if (!a)
	//	return false;
	Cube* cube = dynamic_cast<Cube*>(a);
	if (!cube)
		return false;
	cube->setLength(length);
	return true;
}

inline bool cube_setWidth(BPObject* a, int width)
{
	if (!a)
		return false;
	Cube* cube = dynamic_cast<Cube*>(a);
	if (!cube)
		return false;
	cube->setWidth(width);
	return true;
}

inline bool cube_setHigh(BPObject* a, int high)
{
	if (!a)
		return false;
	Cube* cube = dynamic_cast<Cube*>(a);
	if (!cube)
		return false;
	cube->setHigh(high);
	return true;
}

class CatchTool
{
public:
	virtual ~CatchTool()
	{

	}
	virtual void _onDataButton();
	virtual void _dynamicFrame();
};
class CatchCubePropertyTool : public CatchTool
{
public:
	//point m_point1;
	//point m_point2;
	//point m_point3;
	//point m_point4;
	//point m_point5;
	//point m_point1;
	//point m_point2;
	//point m_point3;
	//point m_point4;
	//point m_point5;
	virtual void _onDataButton() override;
	virtual void _dynamicFrame() override;
};

class BPGeometricPrimitive
{
public:
	BPGeometricPrimitive(BPObject* object, bool hollow) :
		m_imp(object),
		m_className("NULL"),
		ishollow(hollow)
	{
		initIndex();
	}
	BPObject* m_imp;
	std::string m_className;
	bool ishollow;
	void setPropertyValue(std::string propertyName, int length)
	{
		DpIn* dp = getFun(m_className, propertyName);
		if (!dp)
			return;
		if (dp->is<bool(BPObject*, int)>())
		{
			std::function<bool(BPObject*, int)> _fun = dp->as<bool(BPObject*, int)>();
			_fun(m_imp, length);
		}
		else if (dp->is<void(Cube::*)(int)>())
		//else if (dp->is<&Cube::setWidth>())
		{
			if (!dp->m_fun)
				return;
			//void(Cube:: *p)(int)= &Cube::setWidth;
			//int width;
			//((p)(dp->m_fun))(this, width);
			auto ptr = dp->as<void(Cube::*)(int)>();
			//(ptr)(this, length);
			std::function<void(Cube*, int)> fp = *ptr;
			Cube* sub = dynamic_cast<Cube*>(m_imp);
			// std::function<void(Cube*, int)> fp1 = &Cube::setHigh;
			fp(sub, length);
		}
	}

	void initIndex()
	{
		if (!m_imp)
			return;
		std::type_index a1 = typeid(m_imp); //typeid识别动态类型：指针的解引用
		std::type_index b1 = typeid(*m_imp);

		std::string a2 = typeid(m_imp).name();
		std::string b2 = typeid(*m_imp).name();


		Cube* cube = dynamic_cast<Cube*>(m_imp);
		std::string a3 = typeid(cube).name();
		std::string a4 = typeid(cube).raw_name();
		size_t a5 = typeid(cube).hash_code();


		bool eq = (typeid(Cube) == typeid(m_imp));
		for (auto& iter : g_funMap)
		{
			if (iter.first == m_imp->getClassName())
			{
				m_className = iter.first;
				break;
			}
		}
	}
};


/* Warning, change the file to script automatically.
Please modify the source file under source. 
Source file: "test_triangular.h".*/

namespace psykronix
{
    class Vertex
    {
    public:
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        Vertex() {}
		Vertex(double x_, double y_, double z_ = 0.0) :
            x(x_),
            y(y_),
            z(z_)
        {
        }
        inline bool operator==(const Vertex& other) const
        {
            return x == other.x &&
                y == other.y &&
                z == other.z;
        }
#ifdef EIGEN_WORLD_VERSION
        Vertex(const Eigen::Vector3d& vec) :
            x(vec.x()),
            y(vec.y()),
            z(vec.z())
        {
        }
        operator Eigen::Vector3d() const
        {
            return Eigen::Vector3d(x, y, z);
        }
        operator Eigen::Vector3f() const
        {
            return Eigen::Vector3f(x, y, z);
        }
#endif
    };

    //overload
    __declspec(dllimport) Eigen::Matrix4d rotx(double theta);
    __declspec(dllimport) Eigen::Matrix4d roty(double theta);
    __declspec(dllimport) Eigen::Matrix4d rotz(double theta);
	__declspec(dllimport) Eigen::Matrix4d rotate(const Eigen::Vector3d& axis = { 0, 0, 1 }, double theta = 0.0);
	__declspec(dllimport) Eigen::Matrix4d translate(const Eigen::Vector3d& vec);
	//__declspec(dllimport) Eigen::Matrix4d translate(const Eigen::Vector4d& vec);
	__declspec(dllimport) Eigen::Matrix4d translate(double x, double y, double z = 0.0);
    __declspec(dllimport) Eigen::Matrix4d scale(double x, double y, double z = 1.0);
    __declspec(dllimport) std::array<Eigen::Vector3d, 3> operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector3d, 3>& tri);
    __declspec(dllimport) std::array<Eigen::Vector3f, 3> operator*(const Eigen::Matrix4d& mat, const std::array<Eigen::Vector3f, 3>& tri);
    //__declspec(dllimport) std::array<Eigen::Vector3f, 3> operator*(const Eigen::Matrix4f& mat, const std::array<Eigen::Vector3f, 3>& tri);

}

//__declspec(dllimport) bool _isTwoTriangularIntersection(const std::array<BPParaVec, 3>& tBase, const std::array<BPParaVec, 3>& tLine);
__declspec(dllimport) bool isTwoTrianglesIntersection(const std::array<Eigen::Vector3f, 3>& triL, const std::array<Eigen::Vector3f, 3>& triR);
__declspec(dllimport) bool isTwoTrianglesIntersection1(const std::array<Eigen::Vector3d, 3>& triL, const std::array<Eigen::Vector3d, 3>& triR);
__declspec(dllimport) bool isTwoTrianglesIntersection2(const std::array<Eigen::Vector3d, 3>& triL, const std::array<Eigen::Vector3d, 3>& triR);
__declspec(dllimport) bool isTwoTrianglesIntersection(const std::array<Eigen::Vector3d, 3>& triL, const std::array<Eigen::Vector3d, 3>& triR);
__declspec(dllimport) bool TriangularIntersectionTest(const std::array<Eigen::Vector3d, 3>& T1, const std::array<Eigen::Vector3d, 3>& T2);
__declspec(dllimport) bool TriangularIntersectionTest(const std::array<Eigen::Vector3f, 3>& T1, const std::array<Eigen::Vector3f, 3>& T2);



