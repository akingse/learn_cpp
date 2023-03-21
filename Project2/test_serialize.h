#pragma once
using namespace std;
class Vec3
{
public:
	double x;
	double y;
	double z;
	__declspec(dllexport) Vec3() = default; //C++11 new feature
	__declspec(dllexport) Vec3(double x, double y, double z = 0) : x(x), y(y), z(z)
	{
	}
	__declspec(dllexport) Vec3(const Vec3& vec) : x(vec.x), y(vec.y), z(vec.z) //拷贝构造
	{
	}

	__declspec(dllexport) Vec3(Vec3&& rhs) //移动构造
	{
		this->x = rhs.x;
		this->y = rhs.y;
		this->z = rhs.z;
	}

	__declspec(dllexport) Vec3 operator=(const Vec3& rhs)  //拷贝赋值
	{
		if (&rhs != this)
		{
			x = rhs.x;
			y = rhs.y;
			z = rhs.z;
		}
		return *this;
	}

	__declspec(dllexport) Vec3 operator=(Vec3&& rhs)  //移动赋值
	{
		this->x = rhs.x;
		this->y = rhs.y;
		this->z = rhs.z;
		return *this;
	}
	//compare
	__declspec(dllexport) bool operator==(const Vec3& vec) const
	{
		return abs(vec.x - x) + abs(vec.y - y) + abs(vec.z - z) < PL_A;
	}
	__declspec(dllexport) bool operator<(const Vec3& vec) const
	{
		return vec.x * vec.x + vec.y * vec.y + vec.z * vec.z < x* x + y * y + z * z;
	}
	__declspec(dllexport) operator bool() const
	{
		return (abs(x) + abs(y) + abs(z) < PL_A);
	}
	__declspec(dllexport) ~Vec3()
	{
	}
};



class BPGeometricPrimitiveSer
{
public:
	bool m_hollow = false;
	size_t m_identification;
	string m_remark;
	BPGeometricPrimitiveSer() = default;
	BPGeometricPrimitiveSer(bool hollow, size_t identification, const string& remark)
	{
		m_hollow = hollow;
		m_identification = identification;
		m_remark = remark;
	}
	BPGeometricPrimitiveSer(const BPGeometricPrimitiveSer&) = default;
	~BPGeometricPrimitiveSer() = default;

	template<class T>
	bool is() 
	{
		return true;
	}
	template<class T>
	T as()
	{
		return T;
	}
};
