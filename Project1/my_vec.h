#pragma once
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
	__declspec(dllexport) Vec2() = default;
	__declspec(dllexport) Vec2(double x, double y) : x(x), y(y) {}
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
	__declspec(dllexport) Vec3() = default; //C++11 new feature
	//Vec3() : x(0), y(0), z(0)
	//{
	//	std::cout << "default construct" << std::endl;
	//}

	__declspec(dllexport) Vec3(double x, double y, double z = 0) : x(x), y(y), z(z)
	{
		//pvalue = new double[5e2];
		std::cout << "param construct" << std::endl;
	}
	__declspec(dllexport) Vec3(const Vec3& vec) : x(vec.x), y(vec.y), z(vec.z) //拷贝构造
	{
		std::cout << "copy construct" << std::endl;
	}

	__declspec(dllexport) Vec3(Vec3&& rhs) //移动构造
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

	__declspec(dllexport) Vec3 operator=(const Vec3& rhs)  //拷贝赋值
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

	__declspec(dllexport) Vec3 operator=(Vec3&& rhs)  //移动赋值
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
	__declspec(dllexport) Segment(Vec3 start, Vec3 end):m_start(start),m_end(end)
	{}
	__declspec(dllexport) Vec3 operator[](bool i) const
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