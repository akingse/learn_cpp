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
Source file: "my_gnrc.h".*/// Generic
//using namespace rel_ops;

class IClass  //�����
{
public:
	virtual ~IClass()
	{
		//�����е�������������Ϊ�麯�����������ֶ����ͷŴ���
		//std::cout << "IClass destruct" << std::endl;
	}
	virtual IClass* _constructor() const = 0; //make_constructor����
	virtual bool _is(const std::type_index& type) const = 0;
	virtual bool _is_equal(const IClass& other) const = 0;
	virtual bool _is_less(const IClass& other) const = 0;
	virtual std::type_index _get_id() const = 0;
protected:
	virtual bool _copy_constructor(const IClass* src) = 0; //_deep_copy_from��������
	//virtual bool _operator_eq_copy(const IClass& src) = 0; //��ֵ
	//virtual bool _move_constructor(const IClass* src) = 0; //�ƶ�����
	//virtual bool _operator_eq_rr(const IClass&& src) = 0; //�ƶ���ֵ
	//virtual bool operator==(const IClass& p) const = 0;
	//virtual bool operator!=(const IClass& p) const = 0;
	//virtual bool operator<(const IClass& p) const = 0;
	//virtual bool operator>(const IClass& p) const = 0;
	//virtual bool operator>=(const IClass& p) const = 0;
	//virtual bool operator<=(const IClass& p) const = 0;
	//virtual std::type_index _type() const = 0;
};


//c++��дģ�溯��ʱ��template<class T>֮��ģ���ͷ�ļ�������cpp�ļ����롣
//�����ζ�ߣ���ͷ�ļ�����ĺ�ģ��ĵط�������ͷ�ļ���ʵ�֣�û��ģ�涨��ĵط����Է���cpp��ʵ�֡�
template <typename T>
class Templator :public IClass //ģ����
{
public:
	T m_imp;
	Templator(T src) : m_imp(src)
	{
		//1 T�Ŀ������������ Tempʵ��
		//2 Gnrc�޷��������� ����������ֻ�ܵ��ø�����ӿڣ��Լ̳������ʵ��
	}
	~Templator()
	{
		//std::cout << "Temp destruct" << std::endl;

	}
	inline IClass* _constructor() const override
	{
		return new Templator<T>(m_imp);
	}

protected:
	bool _copy_constructor(const IClass* src) override
	{
		if (!src->_is(typeid(T)))
			return false;
		const Templator<T>* ptr = dynamic_cast<const Templator<T>*>(src);
		if (nullptr == ptr)
			return false;
		m_imp = ptr->m_imp;
		return true;
	}
	inline bool _is(const std::type_index& type) const override
	{
		return type == typeid(T) ;
		//return typeid(T) == type;
	}

	//bool operator==(const IClass& other) const override
	bool _is_equal(const IClass& other) const override
	{
		const Templator<T>* ptr = dynamic_cast<const Templator<T>*>(&other);
		if (!ptr) //(ptr == nullptr)
			return false;
		return ptr->m_imp == this->m_imp;
	}
	bool _is_less(const IClass& other) const override
	{
		if (!(&other))
			return false;
		const Templator<T>* ptr = dynamic_cast<const Templator<T>*>(&other);
		if (!ptr)
		{
			/*string name1 = typeid(*this).name();
			string name2 = typeid(other).name();
			if (name1< name2)
				int a = 1;*/
			return typeid(*this).name() < typeid(other).name(); //compare string
		}
		return this->m_imp < ptr->m_imp;
	}

	inline std::type_index _get_id() const
	{
		return typeid(T);
	}
};


template <typename T>
class TemplatorRef :public IClass //����ģ����
{
public:
	T* m_imp;
	bool m_ref;
	TemplatorRef() : m_imp(nullptr), m_ref(false)
	{
	}
	TemplatorRef(T* src,bool isRef=false)//: m_imp(src), m_ref(isRef)
	{
		m_ref = isRef;
		m_imp = new T(*src);
	}
	~TemplatorRef()
	{
		if (m_ref) //only destruct value, not ref
			return;
		/*else if (!m_ref &&!m_imp) //deleteֻɾ��ָ��ָ����ڴ�ռ�-ֵ
			return;*/
		else
		{
			delete m_imp; //
			m_imp = nullptr;
		}
	}
	inline IClass* _constructor() const override
	{
		return new TemplatorRef<T>(m_imp, m_ref);
	}

protected:
	bool _copy_constructor(const IClass* src) override
	{
		if (!src->_is(typeid(T)))
			return false;
		const TemplatorRef<T>* ptr = dynamic_cast<const TemplatorRef<T>*>(src);
		if (nullptr == ptr)
			return false;
		m_imp = ptr->m_imp;
		return true;
	}
	inline bool _is(const std::type_index& type) const override
	{
		return type == typeid(T);
	}

	bool _is_equal(const IClass& other) const override
	{
		const TemplatorRef<T>* ptr = dynamic_cast<const TemplatorRef<T>*>(&other);
		if (!ptr) //(ptr == nullptr)
			return false;
		return ptr->m_imp == this->m_imp;
	}
	bool _is_less(const IClass& other) const override
	{
		if (!(&other))
			return false;
		const TemplatorRef<T>* ptr = dynamic_cast<const TemplatorRef<T>*>(&other);
		if (!ptr)
			return typeid(*this).name() < typeid(other).name(); //compare string
		return this->m_imp < ptr->m_imp;
	}

	inline std::type_index _get_id() const
	{
		return typeid(T*); // T/T*
	}
};

class None
{
public:
	//std::nullptr_t m_null; //�ղ���
	None() {}; // : m_null(nullptr) {}
	bool operator<(const None& other) const
	{
		return false;
	}
	bool operator==(const None& other) const
	{
		return true;
	}
};


////��ʱGnrc��ָ�뿽��
//class Gnrc
//{
//	type_index m_id;
//	void* m_imp;
//	bool m_ref;
//
//public:
//
//	Gnrc(std::type_index id, void* src = nullptr, bool isRef = false) :
//		m_id(id),//(id = typeid(None)),
//		m_imp(src),
//		m_ref(isRef)
//	{
//	}
//
//	template<typename T>
//	inline bool is() const
//	{
//		return m_id == typeid(T) && (m_imp);
//	}
//
//	template<typename T>
//	inline T& as()
//	{
//		assert(is<T>());
//		return static_cast<T>(m_imp);
//	}
//	template<typename T>
//	inline const T& as() const 
//	{
//		assert(is<T>());
//		return static_cast<T>(m_imp);
//	}
//
//	~Gnrc()
//	{
//		if (m_ref)
//			return;
//		else
//		{
//			delete m_imp;
//			m_imp = nullptr;
//		}
//	}
//};


class GeneFactoryClassBase
{
public:
	virtual ~GeneFactoryClassBase() {};
	virtual IClass* create(void* ptr, bool isRef) = 0;
};

template <typename T>
class GeneFactoryClass:public GeneFactoryClassBase
{
public:
	~GeneFactoryClass() {}; 
	virtual IClass* create(void* ptr, bool isRef) //override
	{
		TemplatorRef<T>* tempRef = new TemplatorRef<T>();
		if (!ptr)
			tempRef->m_imp = new T;
		else
		{
			tempRef->m_imp = static_cast<T*>(ptr);
			if (!(tempRef->m_imp))
			{
				throw std::runtime_error("static_cast error!");
			}
		}
		tempRef->m_ref = isRef;
		return tempRef;
	}
};



class Gene
{
public:

	IClass* m_imp;
	static std::map<std::type_index, GeneFactoryClassBase*> s_map;

	Gene();
	Gene(std::type_index id ,void* src=nullptr, bool isRef=false);
	template <typename T>
	static void enrol()
	{
		if (s_map.find(typeid(T)) == s_map.end())
			s_map[typeid(T)] = new GeneFactoryClass<T>();
	}
	template <typename T> //Gnrc Ĭ�Ϲ���(ֵ)
	Gene(const T& src) : m_imp(new Templator<T>(src)) 
	{ //enrol here
		static bool isOnce = false;
		if (isOnce)
			return;
		isOnce = true;
		enrol<T>();
	} //Gnrc::Gnrc(T src)
	template <typename T>
	Gene(const T*) = delete;
	template <typename T>
	Gene(T*) = delete;
	Gene(const Gene& lhs); //��������
	Gene& operator=(const Gene& lhs) noexcept; //������ֵ
	Gene(Gene&& rhs) noexcept;  //�ƶ�����
	Gene& operator=(Gene&& src) noexcept; //�ƶ���ֵ
	Gene* operator&(); //��һ������ȡַ����
	const Gene* operator&() const; //�Գ������ȡַ����
	bool operator==(const Gene& other) const;
	bool operator<(const Gene& other) const;
	// using namespace rel_ops; auto deduce
	bool operator!=(const Gene& other) const;
	bool operator<=(const Gene& other) const;
	bool operator>(const Gene& other) const;
	bool operator>=(const Gene& other) const;
	std::type_index get_id() const;

	~Gene(); //Gnrc::~Gnrc()


	template <typename T>
	inline bool is() const
	{
		return m_imp->_is(typeid(T));
		//Temp<T>* ptr = dynamic_cast<Temp<T>*>(m_imp);
		//return ptr != nullptr;
	}

	template <typename T>
	inline T& as()
	{
		assert(m_imp->_is(typeid(T)));
		//bool bl = (typeid(*m_imp) == typeid(Templator<T>));
		if (typeid(*m_imp) == typeid(Templator<T>))
		{
			Templator<T>* ptr = dynamic_cast<Templator<T>*>(m_imp);
			return ptr->m_imp;
		}
		else
		{
			TemplatorRef<T>* ptr = dynamic_cast<TemplatorRef<T>*>(m_imp);
			return *(ptr->m_imp);
		}
	}

	const std::type_index& _id() //compat
	{
		return m_imp->_get_id();
	}

	void* _imp() const
	{
		return (void*)m_imp;
	}
};


using GeneList = std::vector<Gene>;
using GeneDict = std::map<Gene, Gene>;
//int test_gnrc_int(int n); //for google test



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
	//double x;
	//double y;
	double transform[16];
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
		{//����������죬ָ���Ա����֧�ֿ�������
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
	//double* pvalue = NULL; // ��ʼ��Ϊ null ��ָ��
	//�� C++11 �±�׼�У�����ͨ�� =default �ؼ����������캯�������߱�����Ϊ��������һ��Ĭ�ϵİ汾���ɱ������Լ����ɵ�Ĭ�Ϲ��캯����������һ�����û��Լ�����ĸ��ã�����Ҳ���б�־�ԣ����ڴ�����Ķ���
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
	__declspec(dllimport) Vec3(const Vec3& vec) : x(vec.x), y(vec.y), z(vec.z) //��������
	{
		std::cout << "copy construct" << std::endl;
	}

	__declspec(dllimport) Vec3(Vec3&& rhs) //�ƶ�����
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

	__declspec(dllimport) Vec3 operator=(const Vec3& rhs)  //������ֵ
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

	__declspec(dllimport) Vec3 operator=(Vec3&& rhs)  //�ƶ���ֵ
	{
		this->x = rhs.x;
		this->y = rhs.y;
		this->z = rhs.z;
		std::cout << "move operator=" << std::endl;
		return *this;
	}
	/*
	����Ϊ�κ���ָ����ʽ�ƶ����캯�����ƶ���������������Ƿ����ָ���Ա��ȷʵ���ƶ����캯���������ͨ�������ĳ��ָ�����һ��ʹ�á�����ֻ����Ϊ����Щ����£���һ�ַ������Ա��������ĸ��ƿ�����

	ƽ�����͵�Ĭ���ƶ�������Ч�ڸ��Ʋ��������Ĭ���ƶ������൱�����ÿ����Ա���ƶ�������

	��ˣ�������ʾ���У��ƶ�������Ч�ڸ��Ʋ���������û�пɹ۲쵽�Ĳ��졣
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
		//ǰ��ʹ��const ��ʾ����ֵΪconst
		//����� const��ʾ�����������޸�class�ĳ�Ա
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
	struct hash<Vec3> //: public __hash_base<size_t, MyClass>   //��׼����������̳У��鿴һ����ʵֻ�Ǽ̳�����typedef���ѣ�
	{
		size_t operator()(const Vec3& rhs) const noexcept    //���const noexpectһ��Ҫд��ȥ
		{
			return (std::hash<double>()(rhs.x)) ^ (std::hash<double>()(rhs.y) << 1) ^ (std::hash<double>()(rhs.z) << 2); //��Ȼ������ʹ�������ķ�ʽ����������ϣֵ,
																							//������cppreference��������ӣ������������Ҿ��С�
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
//class Templ :public IC //ģ����
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
////��̳�
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

        //��һ����ӵ���������ϵĳ�Աʱ����Ҫдͳһ�ıȽϺ���
		int res0 = memcmp((char*)(this) + sizeof(void*), (char*)(&_r) + sizeof(void*), sizeof(size_t) + sizeof(size_t) + sizeof(unsigned char)); //v
		int res1 = memcmp((size_t*)(this) + 1, (size_t*)(&_r) + 1, sizeof(size_t) + sizeof(size_t) + sizeof(unsigned char)); 
		int res2 = memcmp(this, &_r , sizeof(void*)+ sizeof(size_t) + sizeof(size_t) + sizeof(unsigned char)); //x
        return res2==-1;
        //this �Ǵ�ʱ�������ָ�룬_r���׵�ַ����������ָ��

        if (!(m_handle1 < _r.m_handle1)) 
            return false;
        if (!(m_handle2 < _r.m_handle2))
            return false;
        return m_type < _r.m_type;


        //if (!m_handle1 < _r.m_handle1) //try_emplace���޷��ж�
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



