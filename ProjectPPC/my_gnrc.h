// Generic
//using namespace rel_ops;

class IClass  //虚基类
{
public:
	virtual ~IClass()
	{
		//基类中的析构函数必须为虚函数，否则会出现对象释放错误
		//std::cout << "IClass destruct" << std::endl;
	}
	virtual IClass* _constructor() const = 0; //make_constructor构造
	virtual bool _is(const std::type_index& type) const = 0;
	virtual bool _is_equal(const IClass& other) const = 0;
	virtual bool _is_less(const IClass& other) const = 0;
	virtual std::type_index _get_id() const = 0;
protected:
	virtual bool _copy_constructor(const IClass* src) = 0; //_deep_copy_from拷贝构造
	//virtual bool _operator_eq_copy(const IClass& src) = 0; //赋值
	//virtual bool _move_constructor(const IClass* src) = 0; //移动构造
	//virtual bool _operator_eq_rr(const IClass&& src) = 0; //移动赋值
	//virtual bool operator==(const IClass& p) const = 0;
	//virtual bool operator!=(const IClass& p) const = 0;
	//virtual bool operator<(const IClass& p) const = 0;
	//virtual bool operator>(const IClass& p) const = 0;
	//virtual bool operator>=(const IClass& p) const = 0;
	//virtual bool operator<=(const IClass& p) const = 0;
	//virtual std::type_index _type() const = 0;
};


//c++在写模版函数时（template<class T>之类的），头文件不能与cpp文件分离。
//这就意味者，你头文件定义的含模版的地方必须在头文件中实现，没用模版定义的地方可以放在cpp中实现。
template <typename T>
class Templator :public IClass //模板类
{
public:
	T m_imp;
	Templator(T src) : m_imp(src)
	{
		//1 T的拷贝构造必须在 Temp实现
		//2 Gnrc无法调用子类 操作函数，只能调用父类虚接口，以继承子类的实现
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
		//return ptr->m_imp == this->m_imp;
		return false;
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
		//return this->m_imp < ptr->m_imp;
		//
		return false;
	}

	inline std::type_index _get_id() const
	{
		return typeid(T);
	}
};


template <typename T>
class TemplatorRef :public IClass //引用模板类
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
		/*else if (!m_ref &&!m_imp) //delete只删除指针指向的内存空间-值
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

//class None
//{
//public:
//	//std::nullptr_t m_null; //空参类
//	None() {}; // : m_null(nullptr) {}
//	bool operator<(const None& other) const
//	{
//		return false;
//	}
//	bool operator==(const None& other) const
//	{
//		return true;
//	}
//};


////临时Gnrc，指针拷贝
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

	DLLEXPORT_PPC Gene();
	DLLEXPORT_PPC Gene(std::type_index id ,void* src=nullptr, bool isRef=false);
	template <typename T>
	static void enrol()
	{
		if (s_map.find(typeid(T)) == s_map.end())
			s_map[typeid(T)] = new GeneFactoryClass<T>();
	}
	template <typename T> //Gnrc 默认构造(值)
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
	DLLEXPORT_PPC Gene(const Gene& lhs); //拷贝构造
	Gene& operator=(const Gene& lhs) noexcept; //拷贝赋值
	DLLEXPORT_PPC Gene(Gene&& rhs) noexcept;  //移动构造
	Gene& operator=(Gene&& src) noexcept; //移动赋值
	Gene* operator&(); //对一般对象的取址函数
	const Gene* operator&() const; //对常对象的取址函数
	bool operator==(const Gene& other) const;
	bool operator<(const Gene& other) const;
	// using namespace rel_ops; auto deduce
	bool operator!=(const Gene& other) const;
	bool operator<=(const Gene& other) const;
	bool operator>(const Gene& other) const;
	bool operator>=(const Gene& other) const;
	std::type_index get_id() const;

	DLLEXPORT_PPC ~Gene(); //Gnrc::~Gnrc()


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

	inline const std::type_index& _id() //compat
	{
		return m_imp->_get_id();
	}

	inline void* _imp() const
	{
		return (void*)m_imp;
	}
};


using GeneList = std::vector<Gene>;
using GeneDict = std::map<Gene, Gene>;
//int test_gnrc_int(int n); //for google test

