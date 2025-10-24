// Generic
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

//class None
//{
//public:
//	//std::nullptr_t m_null; //�ղ���
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

	DLLEXPORT_PPC Gene();
	DLLEXPORT_PPC Gene(std::type_index id ,void* src=nullptr, bool isRef=false);
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
	DLLEXPORT_PPC Gene(const Gene& lhs); //��������
	Gene& operator=(const Gene& lhs) noexcept; //������ֵ
	DLLEXPORT_PPC Gene(Gene&& rhs) noexcept;  //�ƶ�����
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

