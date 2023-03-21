#pragma once
#pragma once
#include <map>
#include <set>
#include <queue>
#include <stack>
#include <mutex>
#include <string>
#include <vector>
#include <chrono>
#include <memory>
#include <thread>
#include <atomic>
#include <utility>
#include <cassert>
#include <sstream>
#include <fstream>
#include <iostream>
#include <typeinfo>
#include <typeindex>
#include <filesystem>
#include <functional>
#include <type_traits>
#include <unordered_map>
#include <regex>


#pragma once
namespace para
{
	class None
	{
	public:
		None() {} //:m_null(nullptr) {}
		bool operator<(const None& other) const
		{
			return false;
		}
		bool operator==(const None& other) const
		{
			return true;
		}
	};

	//custom template meta 
	template<typename T>
	struct has_less_operator
	{
		template<typename U> static void test(...) {};
		template<typename U> static auto test(int)->decltype(std::declval<const U>() < std::declval<const U>()) {};
		enum { value = std::is_same<decltype(test<T>(0)), bool > ::value };
	};

	template<typename T>
	struct has_equal_operator
	{
		template<typename U> static void test(...) {};
		template<typename U> static auto test(int)->decltype(std::declval<const U>() == std::declval<const T>()) {};
		enum { value = std::is_same<decltype(test<T>(0)), bool > ::value };
	};

	class GeneBaseClass
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
		virtual ~GeneBaseClass()
		{
		}
		virtual GeneBaseClass* _constructor() const = 0; //make_constructor
		virtual bool _is(const std::type_index& type) const = 0;
		virtual bool _is_equal(const GeneBaseClass& other) const = 0;
		virtual bool _is_less(const GeneBaseClass& other) const = 0;
		virtual std::type_index _get_id() const = 0;
		virtual void* _get_imp() const = 0;
	protected:
		virtual bool _copy_constructor(const GeneBaseClass* src) = 0;
	};

	// case 1: value
	template <typename T>
	class GeneTemplateClass :public GeneBaseClass
	{
	public:
		T m_imp;
		GeneTemplateClass(T src) : m_imp(src)
		{
		}
		~GeneTemplateClass()
		{
		}
		inline GeneBaseClass* _constructor() const override
		{
			return new GeneTemplateClass<T>(m_imp);
		}

	protected:
		bool _copy_constructor(const GeneBaseClass* src) override
		{
			if (!src->_is(typeid(T)))
				return false;
			const GeneTemplateClass<T>* ptr = dynamic_cast<const GeneTemplateClass<T>*>(src);
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

		bool _is_equal(const GeneBaseClass& other) const override
		{
			const GeneTemplateClass<T>* ptr = dynamic_cast<const GeneTemplateClass<T>*>(&other);
			if (!ptr) //(ptr == nullptr)
				return false;
			//return ptr->m_imp == this->m_imp;
			return AbleEqual<T, has_equal_operator<T>::value>::operator_equal(ptr->m_imp, this->m_imp);
		}
		bool _is_less(const GeneBaseClass& other) const override
		{
			if (!(&other))
				return false;
			const GeneTemplateClass<T>* ptr = dynamic_cast<const GeneTemplateClass<T>*>(&other);
			if (!ptr)
			{
				return typeid(*this).name() < typeid(other).name(); //compare string
				//return typeid(*this) < typeid(other); //compare hashcode
			}
			//return this->m_imp < ptr->m_imp;
			return AbleLess<T, has_less_operator<T>::value>::operator_less(this->m_imp, ptr->m_imp);
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
	class GeneTemplateClassRef :public GeneBaseClass
	{
	public:
		T* m_imp;
		bool m_ref;
		GeneTemplateClassRef() :
			m_imp(nullptr),
			m_ref(false)
		{
		}
		GeneTemplateClassRef(T* src, bool isRef = false) //: m_imp(src), m_ref(isRef)
		{
			m_ref = isRef;
			m_imp = new T(*src);
		}
		~GeneTemplateClassRef()
		{
			if (m_ref)
				return;
			else
			{
				delete m_imp;
				m_imp = nullptr;
			}
		}
		inline GeneBaseClass* _constructor() const override
		{
			return new GeneTemplateClassRef<T>(m_imp);
		}

	protected:
		bool _copy_constructor(const GeneBaseClass* src) override
		{
			if (!src->_is(typeid(T)))
				return false;
			const GeneTemplateClassRef<T>* ptr = dynamic_cast<const GeneTemplateClassRef<T>*>(src);
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

		bool _is_equal(const GeneBaseClass& other) const override
		{
			const GeneTemplateClassRef<T>* ptr = dynamic_cast<const GeneTemplateClassRef<T>*>(&other);
			if (!ptr) //(ptr == nullptr)
				return false;
			//return ptr->m_imp == this->m_imp;
			return AbleEqual<T, has_equal_operator<T>::value>::operator_equal(*ptr->m_imp, *this->m_imp);
		}
		bool _is_less(const GeneBaseClass& other) const override
		{
			if (!(&other))
				return false;
			const GeneTemplateClassRef<T>* ptr = dynamic_cast<const GeneTemplateClassRef<T>*>(&other);
			if (!ptr)
			{
				return typeid(*this).name() < typeid(other).name(); //compare string
				//return typeid(*this) < typeid(other); //compare hashcode
			}
			//return this->m_imp < ptr->m_imp;
			return AbleLess<T, has_less_operator<T>::value>::operator_less(*this->m_imp, *ptr->m_imp);
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


	class GeneFactoryClassBase
	{
	public:
		virtual ~GeneFactoryClassBase() {};
		virtual GeneBaseClass* create(void* ptr, bool isRef) = 0;
	};

	template<typename T>
	class GeneFactoryClass :public GeneFactoryClassBase
	{
	public:
		~GeneFactoryClass() {};
		virtual GeneBaseClass* create(void* ptr, bool isRef)
		{
			GeneTemplateClassRef<T>* tempRef = new GeneTemplateClassRef<T>();
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
		GeneBaseClass* m_imp;
		static std::map<std::type_index, GeneFactoryClassBase*> s_map;
		__declspec(dllimport) static std::map<std::type_index, GeneFactoryClassBase*>& _getFactoryMap();


	public:
		template<typename T>
		static void enrol()
		{
			if (_getFactoryMap().find(typeid(T)) == _getFactoryMap().end())//avoid mem-leak
			{
				_getFactoryMap()[typeid(T)] = new GeneFactoryClass<T>;
			}
		}
		__declspec(dllimport) Gnrc();
		__declspec(dllimport) Gnrc(std::type_index id, void* src = nullptr, bool isRef = false);
		template <typename T> //
		Gnrc(const T& src) : m_imp(new GeneTemplateClass<T>(src))
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
		__declspec(dllimport) const std::type_index& _id() const; //compat
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
	};

	using GnrcList = std::vector<Gnrc>;
	using GnrcDict = std::map<Gnrc, Gnrc>;
}
