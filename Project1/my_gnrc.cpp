#include "pch.h"
using namespace std;
using namespace rel_ops;//relational operators


//shortcut key

// ctrl X = ctrl L
//全屏 SHIFT + ALT + ENTER 
// alt+o = ctrl KO



Gene::Gene() //:m_imp(nullptr)
{
	//None none;
	m_imp = new Templator<None>(None());
}

//template <typename T> //Gnrc 默认构造
//Gene::Gene(T src) : m_imp(new Temp<T>(src)) //Gnrc::Gnrc(T src)
//{
//	//m_imp = new Temp<T>(src);
//	//std::cout << "Constructor" << std::endl;
//}

std::map<std::type_index, GeneFactoryClassBase*> Gene::s_map;
Gene::Gene(std::type_index id, void* src /*= nullptr*/, bool isRef /*= false*/)
{
	auto iter = s_map.find(id);
	if (iter == s_map.end())
	{
#ifdef _DEBUG
		throw runtime_error("class without enrol");
#else
		m_imp = nullptr;
#endif // _DEBUG
	}
	else
		m_imp = iter->second->create(src, isRef);
}

Gene::Gene(const Gene& lhs) 
	//m_imp(lhs.m_imp->_constructor()) //拷贝构造
{
	if (!lhs.m_imp)
		m_imp = nullptr;
	else
		m_imp = lhs.m_imp->_constructor();
	////m_imp = new IClass;
	//auto imp = (src.m_imp);
	//m_imp = (src.m_imp); // 拷贝值

	//m_imp = new Gnrc(*(src.m_imp));
	//std::cout << "Copy Constructor" << std::endl;
}

Gene& Gene::operator=(const Gene& lhs) noexcept //拷贝赋值
{
	if (!lhs.m_imp)
	{ //兼容python类型
		m_imp = new Templator<None>(None()); //nullptr;
	}
	else if (this != &lhs)
	{
		m_imp = lhs.m_imp->_constructor();
	}
	/*if (this == &src)
		return *this;
	delete m_imp;*/
	//std::cout << "operator=(const &)" << std::endl;
	return *this;
}


Gene::Gene(Gene&& rhs) noexcept : m_imp(rhs.m_imp)  //移动构造
{
	rhs.m_imp = nullptr;
	//std::cout << "Move Constructor" << std::endl;
	//1 定义的类使用了资源并定义了移动构造函数和移动赋值运算符，
	//2 该变量即将不再使用；比如vector动态扩容
}


Gene& Gene::operator=(Gene&& src) noexcept //移动赋值
{
	if (this != &src)
	{
		delete m_imp;
		m_imp = src.m_imp;
		src.m_imp = nullptr;
		//std::cout << "operator=(const &&)" << std::endl;
	}
	return *this;


	//if (this == &src)
	//	return *this;
	//delete m_imp;
	//m_imp = src.m_imp;
	//src.m_imp = nullptr;
	//std::cout << "operator=(const &&)" << std::endl;
	//return *this;
}
Gene* Gene::operator&() //对一般对象的取址函数
{
	return this;
}
const Gene* Gene::operator&() const //对常对象的取址函数
{
	return this;
}

bool Gene::operator==(const Gene& other) const
{
	if (!this->m_imp && !other.m_imp)
		return true;
	if (!this->m_imp || !other.m_imp)
		return false;

	return this->m_imp->_is_equal(*other.m_imp);
	//return this->m_imp == other.m_imp;
}

bool Gene::operator!=(const Gene& other) const
{
	if (!this->m_imp && !other.m_imp)
		return false;
	if (!this->m_imp || !other.m_imp)
		return true;

	return !(this->m_imp->_is_equal(*other.m_imp));
}

bool Gene::operator<(const Gene& other) const
{
	if (!this->m_imp)
		return false;
	return this->m_imp->_is_less(*other.m_imp);
}

bool Gene::operator<=(const Gene& other) const
{
	if (!this->m_imp)
		return false;
	return this->m_imp->_is_less(*other.m_imp) || this->m_imp->_is_equal(*other.m_imp);
}
bool Gene::operator>(const Gene& other) const
{
	if (!this->m_imp)
		return false; //opposite to operator<
	return !(this->m_imp->_is_less(*other.m_imp) || this->m_imp->_is_equal(*other.m_imp));
}
bool Gene::operator>=(const Gene& other) const
{
	if (!this->m_imp)
		return false; //opposite to operator<
	return !(this->m_imp->_is_less(*other.m_imp));
}

std::type_index Gene::get_id() const
{
	return m_imp->_get_id();
}


Gene::~Gene() //Gnrc::~Gnrc()
{
	if (m_imp)
	{
		delete m_imp;
		m_imp = nullptr;
	}
}

