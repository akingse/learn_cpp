#include "pch.h"
using namespace std;


ToGeCone::WrapCone::WrapCone(const Primitive* ptr) :
	m_imp(dynamic_cast<const ToGeCone*>(ptr))
{}
ToGeCone::WrapCone::WrapCone(const WrapCone& rhs) : m_imp(rhs.m_imp) //copy
{
	//using baseclass copy constructor
}
ToGeCone::WrapCone::WrapCone(WrapCone&& rhs) : m_imp(rhs.m_imp) //move
{
	rhs.m_imp = nullptr;
}
ToGeCone::WrapCone::~WrapCone() {}

bool ToGeCone::WrapCone::operator==(const ToGeCone::WrapCone& wrap)const
{
	return wrap.m_imp->m_r == m_imp->m_r && wrap.m_imp->m_h == m_imp->m_h;
}


const ToGeCone* ToGeCone::create(double r, double h)
{
	ToGeCone* ptr = new ToGeCone;
	ptr->m_r = r;
	ptr->m_h = h;
	WrapCone wrap(ptr);
	auto iter = sm_coneMap.find(wrap);
	if (iter != sm_coneMap.end()) //model in map
	{
		sm_coneMap[wrap]++;
		delete ptr;
		ptr = nullptr;
		return iter->first.m_imp;
	}
	else //new model
	{
		sm_coneMap[wrap] = 1; //insert to map
		sm_totalCount++; //add total count
		return wrap.m_imp;
	}
}
void ToGeCone::countIncrease(const Primitive* imp)
{
	sm_coneMap[WrapCone(imp)]++;
}
void ToGeCone::countDecrease(const Primitive* imp) //while count==0 delete
{
	sm_coneMap[WrapCone(imp)]--;
	auto iter = sm_coneMap.find(imp);
	if (iter == sm_coneMap.end()) //imposible
		return;
	if (sm_coneMap[WrapCone(imp)] <1 && sm_totalCount > sizeMax)
	{
		delete iter->first.m_imp;
		sm_coneMap.erase(iter);
	}

}
const Primitive* ToGeCone::set_h(const Primitive* ptr, double h)
{
	const ToGeCone* cone = dynamic_cast<const ToGeCone*>(ptr);
	return create(cone->m_r, h);
}

const double ToGeCone::get_h(const Primitive* ptr)
{
	const ToGeCone* cone = dynamic_cast<const ToGeCone*>(ptr);
	return cone->m_h;
}



//std::map<ToGeCone::WrapCone, size_t> ToGeCone::sm_coneMap;
std::unordered_map<ToGeCone::WrapCone, size_t> ToGeCone::sm_coneMap;
size_t ToGeCone::sm_totalCount = 0;



Cone::Cone() :
	m_imp(ToGeCone::create(0, 0))
{
	//ToGeCone::countIncrease(m_imp);
}
Cone::Cone(double r, double h) :
	m_imp(ToGeCone::create(r, h))
{
	//ToGeCone::countIncrease(m_imp);
}
Cone::~Cone()
{
	ToGeCone::countDecrease(m_imp);
}
void Cone::set_h(double h)
{
	ToGeCone::countDecrease(m_imp);
	const Primitive* primitive = ToGeCone::set_h(m_imp, h);
	m_imp = primitive;
}
double Cone::get_h() const
{
	return ToGeCone::get_h(m_imp);
}