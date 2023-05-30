//#pragma once
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
	__declspec(dllexport) static size_t sm_totalCount;
	//static std::map<WrapCone, size_t> sm_coneMap;
	__declspec(dllexport) static std::unordered_map<ToGeCone::WrapCone, size_t> sm_coneMap;
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
	__declspec(dllexport) Cone();
	__declspec(dllexport) Cone(double r, double h);
	__declspec(dllexport) ~Cone();
	__declspec(dllexport) void set_h(double h);
	__declspec(dllexport) double get_h() const;

	const Primitive* m_imp;
};



