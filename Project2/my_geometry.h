#pragma once
class DpIn
{
public:
	DpIn() :
		m_funType(typeid(None)),
		m_fun(nullptr)
	{
	}
	std::type_index m_funType;
	std::string m_name;
	void* m_fun;

	template<typename T>
	bool is()
	{
		type_index tmp0 = typeid(T);
		string tmp1 = typeid(T).name();
		type_index tmp2 = typeid(None);
		string tmp3 = typeid(None).name();

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
inline bool enrol(std::string object, std::string propertyID, T t)
{
	if (t == nullptr)
		return false;
	DpIn* dp = new DpIn();
	dp->m_fun = &t;
	dp->m_funType = typeid(T);//Cube:setHigh(int)
	dp->m_name = typeid(T).name();
 	string name = typeid(T).name();//"void (__cdecl Cube::*)(int) __ptr64"
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
inline void enrol(std::string object, std::string propertyID, T* t)
{
	if (t == nullptr)
		return;
	DpIn* dp = new DpIn();
	dp->m_fun = t;
	dp->m_funType = typeid(T); //cube_setlength(class BPObject *,int)
	dp->m_name = typeid(T).name();
	string name = typeid(T).name();//"bool __cdecl(class BPObject * __ptr64,int)"
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
	virtual std::string getClassName() { return string(); };

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
		type_index a1 = typeid(m_imp); //typeid识别动态类型：指针的解引用
		type_index b1 = typeid(*m_imp);

		string a2 = typeid(m_imp).name();
		string b2 = typeid(*m_imp).name();


		Cube* cube = dynamic_cast<Cube*>(m_imp);
		string a3 = typeid(cube).name();
		string a4 = typeid(cube).raw_name();
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
