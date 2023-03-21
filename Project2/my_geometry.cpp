#include "pch.h"

static int _enrol2=[]()->int {
	Cube cube(100, 100, 200);

	auto type = &Cube::setHigh; //(0x00007ff6c3826a64)(0x000000c87852ebb8,100)-		&cube	0x000000c87852ebb8 {m_l=100 m_w=100 m_h=200 }	Cube *
	//constexpr  long long size = long long(&Cube::setHigh);
	std::function<int(Cube*)> fp = &Cube::getHigh;
	//显示指定类型
	std::function<void(Cube*,int)> fp1 = &Cube::setHigh;
	fp1(&cube, 300);

	const std::map<std::string, std::map<std::string, DpIn*>>* pMap = &g_funMap;
	//注册函数，统一改参
	enrol("CUBE", "长", cube_setLength);
	enrol("CUBE", "宽", cube_setWidth);
	enrol("CUBE", "mf高", &Cube::setHigh);
	//enrol("CUBE", "高", cube_setHigh);

	BPGeometricPrimitive primitive(new Cube(100, 100, 200), false);
	//BPGeometricPrimitive primitive(nullptr, false);
	Cube* ptr = new Cube(100, 100, 200);
	const Cube cubec(100, 100, 200);
	int lengthc = cube.getLength();
	int length = cubec.getLength();
	auto mf4 = std::mem_fn(&Cube::setLength);
	BPObject* ptrO = ptr;
	Cube* sub_prt = dynamic_cast<Cube*>(ptrO);
	/*mf6(cube, 300); *///注意参数匹配
	type_index a1 = typeid(ptr);
	type_index a2 = typeid(Cube);
	BPObject ob = cube;
	type_index a3 = typeid(&ob);
	primitive.setPropertyValue("长", 5000);
	//primitive.setPropertyValue("mf高", 6000);
	bool res = cube_setLength(nullptr, 300);

	return 0;
}();