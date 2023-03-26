#include "pch.h"


void test_handle()
{
	BPPropertyHandle key(3, 7);
	if (key < key)
		return;

	//std::map<BPPropertyHandle, int> aMap;
	std::map<BPParaHandle, int> aMap;
	//aMap.insert({ BPPropertyHandle(3,6),1 });
	aMap.try_emplace(BPPropertyHandle(3,6),1.0);
	aMap.try_emplace(BPPropertyHandle(3,7),2);
	aMap.insert({ BPPropertyHandle(4,6),3 });

	//aMap.try_emplace(BPPropertyHandle(4,6),3);
	aMap.try_emplace(BPPropertyHandle(5,6),4);
	aMap.try_emplace(BPPropertyHandle(13,14),5);
	aMap.try_emplace(BPPropertyHandle(13,15),5);
	aMap.try_emplace(BPPropertyHandle(13,16),5);
	aMap.try_emplace(BPPropertyHandle(13,17),5);
	auto res = aMap.find(key);


	return;
}

static int enrol = []()->int
{
	test_handle();
	return 0;
}();