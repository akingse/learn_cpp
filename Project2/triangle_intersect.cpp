#include "pch.h"
using namespace std;

class TriangleVec
{
public:
	Vec3 m_p0;
	Vec3 m_p1;
	Vec3 m_p2;
};

/*
һ����˵����������������������������Ƿ��ཻ:
ͶӰ�㷨:
����ͶӰ���ۣ����Խ���ά�ռ��е������Σ�ͶӰ������һ��ƽ
���ϣ��۲���ͶӰ֮���Ƿ�����ص����Ӷ��ƶ��Ƿ����ཻ;
�����ʷ��㷨:
�����������ν��������ʷ֣��Ƚ����������εıߡ������Ƿ���
���㣬���н��㣬�����������ο����϶�Ϊ�ཻ;

*/

enum class COLLISION :int
{
	T_SEPA,
	T_INTER,
	T_TANG,

};

COLLISION is_two_triangle_intersec(const TriangleVec& triA, const TriangleVec& triB)
{
	return {};
}