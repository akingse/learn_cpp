#pragma once

//测试 序列化和反序列化
class BPGeometricPrimitiveSer
{
public:
	bool m_hollow = false;
	size_t m_identification = 0;
	std::string m_remark = "";
	BPGeometricPrimitiveSer() = default;
	BPGeometricPrimitiveSer(bool hollow, size_t identification, const std::string& remark)
	{
		m_hollow = hollow;
		m_identification = identification;
		m_remark = remark;
	}
	BPGeometricPrimitiveSer(const BPGeometricPrimitiveSer&) = default;
	~BPGeometricPrimitiveSer() = default;

	template<class T>
	bool is() 
	{
		return true;
	}
	template<class T>
	T as()
	{
		return T;
	}
};
