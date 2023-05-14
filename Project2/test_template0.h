#pragma once

//支持泛型的模板


class Generic //__PrimitiveInterface
{
public:
	const Generic* m_imp;
	size_t m_count;

};


template<typename T>
class Gene : public Generic
{
public:
	T m_data;
	Gene(const T& data) :
		m_data(data)
	{
	}

};