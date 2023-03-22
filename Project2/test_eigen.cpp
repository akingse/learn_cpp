#include "pch.h"


int main_eg()
{
	MatrixXd m(2, 2);
	m(0, 0) = 3;
	m(1, 0) = 2.5;
	m(0, 1) = -1;
	m(1, 1) = m(1, 0) + m(0, 1);
	std::cout << "Here is the matrix m:\n" << m << std::endl;
	Vector2d v(2);
	v(0) = 4;
	v(1) = v(0) - 1;
	std::cout << "Here is the vector v:\n" << v << std::endl;
	return 0;
}

static int _enrol = []()->int {

	return 0;
}();