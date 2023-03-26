#include "pch.h"
#include <iostream>
#include <boost/version.hpp>
#include <boost/config.hpp>

using namespace std;
int test_boost()
{
	cout << BOOST_VERSION << endl;
	cout << BOOST_LIB_VERSION << endl;
	cout << BOOST_PLATFORM << endl;
	cout << BOOST_COMPILER << endl;
	cout << BOOST_STDLIB << endl;
	//getchar();
	return 0;
}

#include <iostream>
#include <vector>
#include <iterator>
#include <algorithm>

#include <boost/timer.hpp>
#include <boost/progress.hpp>

#include <libs/date_time/src/gregorian/greg_names.hpp>
#include <libs/date_time/src/gregorian/greg_month.cpp>
#include <libs/date_time/src/gregorian/gregorian_types.cpp>

#include <boost/date_time/posix_time/posix_time.hpp>

using namespace boost;

int test_boost1()
{
	boost::timer t;

	boost::progress_display pd(100);

	for (int i = 0; i < 100; ++i) //进度条
	{
		++pd;
	}

	boost::gregorian::date dt(2009, 12, 8); //date_time 库
	assert(dt.year() == 2009);
	assert(dt.day() == 8);
	boost::gregorian::date::ymd_type ymd = dt.year_month_day();
	std::cout << "\n" << ymd.year << "/" << ymd.month << "/" << ymd.day << " the day is "
		<< dt.day_of_year() << " days of this year" << std::endl;

	std::cout << boost::gregorian::to_iso_extended_string(dt) << std::endl; //转换为其他格式
	std::cout << boost::gregorian::to_iso_string(dt) << std::endl;
	std::cout << boost::gregorian::to_simple_string(dt) << std::endl << std::endl;

	//对数组排序操作
	std::vector<int> test_vc(100);
	std::vector<int>::iterator beg_it = test_vc.begin();
	std::vector<int>::iterator end_it = test_vc.end();
	std::srand(std::time(NULL));

	std::for_each(beg_it, end_it, [](int& n) {n = rand(); });
	std::copy(beg_it, end_it, std::ostream_iterator<int>(std::cout, " "));
	std::cout << std::endl << std::endl;
	std::sort(beg_it, end_it, std::greater<int>());
	std::copy(beg_it, end_it, std::ostream_iterator<int>(std::cout, " "));
	std::cout << std::endl << std::endl;

	boost::posix_time::ptime pt(boost::gregorian::date(2005, 2, 6));

	std::cout << t.elapsed() << "s" << std::endl; //程序运行时间

	return 0;
}

static int enrol = []()->int
{
	test_boost();
	test_boost1();
	return 0;
}();