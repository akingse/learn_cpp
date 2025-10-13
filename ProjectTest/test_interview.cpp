#include"pch.h"
using namespace std;

static void _test0()
{

}


static int enrol = []()->int
    {
        //_test0();
        cout << clash::get_filepath_filename(__FILE__) << " finished.\n" << endl;
        return 0;
    }();

