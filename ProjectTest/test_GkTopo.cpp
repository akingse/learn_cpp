#include "pch.h"
using namespace std;

//≤‚ ‘natvisµ˜ ‘º‡ ”

namespace GeomKernel
{
    class GkMaVec
    {
    public:
        void* impl = 0;
        GkMaVec(double x, double y, double z)
        {
            double* imp = new double(3);
            imp[0] = x;
            imp[1] = y;
            imp[2] = z;
            impl = imp;
        }
        std::string to_str() const
        {
            string num;
            for (int i = 0; i < 3; i++)
            {
                ostringstream oss;
                oss << std::setprecision(numeric_limits<double>::max_digits10) << *((double*)impl + i);
                oss.clear();
                num += oss.str() + ",";
            }
            num.pop_back();
            return num;
        }
        std::string debug() const
        {
            return to_str();
        }
    };
    class GkMaPos
    {
    public:
        void* impl = 0;
        GkMaPos(double x, double y, double z)
        {
            double* imp = new double(3);
            imp[0] = x;
            imp[1] = y;
            imp[2] = z;
            impl = imp;
        }
        std::string to_str() const
        {
            string num;
            for (int i = 0; i < 3; i++)
            {
                ostringstream oss;
                oss << std::setprecision(numeric_limits<double>::max_digits10) << *((double*)impl + i);
                oss.clear();
                num += oss.str() + ",";
            }
            //double a = 0;
            //num += to_string(1.0 / a);
            //if (*(double*)impl == 1)
            //{
            //    double* p = 0;
            //    num += to_string(*p);
            //}
            num.pop_back();
            return num;
        }
        std::string debug() const
        {
            return to_str();
        }
    };


}
using namespace GeomKernel;
static void test0()
{
    GkMaVec vec2 = GkMaVec(1, exp(1), M_PI);
    GkMaVec vec3 = GkMaVec(1, 2, 3);

    GkMaPos pos3 = GkMaPos(2, 2, 3);
    string nump3 = pos3.to_str();
    //string nump2 =  GkMaPos(1, 2, 3).to_str(); //crush
    shared_ptr<GkMaPos> pvec3 = make_shared<GkMaPos>(GkMaPos(2, 3, 4));

    return;
}

static int enrol = []()->int
    {
        GkMaVec vec = GkMaVec(0, 0, 0);
        vec.debug();
        GkMaPos pos = GkMaPos(0, 0, 0);
        pos.debug();
        test0();

        cout << clash::get_filepath_filename(__FILE__) << " finished.\n" << endl;
        return 0;
    }();