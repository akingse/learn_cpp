# include "pch.h"
using namespace std;
double f(double x);
double f(double x)
{
    return (2 * x * x * x - 4 * x * x + 3 * x - 6); // or 2 * pow(x, 3.0) - 4 * pow(x, 2.0) + 3 * x - 6
}
const double Accu = 1e-8;//pow(10.0, -5.0); // define accuracy

int test_alg()
{
    double a, b, c;
    a = b = c = 0;
    do
    {
        cout << "Enter a range: ";
        cin >> a >> b;
    } while (!cin || f(a) * f(b) >= 0); // bad input or f(a) * f(b) >= 0

    do
    {
        c = (a + b) / 2.0; // c is the average number of a and b
        if (f(a) * f(c) < 0)
            b = c;
        else if (f(b) * f(c) < 0)
            a = c;
        else
            break;
    } while (abs(f(c)) > Accu); // if deviation is smaller than pow(10.0, -5.0)

    cout << "Solution: " << c;
    return 0;
}


static int enrol_test_alg = []()->int
{
    //test_alg();
    return 0;
}();
