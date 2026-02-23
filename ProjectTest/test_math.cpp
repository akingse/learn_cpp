#include "pch.h"
using namespace std;
//https://www.cnblogs.com/xiaoxi666/p/6421228.html
//#define N 5
static constexpr int N = 5;

static int _test1()
{
    std::cout << "Hello World!\n";
    //for (int i = 0; i <= 5; i++)
    //for (int i = 0; i <= 5; ++i) //效果一样，++i效率高
    //    std::cout << i << endl;

    double a = 2;
    std::cout << a << endl;

    //a = a ^ 2;
    a = pow(a, 2);
    std::cout << a << endl;
    std::cout << M_PI << endl;
    std::cout << cos(M_PI / 2) << endl;

    vector<int> vecA;
    for (int i = 0; i < 10; i++)
        vecA.push_back(i);
    std::cout << vecA[vecA.size() - 1] << endl;
    auto it = vecA.end();
    std::cout << vecA.front() << endl;
    std::cout << vecA.back() << endl;
    //auto it = vecA.rbegin();

    complex<double> acp(2, 1);
    complex<double> ac2 = 2.0 + 3i;
    std::cout << "complex" << acp << endl;
    std::cout << "complex" << ac2 << endl;
    std::cout << " " << acp.real() << endl;
    std::cout << " " << acp.imag() << endl;

    vector<double> aList;
    aList.push_back(1);
    //aList.push_back(ac2);

    vector<complex<double>> compList;
    compList.push_back(acp);

    /*double operator^( double& a, double& b)
    {

    }*/
    return 0;
}

//矩阵乘法
double* multiply(double A[N * N], double B[N * N])
{
    double* C = new double[N * N]{};
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            for (int k = 0; k < N; k++)
            {
                C[i * N + j] += A[i * N + k] * B[k * N + j];
                if (abs(C[i * N + j]) < 1e-10)
                {
                    C[i * N + j] = 0;
                }
            }
        }
    }
    return C;
}

//LUP分解  LU Factorization
void LUP_Descomposition(double A[N * N], double L[N * N], double U[N * N], int P[N])
{
    int row = 0;
    for (int i = 0; i < N; i++)
    {
        P[i] = i;
    }
    for (int i = 0; i < N - 1; i++)
    {
        double p = 0.0;
        for (int j = i; j < N; j++)
        {
            if (abs(A[j * N + i]) > p)
            {
                p = abs(A[j * N + i]);
                row = j;
            }
        }
        if (0 == p)
        {
            cout << "singular matrixm,there is no inverse matrix!" << endl; //矩阵奇异，无法计算逆
            return;
        }

        //交换P[i]和P[row]
        int tmp = P[i];
        P[i] = P[row];
        P[row] = tmp;

        double tmp2 = 0.0;
        for (int j = 0; j < N; j++)
        {
            //交换A[i][j]和 A[row][j]
            tmp2 = A[i * N + j];
            A[i * N + j] = A[row * N + j];
            A[row * N + j] = tmp2;
        }

        //以下同LU分解
        double u = A[i * N + i], l = 0.0;
        for (int j = i + 1; j < N; j++)
        {
            l = A[j * N + i] / u;
            A[j * N + i] = l;
            for (int k = i + 1; k < N; k++)
            {
                A[j * N + k] = A[j * N + k] - A[i * N + k] * l;
            }
        }

    }

    //构造L和U
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j <= i; j++)
        {
            if (i != j)
            {
                L[i * N + j] = A[i * N + j];
            }
            else
            {
                L[i * N + j] = 1;
            }
        }
        for (int k = i; k < N; k++)
        {
            U[i * N + k] = A[i * N + k];
        }
    }

}

//LUP求解方程
double* LUP_Solve(double L[N * N], double U[N * N], int P[N], double b[N])
{
    double* x = new double[N]();
    double* y = new double[N]();

    //正向替换
    for (int i = 0; i < N; i++)
    {
        y[i] = b[P[i]];
        for (int j = 0; j < i; j++)
        {
            y[i] = y[i] - L[i * N + j] * y[j];
        }
    }
    //反向替换
    for (int i = N - 1; i >= 0; i--)
    {
        x[i] = y[i];
        for (int j = N - 1; j > i; j--)
        {
            x[i] = x[i] - U[i * N + j] * x[j];
        }
        x[i] /= U[i * N + i];
    }
    return x;
}

/*****************矩阵原地转置BEGIN********************/

/* 后继 */
int getNext(int i, int m, int n)
{
    return (i % n) * m + i / n;
}

/* 前驱 */
int getPre(int i, int m, int n)
{
    return (i % m) * n + i / m;
}

/* 处理以下标i为起点的环 */
void movedata(double* mtx, int i, int m, int n)
{
    double temp = mtx[i]; // 暂存
    int cur = i;    // 当前下标
    int pre = getPre(cur, m, n);
    while (pre != i)
    {
        mtx[cur] = mtx[pre];
        cur = pre;
        pre = getPre(cur, m, n);
    }
    mtx[cur] = temp;
}

/* 转置，即循环处理所有环 */
void transpose(double* mtx, int m, int n)
{
    for (int i = 0; i < m * n; ++i)
    {
        int next = getNext(i, m, n);
        while (next > i) // 若存在后继小于i说明重复,就不进行下去了（只有不重复时进入while循环）
            next = getNext(next, m, n);
        if (next == i)  // 处理当前环
            movedata(mtx, i, m, n);
    }
}
/*****************矩阵原地转置END********************/

//LUP求逆(将每列b求出的各列x进行组装)
double* LUP_solve_inverse(double A[N * N])
{
    //创建矩阵A的副本，注意不能直接用A计算，因为LUP分解算法已将其改变
    double* A_mirror = new double[N * N]();
    double* inv_A = new double[N * N]();//最终的逆矩阵（还需要转置）
    double* inv_A_each = new double[N]();//矩阵逆的各列
    //double *B    =new double[N*N]();
    double* b = new double[N]();//b阵为B阵的列矩阵分量

    for (int i = 0; i < N; i++)
    {
        double* L = new double[N * N]();
        double* U = new double[N * N]();
        int* P = new int[N]();

        //构造单位阵的每一列
        for (int i = 0; i < N; i++)
        {
            b[i] = 0;
        }
        b[i] = 1;

        //每次都需要重新将A复制一份
        for (int i = 0; i < N * N; i++)
        {
            A_mirror[i] = A[i];
        }

        LUP_Descomposition(A_mirror, L, U, P);

        inv_A_each = LUP_Solve(L, U, P, b);
        memcpy(inv_A + i * N, inv_A_each, N * sizeof(double));//将各列拼接起来
    }
    transpose(inv_A, N, N);//由于现在根据每列b算出的x按行存储，因此需转置

    return inv_A;
}

static void test0()
{
    double* A = new double[N * N]();
    srand((unsigned)time(0));
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            A[i * N + j] = rand() % 100 * 0.01;
        }
    }

    double* E_test = new double[N * N]();
    double* invOfA = new double[N * N]();
    invOfA = LUP_solve_inverse(A);

    E_test = multiply(A, invOfA);    //验证精确度

    cout << "矩阵A:" << endl;
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            cout << A[i * N + j] << " ";
        }
        cout << endl;
    }

    cout << "inv_A:" << endl;
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            cout << invOfA[i * N + j] << " ";
        }
        cout << endl;
    }

    cout << "E_test:" << endl;
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            cout << E_test[i * N + j] << " ";
        }
        cout << endl;
    }

    return;
}

int funtion(int a, int b)
{
    if (a == 1)
        return 1;
    else if (a == 2)
    {
        if (b == 1)
            return -1;
    }

    else
        return 3;
    return 0;
}

int math_sign(float x)
{
    if (abs(x) < 1e-10)
        return 0;
    //return (x > 0) ? 1 : -1;
    return (x > 0) ? (1) : (-1);
}

static void test1()
{
    int p = 5, b = 2;
    int c = funtion(5, b);
    //cout << c << endl;
    std::cout << pow(-27.0, 1.0 / 3) << endl;

    //&取值改值
    vector<int> numList = { 1,2,3 };
    for (auto& iter : numList) //&
    {
        iter = 2 * iter;
    }
    //cout << numList[0] << endl;
    cout << math_sign(-10) << endl;

    return;
}

static void test2()
{
    //float
    double a = 0.1;
    double b = 0.2;
    double c = a + b;

    if (c == 0.3) {
        std::cout << "Equal" << std::endl;
    }
    else {
        std::cout << "Not equal" << std::endl;
    }

    //double浮点数通过位移，改变大小，
    //std::fegetround();

    float n0 = 131071.0f;
    float n1 = 131071.1f;
    float n2 = 131071.2f;

    float m0 = 131072.0f;
    float m1 = 131072.1f;
    float m2 = 131072.2f;

    float b0 = 1310720000.0f;
    float b1 = 1310720000.1f;
    float b2 = 1310720000.2f;

    float d1 = b2 - b1;
    float d0 = b1 - b0;
    if (b0 == b1)
        return;

    return;
}

static void test3()
{

    double a1 = 1 + 1e-6;
    double a2 = 1 + 2*1e-6;
    double ad = a2 - a1;

    double b1 = 1e6 + 1e-6;
    double b2 = 1e6 + 2 * 1e-6;
    double bd = b2 - b1;
    //小数的精确位数和整数位数据大小有关
    double c = bd - ad; //1e-12
    // 必然如此，float浮点数，故名思意，2^E表示法是浮动的，
    //当整数位变大，将会占用整体的数据长度，留给小数位的数据变短，小数位将变得更不精确；
    std::vector<tuple<int, int>> test0 = { {1,2} };
    std::vector<pair<int, int>> test1 = { {1,2} };
    std::vector<array<int, 2>> test2 = { {1,2} }; //调试1层看不到


    return;
}

static void test4() //C++数学函数 
{
    //https://blog.csdn.net/weixin_33858249/article/details/87960866?spm=1001.2101.3001.6650.6&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7ERate-6-87960866-blog-128446271.235%5Ev39%5Epc_relevant_yljh&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7ERate-6-87960866-blog-128446271.235%5Ev39%5Epc_relevant_yljh&utm_relevant_index=13

    long long llmax = LLONG_MAX;
    size_t ullmax = ULLONG_MAX;
    DBL_MIN; //并不是指最小可表示的浮点数，而是最小规格化浮点值
    //浮点数的存储由：S(sign)符号位、E(exponent)指数位、M(mantissa 或significand)尾数位三个部分组成。
    INFINITY;
    NAN;
    bool is = isfinite(INFINITY);
    is = isnan(NAN);
    //如果一个浮点数中指数位部分全为0，而尾数位部分不全为0则这个浮点数称为非规格化浮点数

    int x = 0;
    //如果x是正无穷大返回1，负无穷大返回-1，否则返回0
    int isinf(x);

    //如果x是无穷大返回0
    int isfinite(x);

    //如果x是一个规格化浮点数则返回非0
    int  isnormal(x);

    //如果x是一个非法的数字返回非0
    int isnan(x);

    //如果x是负数返回非0
    int signbit(x);

    /**
    *返回浮点数的分类：
    FP_INFINITE:  x是无穷大或者无穷小
    FP_NAN：x是一个非法数字
    FP_NORMAL：x是一个规格化浮点数
    FP_SUBNORMAL：x是一个非规格化浮点数
    FP_ZERO：x是0
    */
    //int fpclassify(x);

    int y = 1;
    double res;
    res = log(x); //ln(x)
    res = log2(x); //log2(x)
    res = log10(x); //log10(x)
    res = sqrt(x);
    res = cbrt(x); //立方根
    res = hypot(x, y); //d =√x^2+y^2


    //extern double erf(double x); 误差函数
    //extern double lgamma(double x);伽玛函数
    //extern double tgamma(double x); 阶乘
    //extern double ceil(double x);
    //extern double floor(double x);
    //extern double nearbyint(double x);
    //extern double rint(double x);
    //extern long lrint(double x);
    //extern double round(double x); //四舍五入
    //extern double trunc(double x);//数字拆分
    //extern double fmod(double x, double y);//取余
    //extern double modf(double x, double p);//分解出x的整数和小数部分


}

//数学特殊数字
static void test5()
{
    ///double NaN = 0.0 / 0.0;
//puts(NaN);
//double nan = 0xFFFFFFFFFFFFFFFF;// 0x7FF0000000000000;
//if (nan == 0xFFFFFFFFFFFFFFFF)
//	puts("yes");
//	cout << nan << endl;

//float x = 0.0f / 0.0f;
//if (isnan(x))
//	puts("yes");
//cout << isnan(0.0f / 0.0f) << endl;
    double fnan = nan("0");
    double f2 = fnan * 2;


    double i = 0;
    double in = 1 / i;
    double na = 0 / i;
    cout << in << endl; //inf 
    cout << na << endl; //nan

    cout << DBL_MAX << endl;
    cout << DBL_MIN << endl;
    ;
    double n2 = 1 - in;
    //cout << 1 / i << endl; //inf 
    //cout << 0 / i << endl; //nan
    double nas = std::nan("0");
    //double in = std::asinf(0);

    //数字转字符
    //string转const char*
    float fa = 1234;
    string sn = "5678";
    cout << stoi(sn) << endl;
    cout << stod(sn) << endl;

    puts(to_string(fa).c_str());

    if (isinf(in))
        puts("is_inf");
    if (isnan(na))
        puts("is_nan");

    float zero = 0; //为什么必须定义成变量
    cout << sqrt(-1) << endl;//显示nan
    cout << 1 / zero << endl; //显示inf

    double infm = 1 / zero;
    cout << "inf 1/0=" << infm << endl;

    double constexpr nanValue = std::numeric_limits<double>::quiet_NaN();
    double constexpr infValue = std::numeric_limits<double>::infinity();
    bool is1 = isnan(nanValue);
    bool is2 = isnan(infValue);
    bool is3 = isinf(nanValue);
    bool is4 = isinf(infValue);

    static const int64_t MAX_COORD = INT64_MAX >> 2;
    double maxCoord = 100;// 1e12;// DBL_MAX;// 480104;// 151657;
    int maxAccur = std::floor(std::log10(MAX_COORD / (maxCoord * maxCoord)));


    return;
}

#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <limits>
#include <cmath>

int significant_digits(double x) 
{
    if (std::isnan(x) || std::isinf(x) || x == 0.0)
        return 0;   // 0 可认为有效位为0或1，按需修改

    // 使用足够精度格式化，强制使用定点或科学记数法均可，这里用科学记数法便于处理
    std::ostringstream oss;
    oss << std::scientific << std::setprecision(std::numeric_limits<double>::digits10) << x;
    std::string s = oss.str();

    // 找到 'e' 或 'E' 截断指数部分，只保留尾数部分（含小数点）
    size_t e_pos = s.find('e');
    if (e_pos == std::string::npos)
        e_pos = s.find('E');
    std::string mantissa = s.substr(0, e_pos);

    // 移除可能的前导 '+' 或 '-'
    if (!mantissa.empty() && (mantissa[0] == '+' || mantissa[0] == '-'))
        mantissa = mantissa.substr(1);

    // 移除小数点，得到纯数字字符串
    std::string digits;
    for (char ch : mantissa) 
    {
        if (std::isdigit(ch))
            digits.push_back(ch);
    }

    // 去除前导零
    size_t start = digits.find_first_not_of('0');
    if (start == std::string::npos)
        return 0;   // 全是零（但前面已排除0）

    // 去除尾随零（有效数字不考虑末尾无意义的零）
    size_t end = digits.find_last_not_of('0');
    if (end == std::string::npos)
        end = start; // 不会发生，因为至少有一个非零数字

    // 有效位数 = end - start + 1
    return static_cast<int>(end - start + 1);
}

//double位数
static void test6()
{
    //double value = M_PI;
    //double value = 0.03;
    double value = 0.03001;
    //double value = 0.3;
    //double value = 1e-10;
    int dig = significant_digits(value);


    std::ostringstream oss;
    // 使用科学计数法输出
    oss << std::setprecision(std::numeric_limits<double>::digits10) << value; // 精度可以根据需要调整
    std::string str = oss.str();
    size_t posE = str.find('e');
    if (posE != std::string::npos) {
        str = str.substr(0, posE); // 只获取数字部分
    }
    int count = 0;
    bool leadingZero = true;
    for (const char c : str) 
    {
        if (c == '.') 
            continue; // 跳过小数点
        if (c != '0') 
            leadingZero = false; // 第一个非零数字
        if (!leadingZero) 
            ++count; // 记数有效位数
    }


    return;
}

static int enrol = []()->int
{
    //test0();
    //test1();
    //test2(); //for funciton
    //test3();
    //test5();
    test6();
    cout << clash::get_filepath_filename(__FILE__) << " finished.\n" << endl;
    return 0;
}();

