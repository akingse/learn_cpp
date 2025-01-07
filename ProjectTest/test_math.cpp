#include "pch.h"
using namespace std;
//https://www.cnblogs.com/xiaoxi666/p/6421228.html
//#define N 5
static constexpr int N = 5;

static int _test1()
{
    std::cout << "Hello World!\n";
    //for (int i = 0; i <= 5; i++)
    //for (int i = 0; i <= 5; ++i) //Ч��һ����++iЧ�ʸ�
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

//����˷�
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

//LUP�ֽ�  LU Factorization
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
            cout << "singular matrixm,there is no inverse matrix!" << endl; //�������죬�޷�������
            return;
        }

        //����P[i]��P[row]
        int tmp = P[i];
        P[i] = P[row];
        P[row] = tmp;

        double tmp2 = 0.0;
        for (int j = 0; j < N; j++)
        {
            //����A[i][j]�� A[row][j]
            tmp2 = A[i * N + j];
            A[i * N + j] = A[row * N + j];
            A[row * N + j] = tmp2;
        }

        //����ͬLU�ֽ�
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

    //����L��U
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

//LUP��ⷽ��
double* LUP_Solve(double L[N * N], double U[N * N], int P[N], double b[N])
{
    double* x = new double[N]();
    double* y = new double[N]();

    //�����滻
    for (int i = 0; i < N; i++)
    {
        y[i] = b[P[i]];
        for (int j = 0; j < i; j++)
        {
            y[i] = y[i] - L[i * N + j] * y[j];
        }
    }
    //�����滻
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

/*****************����ԭ��ת��BEGIN********************/

/* ��� */
int getNext(int i, int m, int n)
{
    return (i % n) * m + i / n;
}

/* ǰ�� */
int getPre(int i, int m, int n)
{
    return (i % m) * n + i / m;
}

/* �������±�iΪ���Ļ� */
void movedata(double* mtx, int i, int m, int n)
{
    double temp = mtx[i]; // �ݴ�
    int cur = i;    // ��ǰ�±�
    int pre = getPre(cur, m, n);
    while (pre != i)
    {
        mtx[cur] = mtx[pre];
        cur = pre;
        pre = getPre(cur, m, n);
    }
    mtx[cur] = temp;
}

/* ת�ã���ѭ���������л� */
void transpose(double* mtx, int m, int n)
{
    for (int i = 0; i < m * n; ++i)
    {
        int next = getNext(i, m, n);
        while (next > i) // �����ں��С��i˵���ظ�,�Ͳ�������ȥ�ˣ�ֻ�в��ظ�ʱ����whileѭ����
            next = getNext(next, m, n);
        if (next == i)  // ������ǰ��
            movedata(mtx, i, m, n);
    }
}
/*****************����ԭ��ת��END********************/

//LUP����(��ÿ��b����ĸ���x������װ)
double* LUP_solve_inverse(double A[N * N])
{
    //��������A�ĸ�����ע�ⲻ��ֱ����A���㣬��ΪLUP�ֽ��㷨�ѽ���ı�
    double* A_mirror = new double[N * N]();
    double* inv_A = new double[N * N]();//���յ�����󣨻���Ҫת�ã�
    double* inv_A_each = new double[N]();//������ĸ���
    //double *B    =new double[N*N]();
    double* b = new double[N]();//b��ΪB����о������

    for (int i = 0; i < N; i++)
    {
        double* L = new double[N * N]();
        double* U = new double[N * N]();
        int* P = new int[N]();

        //���쵥λ���ÿһ��
        for (int i = 0; i < N; i++)
        {
            b[i] = 0;
        }
        b[i] = 1;

        //ÿ�ζ���Ҫ���½�A����һ��
        for (int i = 0; i < N * N; i++)
        {
            A_mirror[i] = A[i];
        }

        LUP_Descomposition(A_mirror, L, U, P);

        inv_A_each = LUP_Solve(L, U, P, b);
        memcpy(inv_A + i * N, inv_A_each, N * sizeof(double));//������ƴ������
    }
    transpose(inv_A, N, N);//�������ڸ���ÿ��b�����x���д洢�������ת��

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

    E_test = multiply(A, invOfA);    //��֤��ȷ��

    cout << "����A:" << endl;
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

    //&ȡֵ��ֵ
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

    //double������ͨ��λ�ƣ��ı��С��
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
    //С���ľ�ȷλ��������λ���ݴ�С�й�
    double c = bd - ad; //1e-12
    // ��Ȼ��ˣ�float������������˼�⣬2^E��ʾ���Ǹ����ģ�
    //������λ��󣬽���ռ����������ݳ��ȣ�����С��λ�����ݱ�̣�С��λ����ø�����ȷ��
    std::vector<tuple<int, int>> test0 = { {1,2} };
    std::vector<pair<int, int>> test1 = { {1,2} };
    std::vector<array<int, 2>> test2 = { {1,2} }; //����1�㿴����


    return;
}

static void test4() //C++��ѧ���� 
{
    //https://blog.csdn.net/weixin_33858249/article/details/87960866?spm=1001.2101.3001.6650.6&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7ERate-6-87960866-blog-128446271.235%5Ev39%5Epc_relevant_yljh&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7ERate-6-87960866-blog-128446271.235%5Ev39%5Epc_relevant_yljh&utm_relevant_index=13

    long long llmax = LLONG_MAX;
    size_t ullmax = ULLONG_MAX;
    DBL_MIN; //������ָ��С�ɱ�ʾ�ĸ�������������С��񻯸���ֵ
    //�������Ĵ洢�ɣ�S(sign)����λ��E(exponent)ָ��λ��M(mantissa ��significand)β��λ����������ɡ�
    INFINITY;
    NAN;
    bool is = isfinite(INFINITY);
    is = isnan(NAN);
    //���һ����������ָ��λ����ȫΪ0����β��λ���ֲ�ȫΪ0�������������Ϊ�ǹ�񻯸�����

    int x = 0;
    //���x��������󷵻�1��������󷵻�-1�����򷵻�0
    int isinf(x);

    //���x������󷵻�0
    int isfinite(x);

    //���x��һ����񻯸������򷵻ط�0
    int  isnormal(x);

    //���x��һ���Ƿ������ַ��ط�0
    int isnan(x);

    //���x�Ǹ������ط�0
    int signbit(x);

    /**
    *���ظ������ķ��ࣺ
    FP_INFINITE:  x��������������С
    FP_NAN��x��һ���Ƿ�����
    FP_NORMAL��x��һ����񻯸�����
    FP_SUBNORMAL��x��һ���ǹ�񻯸�����
    FP_ZERO��x��0
    */
    //int fpclassify(x);

    int y = 1;
    double res;
    res = log(x); //ln(x)
    res = log2(x); //log2(x)
    res = log10(x); //log10(x)
    res = sqrt(x);
    res = cbrt(x); //������
    res = hypot(x, y); //d =��x^2+y^2


    //extern double erf(double x); ����
    //extern double lgamma(double x);٤�꺯��
    //extern double tgamma(double x); �׳�
    //extern double ceil(double x);
    //extern double floor(double x);
    //extern double nearbyint(double x);
    //extern double rint(double x);
    //extern long lrint(double x);
    //extern double round(double x); //��������
    //extern double trunc(double x);//���ֲ��
    //extern double fmod(double x, double y);//ȡ��
    //extern double modf(double x, double p);//�ֽ��x��������С������


}

//��ѧ��������
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

    //����ת�ַ�
    //stringתconst char*
    float fa = 1234;
    string sn = "5678";
    cout << stoi(sn) << endl;
    cout << stod(sn) << endl;

    puts(to_string(fa).c_str());

    if (isinf(in))
        puts("is_inf");
    if (isnan(na))
        puts("is_nan");

    float zero = 0; //Ϊʲô���붨��ɱ���
    cout << sqrt(-1) << endl;//��ʾnan
    cout << 1 / zero << endl; //��ʾinf

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

static int enrol = []()->int
{
    //test0();
    //test1();
    //test2(); //for funciton
    //test3();
    //test5();
    cout << "test_math finished.\n" << endl;
    return 0;
}();
