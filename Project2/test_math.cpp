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

static int test0()
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

    return 0;
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

static int test1()
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

    return 0;
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

    return;
}

static int enrol = []()->int
{
    //test0();
    //test1();
    test2(); //for funciton
    //test3();
    cout << "test_math finished.\n" << endl;
    return 0;
}();

