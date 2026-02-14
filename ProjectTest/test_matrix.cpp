#include "pch.h"
using namespace std;
using namespace Eigen;
#define N 3    //测试矩阵维数定义

//按第一行展开计算|A|
double getDetA(double mat[N][N], int n)
{
    if (n == 1)
    {
        return mat[0][0];
    }
    double ans = 0;
    double temp[N][N] = { 0.0 };
    int i, j, k;
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n - 1; j++)
        {
            for (k = 0; k < n - 1; k++)
            {
                temp[j][k] = mat[j + 1][(k >= i) ? k + 1 : k];
            }
        }
        double t = getDetA(temp, n - 1);
        if (i % 2 == 0)
        {
            ans += mat[0][i] * t;
        }
        else
        {
            ans -= mat[0][i] * t;
        }
    }
    return ans;
}

//计算每一行每一列的每个元素所对应的余子式，组成A*
void  getAStart(double mat[N][N], int n, double ans[N][N])
{
    if (n == 1)
    {
        ans[0][0] = 1;
        return;
    }
    int i, j, k, t;
    double temp[N][N];
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            for (k = 0; k < n - 1; k++)
            {
                for (t = 0; t < n - 1; t++)
                {
                    temp[k][t] = mat[k >= i ? k + 1 : k][t >= j ? t + 1 : t];
                }
            }
            ans[j][i] = getDetA(temp, n - 1);  //此处顺便进行了转置
            if ((i + j) % 2 == 1)//奇数
            {
                ans[j][i] = -ans[j][i];
            }
        }
    }
}

//得到给定矩阵src的逆矩阵保存到des中。
bool GetMatrixInverse(double src[N][N], int n, double des[N][N])
{
    double flag = getDetA(src, n);
    double t[N][N];
    if (0 == flag)
    {
        cout << "det_A==0,there isno inverse matrix!" << endl;
        return false;//如果算出矩阵的行列式为0，则不往下进行
    }
    else
    {
        getAStart(src, n, t);
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                des[i][j] = t[i][j] / flag;
            }
        }
    }
    return true;
}

static int test0()
{
    bool flag;//标志位，如果行列式为0，则结束程序
    int row = N;
    int col = N;
    double matrix_before[N][N]{};//{1,2,3,4,5,6,7,8,9};

    //随机数据，可替换
    srand((unsigned)time(0));
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            matrix_before[i][j] = rand() % 100 * 0.01;
        }
    }

    cout << "原矩阵：" << endl;

    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            //cout << matrix_before[i][j] <<" ";
            cout << *(*(matrix_before + i) + j) << " ";
        }
        cout << endl;
    }


    double matrix_after[N][N]{};
    flag = GetMatrixInverse(matrix_before, N, matrix_after);
    if (false == flag)
        return 0;


    cout << "逆矩阵：" << endl;

    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            cout << matrix_after[i][j] << " ";
            //cout << *(*(matrix_after+i)+j)<<" ";
        }
        cout << endl;
    }

    GetMatrixInverse(matrix_after, N, matrix_before);

    cout << "反算的原矩阵：" << endl;//为了验证程序的精度
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            //cout << matrix_before[i][j] <<" ";
            cout << *(*(matrix_before + i) + j) << " ";
        }
        cout << endl;
    }


    return 0;
}

static Eigen::Matrix4d _rotate(const Eigen::Vector3d& axis = { 0, 0, 1 }, double theta = 0.0)
{
    Eigen::Vector3d nv = axis.normalized();
    double c = 1.0 - cos(theta);
    double s = sin(theta);
    Eigen::Matrix4d matA, matB, matC;
    matA <<
        0.0, -nv.z() * c, nv.y()* c, 0.0,
        nv.z()* c, 0.0, -nv.x() * c, 0.0,
        -nv.y() * c, nv.x()* c, 0.0, 0.0,
        0, 0, 0, 1;
    matB <<
        0.0, -nv.z(), nv.y(), 0.0,
        nv.z(), 0.0, -nv.x(), 0.0,
        -nv.y(), nv.x(), 0.0, 0.0,
        0, 0, 0, 1;
    matC <<
        1.0, -nv.z() * s, nv.y()* s, 0.0,
        +nv.z() * s, 1.0, -nv.x() * s, 0.0,
        -nv.y() * s, nv.x()* s, 1.0, 0.0,
        0, 0, 0, 0;
    Eigen::Matrix4d T = matA * matB + matC;
    return T;
}

static void test1()
{
    Matrix4d RE0 = eigen::rotate();
    Matrix4d RE1 = _rotate();

    Matrix4d R0 = eigen::rotate(Vector3d(0,0,1), M_PI_2);
    Matrix4d R1 = _rotate(Vector3d(0, 0, 1), M_PI_2);

    Matrix4d R2 = eigen::rotate(Vector3d(1, 1, 1), 1);
    Matrix4d R3 = _rotate(Vector3d(1, 1, 1), 1);

    Matrix4d dia = R2 - R3;

    Matrix4d mat = eigen::rotz(M_PI / 2);
    return;
}

static int enrol = []()->int
    {
        test1();
        //test2();
        //test3();
        cout << clash::get_filepath_filename(__FILE__) << " finished.\n" << endl;
        return 0;
    }();

