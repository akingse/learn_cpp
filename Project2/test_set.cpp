#include "pch.h"

#define _USE_MATH_DEFINES
using namespace std;


bool pnpoly(int nvert, float* vertx, float* verty, float testx, float testy)
{
    /*
    Argument	Meaning
    nvert	Number of vertices in the polygon. Whether to repeat the first vertex at the end is discussed below.
    vertx, verty	Arrays containing the x- and y-coordinates of the polygon's vertices.
    testx, testy	X- and y-coordinate of the test point.
    */
    bool c = false;
    int i, j;
    for (i = 0, j = nvert - 1; i < nvert; j = i++)
    {
        if (
            ((verty[i] > testy) != (verty[j] > testy)) &&
            (testx - vertx[i] < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) )
            )
            c = !c;
    }
    return c;
}

//inline 是加在头文件内的，用于声明「这个函数虽然定义在头文件内，但多个编译实体中只保留一份」。
//static 是加在源文件里边的，用于声明「这个函数仅用于当前源文件，其它编译实体不可用」。
static int mainstl()
{
    std::cout << "Hello World! test_set\n";
    float xList[5] = { 0, 10, 10, 0 };
    float yList[5] = { 0, 0, 10, 10 };
    int lenP = 4;
    //float x = 1, y = 1;
    float x = -1, y = 0;
    bool bl= pnpoly(lenP, xList, yList, x, y);

    //for (int i = 0; i <= 5; i++)
    //for (int i = 0; i <= 5; ++i) //效果一样，++i效率高
    //    std::cout << i << endl;
    set<string> myset = { "poly","cube" };
    myset.insert("point");
    myset.insert("line");
    myset.insert("arc");
    myset.insert("arc");
    auto a=myset.find("arc");
    auto b=myset.count("arc");
    auto c = myset.erase("line");
    std::cout << myset.count("arc") << endl;
    return 0;
}

static void _test_set()
{
    set<int> mset{ 1,2,3 };
    auto iter = mset.begin();
    //auto iter1 = iter++;
    auto iter1 = ++iter;
    auto iter2 = ++iter;

    set<int> myset;
    myset.insert(1);
    myset.insert(3);
    myset.insert(2);
    bool suc;
    auto it = myset.insert(4);
    suc = it.second;
    auto it2 = myset.erase(5);
    auto it3 = myset.find(2);

    set<int> mset1{ 1,2,3,4 };
    set<int> mset2{ 4,5,6 };
    //set里面有set_intersection（取集合交集）、
    //set_union（取集合并集）、set_difference（取集合差集）、set_symmetric_difference（取集合对称差集）等函数。
    set<int> intersection;
    set<int> convergence;
    set<int> difference;
    set_intersection(mset1.begin(), mset1.end(), mset2.begin(), mset2.end(), inserter(intersection, intersection.begin()));
    set_union(mset1.begin(), mset1.end(), mset2.begin(), mset2.end(), inserter(convergence, convergence.begin()));
    set_difference(mset1.begin(), mset1.end(), mset2.begin(), mset2.end(), inserter(difference, difference.begin()));


    return;
}

static int enrol = []()->int
{
    _test_set();
    //mainstl();
    return 0;
}();


#include <cmath>  // for std::sqrt

// 定义一个三维向量类 Vector3D
class Vector3D {
public:
    float x, y, z;

    Vector3D(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}

    // 向量的模长
    float norm() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    // 向量的规范化
    Vector3D unitize() const {
        float n = norm();
        if (n > 0) {
            return Vector3D(x / n, y / n, z / n);
        }
        else {
            return Vector3D();
        }
    }
};

// 定义一个三维矩阵类 Matrix3D
class Matrix3D {
public:
    float m[4][4];

    Matrix3D() {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m[i][j] = 0;
            }
            m[i][i] = 1;
        }
    }

    // 矩阵转置
    Matrix3D transpose() const {
        Matrix3D res;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                res.m[i][j] = m[j][i];
            }
        }
        return res;
    }

    // 矩阵乘法
    Matrix3D operator*(const Matrix3D& other) {
        Matrix3D res;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                float sum = 0;
                for (int k = 0; k < 4; ++k) {
                    sum += m[i][k] * other.m[k][j];
                }
                res.m[i][j] = sum;
            }
        }
        return res;
    }

    // 矩阵设置缩放
    void setScale(float scaleX, float scaleY, float scaleZ) {
        m[0][0] = scaleX;
        m[1][1] = scaleY;
        m[2][2] = scaleZ;
    }

    // 向量矩阵变换
    Vector3D operator*(const Vector3D& v) {
        Vector3D res;
        res.x = m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3];
        res.y = m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3];
        res.z = m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3];
        return res;
    }
};

// 定义一个矩阵转置的函数
Matrix3D trans(const Matrix3D& mat) {
    return mat.transpose();
}

// 定义一个缩放矩阵的函数
Matrix3D scale(float scaleX, float scaleY, float scaleZ) {
    Matrix3D m;
    m.setScale(scaleX, scaleY, scaleZ);
    return m;
}

// 定义一个变换矩阵的函数
Vector3D get_matrixs_position(const Matrix3D& mat) {
    // TODO: 实现 get_matrixs_position 函数
    // 该函数应该返回矩阵 mat 的位置向量
    // ...
    return {};
}

// 定义一个取矩阵 x 轴的向量的函数
Vector3D get_matrixs_axisx(const Matrix3D& mat) {
    // TODO: 实现 get_matrixs_axisx 函数
    // 该函数应该返回矩阵 mat 的 x 轴向量
    return {};
    // ...
}

// 定义一个取矩阵 y 轴的向量的函数
Vector3D get_matrixs_axisy(const Matrix3D& mat) {
    // TODO: 实现 get_matrixs_axisy 函数
    // 该函数应该返回矩阵 mat 的 y 轴向量
    return {};
    // ...
}

// 定义一个取矩阵 z 轴的向量的函数
Vector3D get_matrixs_axisz(const Matrix3D& mat) {
    // TODO: 实现 get_matrixs_axisz 函数
    // 该函数应该返回矩阵 mat 的 z 轴向量
    return {};
    // ...
}

// 定义全局常量 g_matrixO，表示原点矩阵
const Matrix3D g_matrixO;

// 定义函数 setMatrixOfCubeSize，设置矩阵的缩放
//Matrix3D setMatrixOfCubeSize(const Matrix3D& mat, float lwh, char axis, bool isForw = true) {
//    Vector3D pos = get_matrixs_position(mat);
//    if (axis == 'X') {
//        Vector3D axisx = get_matrixs_axisx(mat);
//        if (axisx.isOrigin()) {
//            return g_matrixO;
//        }
//        float x = lwh / axisx.norm();
//        Matrix3D sk = trans(pos) * (trans(-pos) * mat * scale(x, 1, 1));
//        return isForw ? sk : trans((axisx.norm() - lwh) * axisx.unitize()) * sk;
//    }
//    else if (axis == 'Y') {
//        Vector3D axisy = get_matrixs_axisy(mat);
//        if (axisy.isOrigin()) {
//            return g_matrixO;
//        }
//        float y = lwh / axisy.norm();
//        Matrix3D sk = trans(pos) * (trans(-pos) * mat * scale(1, y, 1));
//        return isForw ? sk : trans((axisy.norm() - lwh) * axisy.unitize()) * sk;
//    }
//    else if (axis == 'Z') {
//        Vector3D axisz = get_matrixs_axisz(mat);
//        if (axisz.isOrigin()) {
//            return g_matrixO;
//        }
//        float z = lwh / axisz.norm();
//        Matrix3D sk = trans(pos) * (trans(-pos) * mat * scale(1, 1, z));
//        return isForw ? sk : trans((axisz.norm() - lwh) * axisz.unitize()) * sk;
//    }
//    else {
//        return g_matrixO;
//    }
//}

