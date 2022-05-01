#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstring>
#include "RTKlib.h"

CMatrix::CMatrix() :
        mat(nullptr),
        rows(0),
        cols(0)
{}  // 默认构造函数

CMatrix::CMatrix(const double *arr, int Rows, int Cols)
{
    
    if (Rows && Cols)  // 行数列数均不为0
    {
        rows = Rows;
        cols = Cols;
        mat = new double[Rows * Cols]{};
        for (int i = 0; i < Rows * Cols; ++i)
            mat[i] = arr[i];
    } else
    {
        rows = 0;
        cols = 0;
        mat = nullptr;
    }
}  // 构造函数

CMatrix::CMatrix(int Rows, int Cols)
{
    /*****************************
    * 构造一个Rows行Cols列的全0矩阵
    *****************************/
    if (Rows && Cols)
    {
        rows = Rows;
        cols = Cols;
        mat = new double[rows * cols]{};
    } else
    {
        rows = 0;
        cols = 0;
        mat = nullptr;
    }
}

CMatrix::CMatrix(const CMatrix &orig)
{
    /*********************************
    *			拷贝构造函数
    *********************************/
    cols = orig.cols;  // copy列的数目
    rows = orig.rows;  // copy行的数目
    mat = new double[orig.cols * orig.rows]{};  // 为CMatrix类变量中的mat数组开辟一块内存，大小与orig中的数组大小一致
    memcpy(mat, orig.mat, cols * rows * sizeof(double));  // 数组复制
}

CMatrix::~CMatrix()
{
    delete[] mat;
}

CMatrix CMatrix::Eye(int n)
{
    if (n)
    {
        CMatrix eye(n, n);
        for (int i = 0; i < n; ++i)
            eye.Write(1, i, i);
        return eye;
    } else
    {
        double arr[1]{};
        CMatrix matrix(arr, 1, 1);
        return matrix;
    }
}

CMatrix CMatrix::Zeros(int m, int n)
{
    if (m > 0 && n > 0)
    {
        CMatrix zeros(m, n);  // 构造一个全零的矩阵
        return zeros;
    } else
    {
        CMatrix mat(1, 1);  // 如果m, n不符合要求, 则返回一个1*1的0矩阵
        return mat;
    }
}

void CMatrix::Show() const
{
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; j++)
        {
            std::cout << std::setw(9) << std::setiosflags(std::ios::fixed)
                      << std::setprecision(4) << this->Read(i, j) << ' ';
        }
        std::cout << std::endl;
    }
    printf("\n");
}

void CMatrix::Show(int m, int n) const
{
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; j++)
        {
            std::cout << std::setw(m) << std::setiosflags(std::ios::fixed)
                      << std::setprecision(n) << this->Read(i, j) << ' ';
        }
        std::cout << std::endl;
    }
    printf("\n");
}

void CMatrix::Write(double val, int m, int n) const
{
    if (m < rows && n < cols && mat != nullptr)
        // m, n 均在矩阵范围内
        this->mat[m * cols + n] = val;
    else
        std::cout << "序号不在矩阵范围内" << std::endl;
}

double CMatrix::Read(int m, int n) const
{
    if (m < rows && n < cols)
        return mat[m * cols + n];
    else
    {
        std::cout << "读取矩阵数据错误" << std::endl;
        return -114514.0;
    }
}

void CMatrix::check() const
{
    for (int i = 0; i < cols * rows; ++i)
    {
        if (fabs(mat[i]) < 1e-15)
            mat[i] = 0;
    }
}

CMatrix CMatrix::operator+(const CMatrix &addMat/*被加矩阵*/) const
{
    /*******************************
    *			矩阵加法
    *******************************/
    if (rows == addMat.rows && cols == addMat.cols)
    {
        //矩阵维数一致才可以进行加法运算
        CMatrix sumMatrix(rows, cols);  // 声明一个和矩阵
        for (int i = 0; i < rows * cols; ++i)
        {
            sumMatrix.mat[i] = mat[i] + addMat.mat[i];
        }
        return sumMatrix;
    } else
    {
        printf("Matrix addtion error.\n");
        return *this;
    }
}

CMatrix CMatrix::operator-(const CMatrix &subMat/*减数矩阵*/) const
{
    /*******************************
    *			矩阵减法
    *******************************/
    if (rows == subMat.rows && cols == subMat.cols)
    {
        //矩阵维数一致才可以进行减法运算
        CMatrix differenceMatrix(rows, cols);  // 声明一个差矩阵
        for (int i = 0; i < rows * cols; ++i)
        {
            differenceMatrix.mat[i] = mat[i] - subMat.mat[i];
        }
        return differenceMatrix;
    } else
    {
        printf("Matrix subtraction error.\n");
        return *this;
    }
}

CMatrix &CMatrix::operator=(const CMatrix &orig)
{
    if (this == &orig)  // 身份检测
        return *this;
    cols = orig.cols;  // copy列的数目
    rows = orig.rows;  // copy行的数目
    mat = new double[orig.cols * orig.rows];  // 为CMatrix类变量中的mat数组开辟一块内存，大小与orig中的数组大小一致
    memcpy(mat, orig.mat, cols * rows * sizeof(double));  // 数组复制
    return *this;
}

CMatrix &CMatrix::operator+=(const CMatrix &addMat)
{
    if (rows == addMat.rows && cols == addMat.cols)
    {
        //矩阵维数一致才可以进行加法运算
        for (int i = 0; i < rows * cols; ++i)
            mat[i] += addMat.mat[i];
    } else
        printf("Matrix addtion error.\n");
    
    return *this;
}

CMatrix &CMatrix::operator-=(const CMatrix &subMat)
{
    if (rows == subMat.rows && cols == subMat.cols)
    {
        //矩阵维数一致才可以进行减法运算
        for (int i = 0; i < rows * cols; ++i)
            mat[i] -= subMat.mat[i];
    } else
        printf("Matrix subtraction error.\n");
    
    return *this;
}

CMatrix CMatrix::operator*(const CMatrix &multiplierMat/*乘数矩阵*/) const
{
    /*******************************
    *			矩阵乘法
    *	m*n 维矩阵与 n*p 维矩阵相乘
    ********************************/
    if (cols == multiplierMat.rows)
    {
        //确认左矩阵的列数与右矩阵的行数是否一致
        int m = rows;
        int n = cols;
        int p = multiplierMat.cols;
        CMatrix productMatrix(m, p);  // 声明一个积矩阵
        //实际上这里循环顺序并不重要，对于一维数组来说顺序读取和抽样读取速度差异不大
        for (int i = 0; i < m; ++i)
            for (int j = 0; j < n; j++)
                for (int k = 0; k < p; k++)
                    productMatrix.mat[i * p + k] += (mat[i * n + j] * multiplierMat.mat[j * p + k]);
        return productMatrix;
    } else
    {
        printf("Matrix multiplication error.\n");
        return *this;
    }
}

CMatrix CMatrix::operator*(const double num) const
{
    CMatrix result(*this);
    for (int i = 0; i < result.cols * result.rows; ++i)
        result.mat[i] = num * result.mat[i];
    return result;
}

CMatrix CMatrix::Inv() const
{
    /***************************************************
    *				高斯约当法矩阵求逆
    *			算法我也不太懂，代码抄来的
    *	这个函数会自己判断矩阵是否可逆，不需要额外判断
    ***************************************************/
    int n = rows;
    CMatrix invMat(n, n);
    double *a = mat;
    double *b = invMat.mat;
    int *is = new int[n * n];
    int *js = new int[n * n];
    int i, j, k, l, u, v;
    double d, p;
    
    /* 将输入矩阵赋值给输出矩阵b，下面对b矩阵求逆，a矩阵不变 */
    memcpy(b, a, n * n * sizeof(double));
    for (k = 0; k < n; k++)
    {
        d = 0.0;
        for (i = k; i < n; ++i)   /* 查找右下角方阵中主元素的位置 */
        {
            for (j = k; j < n; j++)
            {
                l = n * i + j;
                p = fabs(b[l]);
                if (p > d)
                {
                    d = p;
                    is[k] = i;
                    js[k] = j;
                }
            }
        }
        
        if (fabs(d) < 1.0E-15)
        {
            return Eye(n);
        }
        
        if (is[k] != k)  /* 对主元素所在的行与右下角方阵的首行进行调换 */
        {
            for (j = 0; j < n; j++)
            {
                u = k * n + j;
                v = is[k] * n + j;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
        
        if (js[k] != k)  /* 对主元素所在的列与右下角方阵的首列进行调换 */
        {
            for (i = 0; i < n; ++i)
            {
                u = i * n + k;
                v = i * n + js[k];
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
        
        l = k * n + k;
        b[l] = 1.0 / b[l];  /* 初等行变换 */
        for (j = 0; j < n; j++)
        {
            if (j != k)
            {
                u = k * n + j;
                b[u] = b[u] * b[l];
            }
        }
        for (i = 0; i < n; ++i)
        {
            if (i != k)
            {
                for (j = 0; j < n; j++)
                {
                    if (j != k)
                    {
                        u = i * n + j;
                        b[u] = b[u] - b[i * n + k] * b[k * n + j];
                    }
                }
            }
        }
        for (i = 0; i < n; ++i)
        {
            if (i != k)
            {
                u = i * n + k;
                b[u] = -b[u] * b[l];
            }
        }
    }
    
    for (k = n - 1; k >= 0; k--)  /* 将上面的行列调换重新恢复 */
    {
        if (js[k] != k)
        {
            for (j = 0; j < n; j++)
            {
                u = k * n + j;
                v = js[k] * n + j;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
        if (is[k] != k)
        {
            for (i = 0; i < n; ++i)
            {
                u = i * n + k;
                v = is[k] + i * n;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
    }
    
    delete[] is;
    delete[] js;
    
    for (int count = 0; count < n * n; count++)
    {
        if (fabs(invMat.mat[count]) < 1.0e-16)
            invMat.mat[count] = 0;
    }
    return invMat;
}

CMatrix CMatrix::Trans() const
{
    /***********************
    *		矩阵转置
    *	原矩阵为 m*n 维矩阵
    *	转置后维 n*m 维矩阵
    ************************/
    int m = rows, n = cols;
    CMatrix transMat(n, m);
    for (int i = 0; i < m; ++i)
        for (int j = 0; j < n; j++)
            transMat.mat[j * m + i] = mat[i * n + j];  // 原矩阵 i 行 j 列元素赋值到转置矩阵中 j 行 i 列处
    return transMat;
}

void CMatrix::SetZero() const
{
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j)
            this->Write(0, i, j);
}

void CMatrix::AddRow(double *vec, int aimRow)
{
    if (aimRow > rows)
        //最多在数组最下面一行多扩展一行
        aimRow = rows;
    if (aimRow < 0)
        aimRow = 0;
    
    CMatrix temp(rows + 1, cols);  // 全0
    memcpy(temp.mat, mat, aimRow * cols * 8);
    for (int i = 0; i < temp.cols; ++i)
    {
        if (fabs(vec[i]) < 1e+60)
            temp.mat[aimRow * temp.cols + i] = vec[i];
        else
            temp.mat[aimRow * temp.cols + i] = 0;
    }
    memcpy(temp.mat + (aimRow + 1) * temp.cols, mat + aimRow * cols,
           (rows - aimRow) * cols * 8);  // 将剩下的数复制进来
    *this = temp;
}

void CMatrix::AddCol(double *vec, int aimCol)
{
    CMatrix temp = this->Trans();
    temp.AddRow(vec, aimCol);
    *this = temp.Trans();
}

void CMatrix::SubRow(int aimRow/*去掉第几行*/)
{
    if (rows < 1)  // 矩阵一行都没有
        return;
    if (aimRow > rows)  // 最多去掉最后一行
        aimRow = rows;
    if (aimRow < 0)  // 小于0的话改成0
        aimRow = 0;
    
    CMatrix temp(mat, rows - 1, cols);  // 先把前一部分复制过来, 后一部分下面加上去
    memcpy(temp.mat + aimRow * temp.cols, mat + (aimRow + 1) * cols,
           (rows - aimRow - 1) * cols * sizeof(double));  // 将剩下的数复制进来
    *this = temp;
}


void CMatrix::SubCol(int aimCol/*去掉第几列*/)
{
    CMatrix temp = this->Trans();
    temp.SubRow(aimCol);
    *this = temp.Trans();
}