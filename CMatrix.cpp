#include <iostream>
#include <stdlib.h>
#include <iomanip>
#include "CMatrix.h"

CMatrix::CMatrix() :
	mat(nullptr), 
	rows(0), 
	cols(0) 
{

};//默认构造函数

CMatrix::CMatrix(double* Mat, int Rows, int Cols)
{

	if (Rows && Cols)
	{
		this->rows = Rows; this->cols = Cols;
		this->mat = new double[Rows * Cols];
		for (int i = 0; i < Rows * Cols; i++)
		{
			this->mat[i] = Mat[i];
		}
	}
	else
	{
		std::cout << "矩阵维数不能为0" << std::endl;
		std::abort();
	}
};//构造函数

CMatrix::CMatrix(int Rows, int Cols)
{
	/*****************************
	* 构造一个Rows行Cols列的全0矩阵
	*****************************/
	if (Rows && Cols)
	{
		this->rows = Rows; this->cols = Cols;
		this->mat = new double[this->rows * this->cols];
		for (int i = 0; i < this->rows * this->cols; i++)
		{
			this->mat[i] = 0;
		}
	}
	else
	{
		std::cout << "矩阵维数不能为0" << std::endl;
		std::abort();
	}
}

CMatrix::CMatrix(const CMatrix& orig)
{
	/*********************************
	*			拷贝构造函数
	*********************************/
	this->cols = orig.cols;//copy列的数目
	this->rows = orig.rows;//copy行的数目
	this->mat = new double[orig.cols * orig.rows];//为CMatrix类变量中的mat数组开辟一块内存，大小与orig中的数组大小一致
	memcpy(mat, orig.mat, this->cols * this->rows * sizeof(double));//数组复制
}

CMatrix::~CMatrix()
{
	if (this->mat != nullptr)
		delete[] this->mat;
}

CMatrix CMatrix::Eye(int n)
{
	if (n)
	{
		CMatrix eye(n, n);
		for (int i = 0; i < n; i++)
			eye.mat[i * n + i] = 1;
		return eye;
	}
	else
	{
		std::cout << "矩阵维数不能为0" << std::endl;
		std::abort();
	}
}

void CMatrix::Show()
{
	for (int i = 0; i < this->rows; i++)
	{
		for (int j = 0; j < this->cols; j++)
		{
			std::cout << this->Read(i, j) << ' ';
		}
		std::cout << std::endl;
	}
}

void CMatrix::Show(int m, int n)
{
	for (int i = 0; i < this->rows; i++)
	{
		for (int j = 0; j < this->cols; j++)
		{
			std::cout << std::setw(m) << std::setiosflags(std::ios::fixed) 
				<< std::setprecision(n) << this->Read(i, j) << ' ';
		}
		std::cout << std::endl;
	}
}

void CMatrix::Write(double val, int m, int n)
{
	if (m <= this->rows && n <= this->cols && this->mat)
	{
		// m, n 均在矩阵范围内
		this->mat[m * cols + n] = val;
	}
	else
	{
		std::cout << "序号不在矩阵范围内" << std::endl;
		std::abort();
	}
}

double CMatrix::Read(int m, int n)
{
	if (m <= this->rows && n <= this->cols)
		return this->mat[m * this->cols + n];
	else
	{
		std::cout << "序号不在矩阵范围内" << std::endl;
		std::abort();
	}
}

bool CMatrix::isSquare()
{
	if (this->mat != nullptr)
	{
		if (this->rows == this->cols)
			return true;//方阵行列数相等
		else
			return false;
		return false;
	}
	else
	{
		std::cout << "矩阵为空" << std::endl;
		std::abort();
	}
}

CMatrix CMatrix::operator+(const CMatrix& addMat/*被加矩阵*/) const
{
	/*******************************
	*			矩阵加法
	*******************************/
	if (this->rows == addMat.rows && this->cols == addMat.cols)
	{
		//矩阵维数一致才可以进行加法运算
		CMatrix sumMatrix(this->rows, this->cols);//声明一个和矩阵
		for (int i = 0; i < this->rows * this->cols; i++)
		{
			sumMatrix.mat[i] = this->mat[i] + addMat.mat[i];
		}
		return sumMatrix;
	}
	else
	{
		//维数不一致直接退出程序
		std::cout << "矩阵维数不一致，无法进行加法运算。" << std::endl;
		std::abort();
	}
}

CMatrix CMatrix::operator-(const CMatrix& subMat/*减数矩阵*/) const
{
	/*******************************
	*			矩阵减法
	*******************************/
	if (this->rows == subMat.rows && this->cols == subMat.cols)
	{
		//矩阵维数一致才可以进行减法运算
		CMatrix differenceMatrix(this->rows, this->cols);//声明一个差矩阵
		for (int i = 0; i < this->rows * this->cols; i++)
		{
			differenceMatrix.mat[i] = this->mat[i] - subMat.mat[i];
		}
		return differenceMatrix;
	}
	else
	{
		//维数不一致直接退出程序
		std::cout << "矩阵维数不一致，无法进行减法运算" << std::endl;
		std::abort();
	}
}

CMatrix& CMatrix::operator=(const CMatrix& orig)
{
	/*********************************
	*			拷贝构造函数
	*********************************/
	this->cols = orig.cols;//copy列的数目
	this->rows = orig.rows;//copy行的数目
	this->mat = new double[orig.cols * orig.rows];//为CMatrix类变量中的mat数组开辟一块内存，大小与orig中的数组大小一致
	memcpy(mat, orig.mat, this->cols * this->rows * sizeof(double));//数组复制
	return *this;
}

CMatrix& CMatrix::operator+=(const CMatrix& addMat)
{
	if (this->rows == addMat.rows && this->cols == addMat.cols)
	{
		//矩阵维数一致才可以进行加法运算
		for (int i = 0; i < this->rows * this->cols; i++)
		{
			this->mat[i] += addMat.mat[i];
		}
		return *this;
	}
	else
	{
		//维数不一致直接退出程序
		std::cout << "矩阵维数不一致，无法进行加法运算。" << std::endl;
		std::abort();
	}
}

CMatrix& CMatrix::operator-=(const CMatrix& subMat)
{
	if (this->rows == subMat.rows && this->cols == subMat.cols)
	{
		//矩阵维数一致才可以进行加法运算
		for (int i = 0; i < this->rows * this->cols; i++)
		{
			this->mat[i] -= subMat.mat[i];
		}
		return *this;
	}
	else
	{
		//维数不一致直接退出程序
		std::cout << "矩阵维数不一致，无法进行减法运算。" << std::endl;
		std::abort();
	}
}

CMatrix CMatrix::operator*(const CMatrix& multiplierMat/*乘数矩阵*/) const
{
	/*******************************
	*			矩阵乘法
	*	m*n 维矩阵与 n*p 维矩阵相乘
	********************************/
	if (this->cols == multiplierMat.rows)
	{
		//确认左矩阵的列数与右矩阵的行数是否一致
		CMatrix productMatrix(this->rows, multiplierMat.cols);//声明一个积矩阵
		int m = this->rows; int n = this->cols; int p = multiplierMat.cols;
		//实际上这里循环顺序并不重要，对于一维数组来说顺序读取和抽样读取速度差异不大
		for (int i = 0; i < m; i++)
			for (int j = 0; j < n; j++)
				for (int k = 0; k < p; k++)
				{
					productMatrix.mat[i * p + k] += (this->mat[i * n + j] * multiplierMat.mat[j * p + k]);
				}
		return productMatrix;
	}
	else
	{
		//维数不一致直接退出程序
		std::cout << "矩阵维数不匹配，无法进行乘法运算" << std::endl;
		std::abort();
	}
}

CMatrix CMatrix::Inv()
{
	/***************************************************
	*				高斯约当法矩阵求逆
	*			算法我也不太懂，代码抄来的
	*	这个函数会自己判断矩阵是否可逆，不需要额外判断
	***************************************************/
	int n = this->rows;
	CMatrix invMat(n, n);
	double* a = this->mat;
	double* b = invMat.mat;
	int* is = new int[n * n];
	int* js = new int[n * n];
	int i, j, k, l, u, v;   
	double d, p;

	/* 将输入矩阵赋值给输出矩阵b，下面对b矩阵求逆，a矩阵不变 */
	memcpy(b, a, n * n * sizeof(double));
	for (k = 0; k < n; k++)
	{
		d = 0.0;
		for (i = k; i < n; i++)   /* 查找右下角方阵中主元素的位置 */
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

		if (abs(d) < 1.0E-15)
		{
			printf("Divided by 0 in MatrixInv!\n");
			std::abort();
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
			for (i = 0; i < n; i++)
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
		for (i = 0; i < n; i++)
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
		for (i = 0; i < n; i++)
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
			for (i = 0; i < n; i++)
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
		if (abs(invMat.mat[count]) < 1.0e-16)
			invMat.mat[count] = 0;
	}
	return invMat;
}

CMatrix CMatrix::Trans()
{
	/***********************
	*		矩阵转置
	*	原矩阵为 m*n 维矩阵
	*	转置后维 n*m 维矩阵
	************************/
	int m = this->rows, n = this->cols;
	CMatrix transMat(n, m);
	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
			transMat.mat[j * m + i] = this->mat[i * n + j];//原矩阵 i 行 j 列元素赋值到转置矩阵中 j 行 i 列处
	return transMat;
}

int CMatrix::Row() const
{
	/********************************
	*		外部访问行数目的接口
	********************************/
	return this->rows;
}

int CMatrix::Col() const
{
	/********************************
	*		外部访问列数目的接口
	********************************/
	return this->cols;
}