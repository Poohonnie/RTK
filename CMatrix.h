#pragma once


class CMatrix
{
public:
	int rows, cols;
	double* mat;

	CMatrix();//默认构造函数
	CMatrix(const double* Mat, int Rows, int Cols);//构造函数
	CMatrix(int Rows, int Cols);//全0矩阵构造函数
	CMatrix(const CMatrix&);//拷贝构造函数
	virtual ~CMatrix();//析构函数
	CMatrix Eye(int n);//单位阵

	void Show() const;//最原始的矩阵的显示函数
	void Show(int m, int n) const;//按照位宽和精度显示矩阵
	void Write(double val, int m, int n) const;//写入 m 行 n 列的值
	double Read(int m, int n) const;//读取 m 行 n 列的值
	bool isSquare() const;//判断是否为方阵
	void check() const;//关于+0 -0的判断

	CMatrix operator+(const CMatrix& addMat) const;//矩阵加法
	CMatrix operator-(const CMatrix& subMat) const;//矩阵减法
	CMatrix& operator=(const CMatrix& origin);//矩阵赋值
	CMatrix& operator+=(const CMatrix& addMat);//+=
	CMatrix& operator-=(const CMatrix& subMat);//-=
	CMatrix operator*(const CMatrix& multiplierMat) const;//矩阵乘法
	CMatrix operator*(double num) const;//矩阵数乘

	CMatrix Inv();//矩阵求逆，返回值为矩阵的逆
	CMatrix Trans() const;//矩阵转置，返回值为转置矩阵
	void AddRow(double *vec/*需要添加的行数组*/, int aimRow/*添加到第几行*/);//矩阵扩展，加一行
	void AddCol(double* vec, int aimCol);////矩阵扩展，加一列
};

