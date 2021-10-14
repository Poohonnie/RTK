#pragma once


class CMatrix
{
public:
	int rows, cols;
	double* mat;

	CMatrix();//Ĭ�Ϲ��캯��
	CMatrix(double* Mat, int Rows, int Cols);//���캯��
	CMatrix(int Rows, int Cols);//ȫ0�����캯��
	CMatrix(const CMatrix&);//�������캯��
	virtual ~CMatrix();//��������
	CMatrix Eye(int n);//��λ��

	void Show();//��ԭʼ�ľ������ʾ����
	void Show(int m, int n);//����λ��;�����ʾ����
	void Write(double val, int m, int n);//д�� m �� n �е�ֵ
	double Read(int m, int n);//��ȡ m �� n �е�ֵ
	bool isSquare();//�ж��Ƿ�Ϊ����

	CMatrix operator+(const CMatrix& addMat) const;//����ӷ�
	CMatrix operator-(const CMatrix& subMat) const;//�������
	CMatrix& operator=(const CMatrix& origin);//����ֵ
	CMatrix& operator+=(const CMatrix& addMat);//+=
	CMatrix& operator-=(const CMatrix& subMat);//-=
	CMatrix operator*(const CMatrix& multiplierMat) const;//����˷�

	CMatrix Inv();//�������棬����ֵΪ�������
	CMatrix Trans();//����ת�ã�����ֵΪת�þ���

	int Row() const;//�ⲿ��������������Ŀ�Ľӿ�
	int Col() const;//�ⲿ��������������Ŀ�Ľӿ�
};

