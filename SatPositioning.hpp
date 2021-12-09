#pragma once
#include "Constant.h"
#include "CDecode.h"
#include "CMatrix.h"
#include <cmath>
#include <iostream>

struct PARAS
{
	double A{};//���������
	double n0{};//ƽ���˶����ٶ�
	double tk{};//����������ο���Ԫ��ʱ��
	double n{};//��ƽ���˶����ٶȽ��и���
	double mk{};//ƽ�����
	double ek{};//ƫ����ǣ��������
	double vk{};//������
	double phik{};//�����Ǿ�

	//���׵��͸�����
	double deltaUk{};//���������Ǿ�ĸ�����
	double deltaRk{};//�����򾶵ĸ�����
	double deltaIk{};//��������Ǹ�֤��

	//���㾭�������������Ǿ࣬�򾶺͹�����
	double uk{};//�������������Ǿ�
	double rk{};//����������
	double ik{};//�������Ĺ�����
	double omegak{};//������������㾭��
	double xy0[2]{};//�����ڹ��ƽ���ϵ�λ��


	//���α�����ʱ��ĵ���
	double mkDot{};
	double ekDot{};
	double vkDot{};
	double phikDot{};

	double deltaUkDot{};
	double deltaRkDot{};
	double deltaIkDot{};

	double omegakDot{};
	double ikDot{};
	double rkDot{};
	double ukDot{};

	//�����ڹ��ƽ���ڵ��ٶ�
	double xkDot{};
	double ykDot{};
	
	PARAS()
	{
		memset(this, 0, sizeof(PARAS));
	}
};

class SatPositioning
{
protected:
	XYZ satXyz{};//�����ڵ��ĵع�����ϵ�µ�����
	double satV[3]{};//�����˶��ٶ�
	double clkBias{};//�����Ӳ�
	double clkRate{};//��������
	double eleAngle{};//���Ǹ߶Ƚ�
	double tropDelay{};//�������ӳ�
	
public:
	friend class SPP;
	static PARAS CalculateParas(GPSTIME t/*�����ӱ���ʱ*/, const GPSEPHEM& gpsEphem);
	static PARAS CalculateParas(BDSTIME t/*�����ӱ���ʱ*/, const BDSEPHEM& bdsEphem);

	void CalGps(GPSTIME t/*�����ӱ���ʱ*/, const GPSEPHEM& gpsEphem);
	void GpsPosVel(GPSTIME t/*�����ӱ���ʱ*/, const GPSEPHEM& gpsEphem, PARAS& gPara);//GPS����λ�ü���
	void GpsClockBias(GPSTIME t/*�����ӱ���ʱ*/, double ek, const GPSEPHEM& gpsEphem);//GPS�Ӳ����
	void GpsClockRate(GPSTIME t/*�����ӱ���ʱ*/, double ek, double ekDot, const GPSEPHEM& gpsEphem);//GPS���ټ���
	static bool GpsOod(GPSTIME t, const GPSEPHEM& gpsEphem);//�ж������������

	void CalBds(GPSTIME t/*�����ӱ���ʱ*/, const BDSEPHEM& bdsEphem);
	void BdsPosVel(BDSTIME t/*�����ӱ���ʱ*/, const BDSEPHEM& bdsEphem, PARAS& bPara);//BDS����λ�ü���
	void BdsClockBias(BDSTIME t/*�����ӱ���ʱ*/, double ek, const BDSEPHEM& bdsEphem);//BDS�Ӳ����
	void BdsClockRate(BDSTIME t/*�����ӱ���ʱ*/, double ek, double ekDot, const BDSEPHEM& bdsEphem);//BDS���ټ���
	static bool BdsOod(GPSTIME t, const BDSEPHEM& bdsEphem);//�ж������������

	template<typename T>
	void CalSatE(const XYZ& rcvr/*���ջ����ĵع�����*/, T& coor/*����ϵ*/);//���Ǹ߶Ƚǵļ���
	template<typename T1>
	void Hopefield(const XYZ& rcvr/*���ջ����ĵع�����*/, T1& coor/*����ϵ*/);//������Hopefieldģ�͸���

	SatPositioning()
	{
		memset(this, 0, sizeof(SatPositioning));
	}
};

template<typename T>
void SatPositioning::CalSatE(const XYZ& rcvrXyz, T& coor)
{
	//���Ǹ߶ȽǼ���

	BLH rcvrBlh = XYZ2BLH(rcvrXyz, coor);//���ջ��Ĵ������
	CMatrix trans(3, 3);//ת������
	CMatrix line(3, 1);//���ջ������ǵ�����
	CMatrix satxyz(3, 1);//������վ������ϵ�е�����

	trans.mat[0] = -sin(rcvrBlh.B) * cos(rcvrBlh.L);
	trans.mat[1] = -sin(rcvrBlh.B) * sin(rcvrBlh.L);
	trans.mat[2] = cos(rcvrBlh.B);
	trans.mat[3] = -sin(rcvrBlh.L);
	trans.mat[4] = cos(rcvrBlh.L);
	trans.mat[5] = 0;
	trans.mat[6] = cos(rcvrBlh.B) * cos(rcvrBlh.L);
	trans.mat[7] = cos(rcvrBlh.B) * sin(rcvrBlh.L);
	trans.mat[8] = sin(rcvrBlh.B);
	trans.check();//B=0 L=0 H=0�����

	line.mat[0] = this->satXyz.x - rcvrXyz.x;
	line.mat[1] = this->satXyz.y - rcvrXyz.y;
	line.mat[2] = this->satXyz.z - rcvrXyz.z;

	satxyz = trans * line;//������վ������ϵ�е�����

	this->eleAngle = atan(satxyz.mat[2] / sqrt(satxyz.mat[0] * satxyz.mat[0] + satxyz.mat[1] * satxyz.mat[1]));//���Ǹ߶Ƚ�
}

template<typename T1>
void SatPositioning::Hopefield(const XYZ& rcvrXyz, T1& coor)
{
	BLH rcvrBlh = XYZ2BLH(rcvrXyz, coor);
	double H = rcvrBlh.H;//���ջ���ظߣ����������д
	if (H > 1e+4)
	{
		this->tropDelay = 10;
		return ;//��վ�߶Ȳ��ڶ����㷶Χ
	}
	double H0 = 0.0;//m ��ƽ��
	double T0 = 20.0 + 273.16;//K �¶�
	double p0 = 1013.25;//mbar ��ѹ
	double RH0 = 0.5;//���ʪ��

	double RH = RH0 * exp(-0.0006369 * (H - H0));
	double p = p0 * pow(1 - 2.26e-5 * (H - H0), 5.225);
	double T = T0 - 0.0065 * (H - H0);
	double e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
	double hw = 11000.0;
	double hd = 40136.0 + 148.72 * (T0 - 273.16);
	double Kw = 155.2e-7 * 4810.0 / (T * T) * e * (hw - H);
	double Kd = 155.2e-7 * p / T * (hd - H);

	double E = this->eleAngle * 180.0 / constant::pi;
	double deltaD = Kd / sin(sqrt(E * E + 6.25) / 180.0 * constant::pi);
	double deltaW = Kw / sin(sqrt(E * E + 2.25) / 180.0 * constant::pi);
	this->tropDelay = deltaD + deltaW;
}
