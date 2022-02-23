#pragma once
#include "lib.h"
#include "CDecode.h"
#include "CMatrix.h"
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
 
	void CalSatE(const XYZ& rcvr/*���ջ����ĵع�����*/, CoorSys& coor/*����ϵ*/);//���Ǹ߶Ƚǵļ���
	void Hopefield(const XYZ& rcvr/*���ջ����ĵع�����*/, CoorSys& coor/*����ϵ*/);//������Hopefieldģ�͸���

	SatPositioning()
	{
		memset(this, 0, sizeof(SatPositioning));
	}
};
