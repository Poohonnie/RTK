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

class SatPos
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
    
    static PARAS CalParas(GPSTIME t/*�����ӱ���ʱ*/, const EPHEMERIS& ephem);

    void CalSat(GPSTIME t, const  EPHEMERIS& ephem);//���������������
    void CalPosVel(const EPHEMERIS& ephem, PARAS& para);//����λ���ٶȼ���
    void ClockBias(GPSTIME t, double ek, const EPHEMERIS& ephem);//�Ӳ����
    void ClockRate(GPSTIME t, double ek, double ekDot, const EPHEMERIS& ephem);//���ټ���
    static bool Overdue(GPSTIME t, const  EPHEMERIS& ephem);//�ж������Ƿ����
 
	void CalSatE(const XYZ& rcvr/*���ջ����ĵع�����*/, CoorSys& coor/*����ϵ*/);//���Ǹ߶Ƚǵļ���
	void Hopefield(const XYZ& rcvr/*���ջ����ĵع�����*/, CoorSys& coor/*����ϵ*/);//������Hopefieldģ�͸���
 
};
