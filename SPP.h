#pragma once
#include "CDecode.h"
#include "SatPos.h"
#include "CDetectOutlier.h"
#include <cstdio>

class SPP
{
private:
	GPSTIME t{};//�źŷ���ʱ��

	XYZ sttnXyz{};//��վ�ڵ��ĵع�����ϵ�µ�����
	BLH sttnBlh{};//��վ��WGS84����ϵ�µ�����

	double sttnClkG{};
	double sttnClkB{};
	double PDOP{};
	double sigmaP{};

	double sttnV[3]{};
	double sigmaV{};

	double dE{}; double dN{}; double dU{};
	int gNum{};int bNum{};

	SatPos satPos[MAXCHANNELNUM];
public:
	friend class Client;
	void ExtendMatB(CMatrix& B, int total) const;//����ƾ���B����GPS�Լ�BDS������Ŀ���������չ
	void ExtendDeltaX(CMatrix& deltaX) const;//��deltaX����GPS�Լ�BDS������Ŀ���������չ
	void StdPntPos(RAWDATA& raw, EPKGFMW& epkGfmw);//���㶨λ
	void StdPntVel(RAWDATA& raw, EPKGFMW& epkGfmw);//�������
	void CalDNEU();//�����վ��NEUϵ�µĶ�λ���

	void check();
};



