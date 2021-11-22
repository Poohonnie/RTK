#pragma once
#include "CDecode.h"
#include "SatPositioning.hpp"
#include "CDetectOutlier.h"
#include "stdio.h"



class SPP
{
private:
	GPSTIME t;//�źŷ���ʱ��

	XYZ sttnXyz;
	BLH sttnBlh;
	double sttnClkG;
	double sttnClkB;
	double PDOP;
	double sigmaP;

	double sttnV[3];
	double sigmaV;

	int gNum;
	int bNum;

	SatPositioning satPos[MAXCHANNELNUM];

public:
	friend class Client;

	void ExtendMatB(CMatrix& B, int total);//����ƾ���B����GPS�Լ�BDS������Ŀ���������չ
	void ExtendDeltaX(CMatrix& deltaX);//��deltaX����GPS�Լ�BDS������Ŀ���������չ
	void StdPntPos(RAWDATA& raw, EPKGFMW& epkGfmw);
	void StdPntVel(RAWDATA& raw, EPKGFMW& epkGfmw);

	void check();

	SPP()
	{
		memset(this, 0, sizeof(SPP));
	}
};



