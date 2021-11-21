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

	int gNum;
	int bNum;

public:
	friend class Client;

	void ExtendMatB(CMatrix& B, int total);//����ƾ���B����GPS�Լ�BDS������Ŀ���������չ
	void SglPntPos(RAWDATA& raw, EPKGFMW& epkGfmw);
	void check();

	SPP()
	{
		memset(this, 0, sizeof(SPP));
	}
};



