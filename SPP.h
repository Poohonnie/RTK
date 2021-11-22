#pragma once
#include "CDecode.h"
#include "SatPositioning.hpp"
#include "CDetectOutlier.h"
#include "stdio.h"



class SPP
{
private:
	GPSTIME t;//信号发射时刻

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

	void ExtendMatB(CMatrix& B, int total);//将设计矩阵B根据GPS以及BDS卫星数目情况进行扩展
	void ExtendDeltaX(CMatrix& deltaX);//将deltaX根据GPS以及BDS卫星数目情况进行扩展
	void StdPntPos(RAWDATA& raw, EPKGFMW& epkGfmw);
	void StdPntVel(RAWDATA& raw, EPKGFMW& epkGfmw);

	void check();

	SPP()
	{
		memset(this, 0, sizeof(SPP));
	}
};



