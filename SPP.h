#pragma once
#include "CDecode.h"
#include "SatPos.h"
#include "CDetectOutlier.h"
#include <cstdio>

class SPP
{
private:
	GPSTIME t{};//信号发射时刻

	XYZ sttnXyz{};//测站在地心地固坐标系下的坐标
	BLH sttnBlh{};//测站在WGS84坐标系下的坐标

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
	void ExtendMatB(CMatrix& B, int total) const;//将设计矩阵B根据GPS以及BDS卫星数目情况进行扩展
	void ExtendDeltaX(CMatrix& deltaX) const;//将deltaX根据GPS以及BDS卫星数目情况进行扩展
	void StdPntPos(RAWDATA& raw, EPKGFMW& epkGfmw);//单点定位
	void StdPntVel(RAWDATA& raw, EPKGFMW& epkGfmw);//单点测速
	void CalDNEU();//计算测站在NEU系下的定位误差

	void check();
};



