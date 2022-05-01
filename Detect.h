#pragma once

#include "RTKLib.h"

struct GFMW
{
	GNSS sys{};
	unsigned short prn{};
	double LMW{};
	double LGF{};
	double LIF{};
	double PIF{};
	int n{};  // 平滑历元数，1表示第一个历元

	bool valid{};
};

struct EPKGFMW
{
	GFMW gfmw[MAXCHANNELNUM];  // 单个历元所有GFMW组合观测值
	int FindSatObsIndex(int prn, GNSS sys);  // 搜索某个prn号的卫星在epkGfmw中的下标
};

class CDetectOutlier
{
protected:
    EPKGFMW lastEpk{};  // 上一历元GF和MW组合
    EPKGFMW curEpk{};  // 当前历元GF和MW组合

public:
    friend class Client;
    friend class SPP;
    
    void DetectOutlier(RAWDATA& raw);  // 粗差探测
};

