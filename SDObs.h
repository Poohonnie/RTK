#pragma once

#include "RTKLib.h"
#include "SPP.h"

struct SatSD  // 单个卫星单差观测值
{
    GNSS sys{};  // 卫星系统
    unsigned short prn{};  // 卫星号
    double psrSD[2]{};  // 双频伪距单差
    double cpSD[2]{};  // 双频载波相位单差
    unsigned short rId{}, bId{};  // 流动站和基站卫星观测值索引号
    
    bool valid{};  // 观测值可用性标志
};

struct SDObs  // 站间单差
{
    GPSTIME t{};  // 时间
    unsigned short sdNum{};  // 单差观测值数目
    SatSD satSd[MAXCHANNELNUM]{};  // 卫星单差观测值数组
    
    void GetSDObs(EPKOBS& roverObs, EPKOBS& baseObs, SPP& roverSpp, SPP& baseSpp);  // 单差求解
};

class CDetectSlip
{
protected:
    EPKGFMW lastEpk{};  // 上一历元GF和MW组合
    EPKGFMW curEpk{};  // 当前历元GF和MW组合

public:
    friend class Client;
    friend class SPP;
    
    void DetectCycleSlip(SDObs& sd);  // 周跳探测
};