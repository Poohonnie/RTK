#pragma once

#include "lib.h"
#include "SatPos.h"
#include "Detect.h"

class SPP
{
private:
    GPSTIME t{};  // 信号发射时刻
    
    XYZ sttnXyz{};  // 测站在地心地固坐标系下的坐标
    BLH sttnBlh{};  // 测站在WGS84坐标系下的坐标
    
    double sttnClkG{};
    double sttnClkB{};
    double PDOP{};
    double sigmaP{};
    
    double sttnV[3]{};
    double sigmaV{};

    int sysNum[4]{};
    
    EpkPos epkPos;  // 卫星位置数据

public:
    friend class Client;
    
    friend class RTK;
    
    friend class SDObs;
    
    void ExtendMatB(CMatrix &B, int total) const;  // 将设计矩阵B根据GPS以及BDS卫星数目情况进行扩展
    void ExtendDeltaX(CMatrix &deltaX) const;  // 将deltaX根据GPS以及BDS卫星数目情况进行扩展
    
    int StdPntPos(RAWDATA &raw, EPKGFMW &epkGfmw, CONFIG &config);  // 单点定位
    void StdPntVel(RAWDATA &raw, EPKGFMW &epkGfmw, CONFIG &config);  // 单点测速
    
    void check();
};