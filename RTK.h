#pragma once

#include "CDecode.h"
#include "Detect.h"
#include "RTKlib.h"
#include "SPP.h"
#include "SDObs.h"

struct DDObs
{
    int refSatPrn[4]{}, refSatIndex[4]{};  // 参考星的prn号, 以及在单差数组中的下标; 0:GPS 1:BDS 2:GLONASS 3:Galileo
    bool selected[4]{};  // 参考星是否选取成功
    
    int ddPrn[MAXCHANNELNUM]{};  // 双差对应的卫星prn号
    GNSS ddSys[MAXCHANNELNUM]{};  // 双差对应的卫星系统
    int ddNum{};  // 双差观测值数目
    int sysNum[4]{};  // 各卫星系统可用双差观测值数目
    double dd[MAXCHANNELNUM][4]{};  // 双差观测值; 0:L1 1:L2 2:P1 3:P2
    double fixedAmb[MAXCHANNELNUM * 4]{};  // 固定解模糊度最优和次优组合
    
    void GetDDObs(const SDObs& sdObs);  // 获取双差观测值
    
};

class RTK
{
private:
    GPSTIME t{};  // 信号发射时刻
    
    XYZ pos{};  // 流动站最终定位结果
    
    double resAmb[2]{};  // 浮点解中的模糊度残差
    double m[3]{};  // 基线分量的中误差
    double delta{};  // 标准差
    bool valid{};  // true为有解
    unsigned short sol{};  // 0:single 1:float 2:fixed
    double ratio{};  // 模糊度固定情况ratio > 3即为可用
    
    SDObs sdObs{};  // 站间单差
    DDObs ddObs{};  // 站星双差
    SPP spp[2]{};  // 不同接收机的解算(定位)结果  0:rover 1:base;
    CDetectSlip detectSlip;  // 单差观测值粗差探测
    
public:
    friend class Client;
    
    void SelectRefSat();  // 参考星选取
    int CalFixedSolution(RAWDATA& roverRaw, RAWDATA& baseRaw, EPKGFMW& rEpkGfmw, EPKGFMW& bEpkGfmw, CONFIG& config);  // 固定解解算
    
};
