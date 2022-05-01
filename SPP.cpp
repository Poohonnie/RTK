#include <cstring>
#include "CDecode.h"
#include "SatPos.h"
#include "Detect.h"
#include "RTKLib.h"
#include "SPP.h"
#include <cmath>

void SPP::ExtendMatB(CMatrix &B, int total) const
{
    //0:GPS 1:BDS 2:GLONASS 3:Galileo
    double sysB[4][MAXCHANNELNUM]{};  // 各卫星系统在B矩阵中的系数，这里按最大的创建，后续可以从中截取
    
    //赋1,并扩展B矩阵
    int sysCount{};
    int curTotal{};
    for (int i = 0; i < 4; ++i)
    {
        if (sysNum[i])
        {
            for (int j = 0; j < sysNum[i] && j < total; j++)
                sysB[i][j + curTotal] = 1;
            B.AddCol(sysB[i], 3 + sysCount);
            curTotal += sysNum[i];
            ++sysCount;
        }
    }
}

void SPP::ExtendDeltaX(CMatrix &deltaX) const
{
    double clk[1] = {};
    for (int i = 0; i < 2; ++i)
    {
        if (!sysNum[i])
        {
            //卫星数为0，说明扩展B矩阵的时候没有考虑他
            //所以要把0钟差再作为最终结果扩展进去
            deltaX.AddRow(clk, 3 + i);
        }
    }
}

int SPP::StdPntPos(RAWDATA &raw, EPKGFMW &epkGfmw, CONFIG &config)
{
    this->t = raw.epkObs.t;
    
    int usfNum{};  // 可用卫星计数
    double arrB[MAXCHANNELNUM * 3] = {};  // 先拿出这么大来，待会再从数组里截取可用的数据下来创建矩阵
    double arrw[MAXCHANNELNUM * 1] = {};  // w矩阵
    
    CMatrix deltaX(5, 1);
    deltaX.mat[0] = 0;
    deltaX.mat[1] = 0;
    deltaX.mat[2] = 0;
    deltaX.mat[3] = 0;  // tG，GPS信号接收时刻接收机钟差
    deltaX.mat[4] = 0;  // tC，BDS信号接受时刻接收机钟差
    
    // 测站初始坐标钟差，设置为上一历元解算结果
    this->check();  // 上一历元解算结果自检
    CMatrix sttnX(5, 1);
    sttnX.mat[0] = this->sttnXyz.x;
    sttnX.mat[1] = this->sttnXyz.y;
    sttnX.mat[2] = this->sttnXyz.z;
    sttnX.mat[3] = this->sttnClkG;
    sttnX.mat[4] = this->sttnClkB;
    memset(&epkPos, 0, MAXCHANNELNUM * sizeof(SatPos));
    epkPos.satNum = raw.epkObs.satNum;  // 因为两个内容是对齐的, 所以需要循环卫星数目相同(尽管实际参与计算的卫星数并不同)
    
    int calTimes = 0;  // 迭代计数
    do
    {
        usfNum = 0;
        memset(&sysNum, 0, 4 * sizeof(int));
        
        sttnX.mat[0] += deltaX.mat[0];
        sttnX.mat[1] += deltaX.mat[1];
        sttnX.mat[2] += deltaX.mat[2];
        sttnX.mat[3] += deltaX.mat[3];
        sttnX.mat[4] += deltaX.mat[4];
        
        for (int i = 0; i < raw.epkObs.satNum; ++i)
        {
            int prn = raw.epkObs.satObs[i].prn;
            GNSS sys = raw.epkObs.satObs[i].sys;
            raw.epkObs.satObs[i].check();
            if (!raw.epkObs.satObs[i].valid)
                //检查双频观测值是否完整
                continue;
            if (!epkGfmw.gfmw[i].valid)
                continue;  // 这一历元存在粗差
            
            EPHEMERIS ephem{};
            double clk{};
            int arrNum{};
            if (raw.epkObs.satObs[i].sys == GNSS::GPS && raw.gpsEphem[prn - 1].prn == prn)
            {
                ephem = raw.gpsEphem[prn - 1];
                clk = sttnClkG;
                arrNum = 3;
            } else if (raw.epkObs.satObs[i].sys == GNSS::BDS && raw.bdsEphem[prn - 1].prn == prn)
            {
                ephem = raw.bdsEphem[prn - 1];
                clk = sttnClkB;
                arrNum = 4;
            }
            if (ephem.prn != prn/*该卫星星历不存在*/ || ephem.health/*卫星星历不健康*/)
                continue;
            
            // 将卫星定位结果存储起来
            epkPos.satPos[i].sys = sys;
            epkPos.satPos[i].prn = prn;
            
            // 计算信号发射时刻
            double transT = epkGfmw.gfmw[i].PIF / constant::c;
            
            GPSTIME ttr = raw.epkObs.t/*观测时刻*/ - transT;  // 信号发射时刻
            if (SatPos::Overdue(ttr, ephem))
                // 星历过期
                continue;
            for (int k = 0; k < 4; k++)
            {
                epkPos.satPos[i].CalSat(ttr, ephem);
                ttr = ttr - epkPos.satPos[i].clkBias;
            }
            // 计算信号传输时刻
            double deltat1 = raw.epkObs.t - ttr - clk / constant::c;
            
            // 地球自转改正前的卫星位置
            XYZ satXyzk{};
            satXyzk.x = epkPos.satPos[i].satXyz.x;
            satXyzk.y = epkPos.satPos[i].satXyz.y;
            satXyzk.z = epkPos.satPos[i].satXyz.z;
            // 地球自转改正角度
            double a = wgs84.omega * deltat1;
            // 地球自转改正后卫星位置
            epkPos.satPos[i].satXyz.x = satXyzk.x * cos(a) + satXyzk.y * sin(a);
            epkPos.satPos[i].satXyz.y = -satXyzk.x * sin(a) + satXyzk.y * cos(a);
            epkPos.satPos[i].satXyz.z = satXyzk.z;
            
            // 卫星高度角计算
            XYZ curSttn{};
            curSttn.x = sttnX.mat[0];
            curSttn.y = sttnX.mat[1];
            curSttn.z = sttnX.mat[2];
            epkPos.satPos[i].CalSatEl(curSttn, wgs84);
            if (epkPos.satPos[i].eleAngle < config.elmin
                && sqrt(curSttn.x * curSttn.x + curSttn.y * curSttn.y + curSttn.z * curSttn.z) > 1e+6)
            {
                // 高度角低于阈值的卫星不参与解算
                // xyz数值明显过小时不进行这一步判断
                epkPos.satPos[i].valid = false;
                continue;
            }
            // 对流层改正
            epkPos.satPos[i].Hopefield(curSttn, wgs84);
            epkPos.satPos[i].valid = true;  // 卫星定位数据可用
            // 卫星到接收机的几何距离
            double lx = sttnX.mat[0] - epkPos.satPos[i].satXyz.x;
            double ly = sttnX.mat[1] - epkPos.satPos[i].satXyz.y;
            double lz = sttnX.mat[2] - epkPos.satPos[i].satXyz.z;
            double range = sqrt(fabs(lx * lx + ly * ly + lz * lz));
            // B矩阵赋值
            arrB[usfNum * 3 + 0] = lx / range;
            arrB[usfNum * 3 + 1] = ly / range;
            arrB[usfNum * 3 + 2] = lz / range;
            
            // w矩阵赋值
            arrw[usfNum] = epkGfmw.gfmw[i].PIF - (range + sttnX.mat[arrNum]
                                                  - constant::c * epkPos.satPos[i].clkBias +
                                                  epkPos.satPos[i].tropDelay);
            if (ephem.satSys == GNSS::GPS)
                ++sysNum[0];
            else if (ephem.satSys == GNSS::BDS)
                ++sysNum[1];
            usfNum++;  // 可用卫星计数+1
        }
        if (usfNum < 5)  // 卫星数目过少，不进行定位解算
            return -114514;
        CMatrix B(arrB, usfNum, 3);  // B矩阵(未根据钟差系数进行扩展)
        ExtendMatB(B, usfNum);
        CMatrix w(arrw, usfNum, 1);
        CMatrix BT = B.Trans();
        CMatrix BTB(B.cols, B.cols);  // BT * B
        CMatrix BTw(B.cols, 1);  // BT * w
        BTB = BT * B;
        BTw = BT * w;
        deltaX = BTB.Inv() * BTw;  // 得到最小二乘解 不包含卫星数为0的系统
        ExtendDeltaX(deltaX);
        
        calTimes++;  // 迭代计数+1
    } while (sqrt(deltaX.mat[0] * deltaX.mat[0] + deltaX.mat[1] * deltaX.mat[1] + deltaX.mat[2] * deltaX.mat[2]) >
             1e-4 && calTimes < 10);
    if (calTimes > 9)  // 说明没有正常收敛, SPP结果不可用
        return -114514;
    
    //最终定位结果
    this->sttnXyz.x = sttnX.mat[0];
    this->sttnXyz.y = sttnX.mat[1];
    this->sttnXyz.z = sttnX.mat[2];
    
    this->sttnBlh = XYZ2BLH(sttnXyz, wgs84);
    
    //进行精度评定
    CMatrix B(arrB, usfNum, 3);  // B矩阵
    ExtendMatB(B, usfNum);
    CMatrix BTB = B.Trans() * B;
    CMatrix w(arrw, usfNum, 1);
    CMatrix V(usfNum, 1);
    V = B * deltaX - w;
    CMatrix VTV(1, 1);
    VTV = V.Trans() * V;
    this->sigmaP = sqrt(VTV.mat[0] / (usfNum - 5.0));
    CMatrix BTBinv = BTB.Inv();
    this->PDOP = sqrt(BTBinv.mat[0] + BTBinv.mat[6] + BTBinv.mat[12]);
    
    //数据存储
    this->sttnClkG = sttnX.mat[3];
    this->sttnClkB = sttnX.mat[4];
    return 0;  // 计算成功, 正常返回
}

void SPP::StdPntVel(RAWDATA &raw, EPKGFMW &epkGfmw, CONFIG &config)
{
    int usfNum = 0;  // 可用卫星计数
    double arrB[MAXCHANNELNUM * 4] = {};  // 先拿出这么大来，待会再从数组里截取可用的数据下来创建矩阵
    double arrw[MAXCHANNELNUM * 1] = {};  // w矩阵
    //测站钟速矩阵
    CMatrix sttnv(4, 1);
    sttnv.mat[0] = 0;
    sttnv.mat[1] = 0;
    sttnv.mat[2] = 0;
    sttnv.mat[3] = 0;
    
    for (int i = 0; i < raw.epkObs.satNum; ++i)
    {
        int prn = raw.epkObs.satObs[i].prn;
        raw.epkObs.satObs[i].check();
        if (!raw.epkObs.satObs[i].valid || !prn)
            //检查双频观测值是否完整
            continue;
        if (!epkGfmw.gfmw[i].valid)
            continue;  // 这一历元存在粗差
        if (epkPos.satPos[i].eleAngle < config.elmin)
            //高度角低于10度的卫星不参与解算
            continue;
        
        double lx = epkPos.satPos[i].satXyz.x - this->sttnXyz.x;
        double ly = epkPos.satPos[i].satXyz.y - this->sttnXyz.y;
        double lz = epkPos.satPos[i].satXyz.z - this->sttnXyz.z;
        double range = sqrt(fabs(lx * lx + ly * ly + lz * lz));
        double rhoDot = (lx * epkPos.satPos[i].satV[0] + ly * epkPos.satPos[i].satV[1]
                         + lz * epkPos.satPos[i].satV[2]) / range;
        
        arrB[usfNum * 4 + 0] = lx / range;
        arrB[usfNum * 4 + 1] = ly / range;
        arrB[usfNum * 4 + 2] = lz / range;
        arrB[usfNum * 4 + 3] = 1;
        
        if (raw.epkObs.satObs[i].sys == GNSS::GPS && raw.gpsEphem[prn - 1].prn == prn/*该卫星星历存在*/)
        {
            double lambda1 = constant::c / 1575.42e+6;
            //w矩阵赋值
            arrw[usfNum] = -lambda1 * raw.epkObs.satObs[i].D[0] + constant::c * epkPos.satPos[i].clkRate - rhoDot;
        } else if (raw.epkObs.satObs[i].sys == GNSS::BDS && raw.bdsEphem[prn - 1].prn == prn/*该卫星星历存在*/)
        {
            double lambda1 = constant::c / 1561.098e+6;
            //w矩阵赋值
            arrw[usfNum] = -lambda1 * raw.epkObs.satObs[i].D[0] + constant::c * epkPos.satPos[i].clkRate - rhoDot;
        }
        
        usfNum++;
    }
    if (usfNum < 5)//卫星数目过少，不进行定位解算
        return;
    CMatrix B(arrB, usfNum, 4);  // B矩阵
    CMatrix w(arrw, usfNum, 1);
    CMatrix BT = B.Trans();
    CMatrix BTB(4, 4);  // BT * B
    CMatrix BTw(4, 1);  // BT * w
    BTB = BT * B;
    BTw = BT * w;
    sttnv = BTB.Inv() * BTw;  // 得到最小二乘解
    
    //最终测速结果
    this->sttnV[0] = sttnv.mat[0];
    this->sttnV[1] = sttnv.mat[1];
    this->sttnV[2] = sttnv.mat[2];
    
    //进行精度评定
    CMatrix V(usfNum, 1);
    V = B * sttnv - w;
    CMatrix VTV(1, 1);
    VTV = V.Trans() * V;
    this->sigmaV = sqrt(VTV.mat[0] / (usfNum - 4.0));
}

void SPP::check()
{
    if (fabs(sttnBlh.H) > 1e+4)
    {
        memset(this, 0, sizeof(SPP));
    }
}
