#include "SDObs.h"
#include <cmath>

void SDObs::GetSDObs(EPKOBS &roverObs, EPKOBS &baseObs, SPP& roverSpp, SPP& baseSpp)
{
    // 清空上一历元单差观测值
    memset(this, 0, sizeof(SDObs));
    
    // 以流动站Rover为准，基准站base数据向流动站对齐
    t = roverObs.t;
    int sdi{};  // 单差数组下标
    for(int i = 0; i < roverObs.satNum; ++i)
    {
        if(!roverObs.satObs[i].check())
            // 流动站第i颗卫星观测值数据不可用
            continue;
        int prn = roverObs.satObs[i].prn;
        GNSS sys = roverObs.satObs[i].sys;
        int bi = baseObs.FindSatObsIndex(prn, sys);  // 同一颗卫星在基准站观测值的下标
        if(bi >= baseObs.satNum)
            // 基准站未观测到该卫星
            continue;
        if(!baseObs.satObs[bi].check())
            // 基准站sys系统的prn号卫星观测值数据不可用
            // 注意这里必须跟上面分开写，因为bi可能为114514，超出数组索引上限
            continue;
        
        int rSatIndex = roverSpp.epkPos.FindSatPosIndex(prn, sys);
        int bSatIndex = baseSpp.epkPos.FindSatPosIndex(prn, sys);
        if(rSatIndex == 114514 || bSatIndex == 114514)
            // 卫星位置未计算
            continue;
        if(!roverSpp.epkPos.satPos[rSatIndex].valid || !baseSpp.epkPos.satPos[bSatIndex].valid)
            // 两个valid其中任意一个为false
            // 说明卫星位置计算失败
            continue;
        
        satSd[sdi].prn = prn; satSd[sdi].sys = sys;  // 单差观测值的卫星系统和编号
        satSd[sdi].rId = i; satSd[sdi].bId = bi;  // 流动站和基站卫星观测值索引号
        // 双频伪距观测值单差; 注意永远是 流动站-基准站
        satSd[sdi].psrSD[0] = roverObs.satObs[i].P[0] - baseObs.satObs[bi].P[0];
        satSd[sdi].psrSD[1] = roverObs.satObs[i].P[1] - baseObs.satObs[bi].P[1];
        // 双频载波相位观测值单差; 注意永远是 流动站-基准站
        satSd[sdi].cpSD[0] = roverObs.satObs[i].L[0] - baseObs.satObs[bi].L[0];
        satSd[sdi].cpSD[1] = roverObs.satObs[i].L[1] - baseObs.satObs[bi].L[1];
        
        // 单差观测值数目(GPS, BDS)
        ++sdNum;
        satSd[sdi].valid = true;  // 观测值可用
        ++sdi;  // 单差观测值循环变量+1
    }
}

void CDetectSlip::DetectCycleSlip(SDObs &sd)
{
    double f1 = 1.0;  // L1频率
    double f2 = 1.0;  // L2频率
    
    memset(&curEpk, 0, sizeof(EPKGFMW));
    
    for (int i = 0; i < sd.sdNum; ++i)
    {
        if (!sd.satSd[i].valid)  // 检查数据是否完整
        {
            //双频观测值才做组合
            memset(curEpk.gfmw + i, 0, sizeof(GFMW));
            curEpk.gfmw[i].valid = false;
            continue;
        }
        double P1 = sd.satSd[i].psrSD[0];
        double P2 = sd.satSd[i].psrSD[1];
        double L1 = sd.satSd[i].cpSD[0];
        double L2 = sd.satSd[i].cpSD[1];
        
        //标记该GFMW组合的卫星种类和PRN号
        curEpk.gfmw[i].sys = sd.satSd[i].sys;
        curEpk.gfmw[i].prn = sd.satSd[i].prn;
        
        if (sd.satSd[i].sys == GNSS::GPS)
        {
            f1 = constant::fq_l1;
            f2 = constant::fq_l2;
        }
        else if (sd.satSd[i].sys == GNSS::BDS)
        {
            f1 = constant::fq_b1;
            f2 = constant::fq_b3;
        }
        curEpk.gfmw[i].LMW = 1.0 / (f1 - f2) * (f1*L1 - f2*L2) - 1.0 / (f1 + f2) * (f1*P1 + f2*P2);  // MW组合
        curEpk.gfmw[i].LGF = L1 - L2;  // GF组合
        
        int j = lastEpk.FindSatObsIndex(sd.satSd[i].prn, sd.satSd[i].sys);  // raw.epkObs里的卫星在上一历元GFMW观测值里的下标
        if (j == 114514)
        {
            //说明这是第一个历元
            curEpk.gfmw[i].valid = true;
            curEpk.gfmw[i].n = 1;
            
            double dif = f1*f1 - f2*f2;  // 平方差
            //IF组合
            curEpk.gfmw[i].LIF = 1.0 / dif * (f1*f1*L1 - f2*f2*L2);  // LIF
            curEpk.gfmw[i].PIF = 1.0 / dif * (f1*f1*P1 - f2*f2*P2);  // PIF
            
        }
        else if (j <sd.sdNum && j >= 0)
        {
            double dGF = curEpk.gfmw[i].LGF - lastEpk.gfmw[j].LGF;
            double dMW = curEpk.gfmw[i].LMW - lastEpk.gfmw[j].LMW;
            if ((fabs(dGF) < 0.05 && fabs(dMW) < 3.0))
            {
                curEpk.gfmw[i].valid = true;
                curEpk.gfmw[i].LMW = (lastEpk.gfmw[j].n * lastEpk.gfmw[j].LMW + curEpk.gfmw[i].LMW)
                                     / (lastEpk.gfmw[j].n + 1.0);
                curEpk.gfmw[i].n = lastEpk.gfmw[j].n + 1;
                
                double dif = f1*f1 - f2*f2;  // 平方差
                //IF组合
                curEpk.gfmw[i].LIF = 1.0 / dif * (f1*f1*L1 - f2*f2*L2);  // LIF
                curEpk.gfmw[i].PIF = 1.0 / dif * (f1*f1*P1 - f2*f2*P2);  // PIF
            }
            else
            {
                curEpk.gfmw[i].valid = false;  // 标记为粗差不可用
            }
        }
        sd.satSd[i].valid = curEpk.gfmw[i].valid;  // 将原始单差观测值打上粗差标记
        
    }
    memcpy(&lastEpk, &curEpk, sizeof(EPKGFMW));
}


