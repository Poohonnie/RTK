#include <cstring>
#include <cmath>
#include "RTKlib.h"
#include "CDecode.h"
#include "Detect.h"

void CDetectOutlier::DetectOutlier(RAWDATA& raw)
{
	double f1 = 1.0;  // L1频率
	double f2 = 1.0;  // L2频率
	
	memset(&curEpk, 0, sizeof(EPKGFMW));

	for (int i = 0; i < raw.epkObs.satNum; ++i)
	{
		raw.epkObs.satObs[i].check();  // 检查数据是否完整
		if (!raw.epkObs.satObs[i].valid)
        {
            //双频观测值才做组合
            memset(curEpk.gfmw + i, 0, sizeof(GFMW));
            curEpk.gfmw[i].valid = false;
            continue;
        }
		double L1 = raw.epkObs.satObs[i].L[0];
		double L2 = raw.epkObs.satObs[i].L[1];
		double P1 = raw.epkObs.satObs[i].P[0];
		double P2 = raw.epkObs.satObs[i].P[1];

		//标记该GFMW组合的卫星种类和PRN号
		curEpk.gfmw[i].sys = raw.epkObs.satObs[i].sys;
		curEpk.gfmw[i].prn = raw.epkObs.satObs[i].prn;

		if (raw.epkObs.satObs[i].sys == GNSS::GPS)
		{
			f1 = constant::fq_l1;
			f2 = constant::fq_l2;
		}
		else if (raw.epkObs.satObs[i].sys == GNSS::BDS)
		{
			f1 = constant::fq_b1;
			f2 = constant::fq_b3;
		}
		curEpk.gfmw[i].LMW = 1.0 / (f1 - f2) * (f1 * L1 - f2 * L2) - 1.0 / (f1 + f2) * (f1 * P1 + f2 * P2);  // MW组合
		curEpk.gfmw[i].LGF = L1 - L2;  // GF组合

		int j = lastEpk.FindSatObsIndex(raw.epkObs.satObs[i].prn, raw.epkObs.satObs[i].sys);  // raw.epkObs里的卫星在上一历元GFMW观测值里的下标
		if (j == 114514)
		{
			//说明这是第一个历元
			curEpk.gfmw[i].valid = true;
			curEpk.gfmw[i].n = 1;

			double dif = f1 * f1 - f2 * f2;  // 平方差
			//IF组合
			curEpk.gfmw[i].LIF = 1.0 / dif * (f1 * f1 * L1 - f2 * f2 * L2);  // LIF
			if (curEpk.gfmw[i].sys == GNSS::GPS)
				curEpk.gfmw[i].PIF = 1.0 / dif * (f1 * f1 * P1 - f2 * f2 * P2);  // PIF
			else if (curEpk.gfmw[i].sys == GNSS::BDS)
				curEpk.gfmw[i].PIF = 1.0 / dif * (f1 * f1 * P1 - f2 * f2 * P2) - constant::c * f1 * f1 * raw.bdsEphem[raw.epkObs.satObs[i].prn - 1].tgd[0] / dif;  // PIF
		}
		else if (j < raw.epkObs.satNum && j >= 0)
        {
            double dGF = curEpk.gfmw[i].LGF - lastEpk.gfmw[j].LGF;
            double dMW = curEpk.gfmw[i].LMW - lastEpk.gfmw[j].LMW;
            if ((fabs(dGF) < 0.05 && fabs(dMW) < 3.0))
            {
                curEpk.gfmw[i].valid = true;
                curEpk.gfmw[i].LMW =
                        (lastEpk.gfmw[j].n * lastEpk.gfmw[j].LMW + curEpk.gfmw[i].LMW) / (lastEpk.gfmw[j].n + 1.0);
                curEpk.gfmw[i].n = lastEpk.gfmw[j].n + 1;
                
                double dif = f1 * f1 - f2 * f2;  // 平方差
                //IF组合
                curEpk.gfmw[i].LIF = 1.0 / dif * (f1 * f1 * L1 - f2 * f2 * L2);  // LIF
                if (curEpk.gfmw[i].sys == GNSS::GPS)
                    curEpk.gfmw[i].PIF = 1.0 / dif * (f1 * f1 * P1 - f2 * f2 * P2);  // PIF
                else if (curEpk.gfmw[i].sys == GNSS::BDS)
                    curEpk.gfmw[i].PIF = 1.0 / dif * (f1 * f1 * P1 - f2 * f2 * P2) -
                                         constant::c * f1 * f1 * raw.bdsEphem[raw.epkObs.satObs[i].prn - 1].tgd[0] /
                                         dif;  // PIF
            }
            else
            {
                curEpk.gfmw[i].valid = false;  // 标记为粗差不可用
            }
        }
        raw.epkObs.satObs[i].valid = curEpk.gfmw[i].valid;  // 将原始观测值打上粗差标记
	}
	memcpy(&lastEpk, &curEpk, sizeof(EPKGFMW));
}

int EPKGFMW::FindSatObsIndex(const int prn, const GNSS sys)
{
	for (int i = 0; i < MAXCHANNELNUM; ++i)
	{
		if (gfmw[i].prn == prn && gfmw[i].sys == sys)
			return i;  // 返回所找到的satObs数组下标
	}
	return 114514;
}
