#include "CDetectOutlier.h"

void CDetectOutlier::DetectOutlier(RAWDATA raw)
{
	double f1 = 1.0;//L1频率
	double f2 = 1.0;//L2频率
	
	memset(&this->curEpk, 0, sizeof(EPKGFMW));

	for (int oi = 0, gi = 0; oi < raw.epkObs.satNum; oi++, gi++)
	{
		raw.epkObs.satObs[oi].check();//检查数据是否完整
		if (!raw.epkObs.satObs[oi].valid)
			//双频观测值才做组合
			continue;

		double L1 = raw.epkObs.satObs[oi].L[0];
		double L2 = raw.epkObs.satObs[oi].L[1];
		double P1 = raw.epkObs.satObs[oi].P[0];
		double P2 = raw.epkObs.satObs[oi].P[1];

		//标记该GFMW组合的卫星种类和PRN号
		this->curEpk.gfmw[gi].sys = raw.epkObs.satObs[oi].sys;
		this->curEpk.gfmw[gi].prn = raw.epkObs.satObs[oi].prn;
		this->curEpk.gfmw[gi].n = 1;//标记该gfmw组合是首次出现，第一个历元

		if (raw.epkObs.satObs[oi].sys == GNSS::GPS)
		{
			f1 = 1575.42e+6;
			f2 = 1227.60e+6;
		}
		else if (raw.epkObs.satObs[oi].sys == GNSS::BDS)
		{
			f1 = 1561.098e+6;
			f2 = 1268.52e+6;
		}
		this->curEpk.gfmw[gi].LMW = 1.0 / (f1 - f2) * (f1 * L1 - f2 * L2) - 1.0 / (f1 + f2) * (f1 * P1 + f2 * P2);//MW组合
		this->curEpk.gfmw[gi].LGF = L1 - L2;//GF组合

		int j = this->lastEpk.FindSatObsIndex(raw.epkObs.satObs[oi].prn, raw.epkObs.satObs[oi].sys);//raw.epkObs里的卫星在上一历元GFMW观测值里的下标
		if (j == 114514)
		{
			//说明这是第一个历元
			this->curEpk.gfmw[gi].valid = true;
			this->curEpk.gfmw[gi].n = 1;

			double dif = f1 * f1 - f2 * f2;//平方差
			//IF组合
			this->curEpk.gfmw[gi].LIF = 1.0 / dif * (f1 * f1 * L1 - f2 * f2 * L2);//LIF
			if (curEpk.gfmw[gi].sys == GNSS::GPS)
				this->curEpk.gfmw[gi].PIF = 1.0 / dif * (f1 * f1 * P1 - f2 * f2 * P2);//PIF
			else if (curEpk.gfmw[gi].sys == GNSS::BDS)
				this->curEpk.gfmw[gi].PIF = 1.0 / dif * (f1 * f1 * P1 - f2 * f2 * P2) - constant::c * f1 * f1 * raw.bdsEphem[raw.epkObs.satObs[oi].prn - 1].tgd[0] / dif;//PIF
		}
		else if (j < raw.epkObs.satNum && j >= 0)
		{
			double dGF = this->curEpk.gfmw[gi].LGF - this->lastEpk.gfmw[j].LGF;
			double dMW = this->curEpk.gfmw[gi].LMW - this->lastEpk.gfmw[j].LMW;
			if ((fabs(dGF) < 0.05 && fabs(dMW) < 3.0))
			{
				this->curEpk.gfmw[gi].valid = true;
				this->curEpk.gfmw[gi].LMW = (this->lastEpk.gfmw[j].n * this->lastEpk.gfmw[j].LMW + this->curEpk.gfmw[gi].LMW) / (this->lastEpk.gfmw[j].n + 1.0);
				this->curEpk.gfmw[gi].n = this->lastEpk.gfmw[j].n + 1;

				double dif = f1 * f1 - f2 * f2;//平方差
				//IF组合
				this->curEpk.gfmw[gi].LIF = 1.0 / dif * (f1 * f1 * L1 - f2 * f2 * L2);//LIF
				if (curEpk.gfmw[gi].sys == GNSS::GPS)
					this->curEpk.gfmw[gi].PIF = 1.0 / dif * (f1 * f1 * P1 - f2 * f2 * P2);//PIF
				else if (curEpk.gfmw[gi].sys == GNSS::BDS)
					this->curEpk.gfmw[gi].PIF = 1.0 / dif * (f1 * f1 * P1 - f2 * f2 * P2) - constant::c * f1 * f1 * raw.bdsEphem[raw.epkObs.satObs[oi].prn - 1].tgd[0] / dif;//PIF
			}
			else
			{
				memset(this->curEpk.gfmw + gi, 0, sizeof(GFMW));
				gi--;
			}
		}
	}
	memcpy(&this->lastEpk, &this->curEpk, sizeof(EPKGFMW));
}

int EPKGFMW::FindSatObsIndex(const int prn, const GNSS sys)
{
	for (int i = 0; i < MAXCHANNELNUM; i++)
	{
		if (this->gfmw[i].prn == prn && this->gfmw[i].sys == sys)
			return i;//返回所找到的satObs数组下标
	}
	return 114514;
}
