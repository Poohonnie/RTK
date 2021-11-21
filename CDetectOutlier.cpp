#include "CDetectOutlier.h"

void CDetectOutlier::DetectOutlier(RAWDATA raw)
{
	double f1 = 1.0;//L1Ƶ��
	double f2 = 1.0;//L2Ƶ��
	
	memset(&this->curEpk, 0, sizeof(EPKGFMW));

	for (int oi = 0, gi = 0; oi < raw.epkObs.satNum; oi++, gi++)
	{
		raw.epkObs.satObs[oi].check();//��������Ƿ�����
		if (!raw.epkObs.satObs[oi].valid)
			//˫Ƶ�۲�ֵ�������
			continue;

		double L1 = raw.epkObs.satObs[oi].L[0];
		double L2 = raw.epkObs.satObs[oi].L[1];
		double P1 = raw.epkObs.satObs[oi].P[0];
		double P2 = raw.epkObs.satObs[oi].P[1];

		//��Ǹ�GFMW��ϵ����������PRN��
		this->curEpk.gfmw[gi].sys = raw.epkObs.satObs[oi].sys;
		this->curEpk.gfmw[gi].prn = raw.epkObs.satObs[oi].prn;
		this->curEpk.gfmw[gi].n = 1;//��Ǹ�gfmw������״γ��֣���һ����Ԫ

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
		this->curEpk.gfmw[gi].LMW = 1.0 / (f1 - f2) * (f1 * L1 - f2 * L2) - 1.0 / (f1 + f2) * (f1 * P1 + f2 * P2);//MW���
		this->curEpk.gfmw[gi].LGF = L1 - L2;//GF���

		int j = this->lastEpk.FindSatObsIndex(raw.epkObs.satObs[oi].prn, raw.epkObs.satObs[oi].sys);//raw.epkObs�����������һ��ԪGFMW�۲�ֵ����±�
		if (j == 114514)
		{
			//˵�����ǵ�һ����Ԫ
			this->curEpk.gfmw[gi].valid = true;
			this->curEpk.gfmw[gi].n = 1;

			double dif = f1 * f1 - f2 * f2;//ƽ����
			//IF���
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

				double dif = f1 * f1 - f2 * f2;//ƽ����
				//IF���
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
			return i;//�������ҵ���satObs�����±�
	}
	return 114514;
}
