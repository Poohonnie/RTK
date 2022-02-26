#include "SPP.h"


void SPP::ExtendMatB(CMatrix& B, int total) const
{
    //0:GPS 1:BDS 2:GLONASS 3:Galileo
    double sysB[4][MAXCHANNELNUM];//������ϵͳ��B�����е�ϵ�������ﰴ���Ĵ������������Դ��н�ȡ
    
    //����ֵ
    memset(sysB, 0, 4 * MAXCHANNELNUM * 8);
    
    //��1,����չB����
    int sysCount{};
    int curTotal{};
    for(int i = 0; i < 4; ++i)
    {
        if(sysNum[i])
        {
            for (int j = 0; j < sysNum[i] && j < total; j++)
                sysB[i][j + curTotal] = 1;
            B.AddCol(sysB[i], 3 + sysCount);
            curTotal += sysNum[i];
            ++sysCount;
        }
    }
}

void SPP::ExtendDeltaX(CMatrix& deltaX) const
{
    double clk[1] = {};
    for(int i = 0; i < 2; ++i)
    {
        if(!sysNum[i])
        {
            //������Ϊ0��˵����չB�����ʱ��û�п�����
            //����Ҫ��0�Ӳ�����Ϊ���ս����չ��ȥ
            deltaX.AddRow(clk, 3 + i);
        }
    }
}

void SPP::StdPntPos(RAWDATA& raw, EPKGFMW& epkGfmw)
{
	this->t = raw.epkObs.t;

	int usfNum{};//�������Ǽ���
	double arrB[MAXCHANNELNUM * 3] = {};//���ó���ô�����������ٴ��������ȡ���õ�����������������
	double arrw[MAXCHANNELNUM * 1] = {};//w����

	CMatrix deltaX(5, 1);
	deltaX.mat[0] = 0; deltaX.mat[1] = 0; deltaX.mat[2] = 0;
	deltaX.mat[3] = 0;//tG��GPS�źŽ���ʱ�̽��ջ��Ӳ�
	deltaX.mat[4] = 0;//tC��BDS�źŽ���ʱ�̽��ջ��Ӳ�

	//��վ��ʼ�����Ӳ����Ϊ��һ��Ԫ������
	this->check();//��һ��Ԫ�������Լ�
	CMatrix sttnX(5, 1);
	sttnX.mat[0] = this->sttnXyz.x;
	sttnX.mat[1] = this->sttnXyz.y;
	sttnX.mat[2] = this->sttnXyz.z;
	sttnX.mat[3] = this->sttnClkG;
	sttnX.mat[4] = this->sttnClkB;
	memset(this->satPos, 0, MAXCHANNELNUM * sizeof(SatPos));

	int calTimes = 0;//��������
	do
	{
		usfNum = 0;
		memset(&sysNum, 0, 32);

		sttnX.mat[0] += deltaX.mat[0];
		sttnX.mat[1] += deltaX.mat[1];
		sttnX.mat[2] += deltaX.mat[2];
		sttnX.mat[3] += deltaX.mat[3];
		sttnX.mat[4] += deltaX.mat[4];

		for (int i = 0; i < raw.epkObs.satNum; i++)
		{
			int prn = raw.epkObs.satObs[i].prn;
			raw.epkObs.satObs[i].check();
			if (!raw.epkObs.satObs[i].valid)
				//���˫Ƶ�۲�ֵ�Ƿ�����
				continue;
			int gi = epkGfmw.FindSatObsIndex(prn, raw.epkObs.satObs[i].sys);
			if (gi == 114514 || !epkGfmw.gfmw[gi].valid)
				continue;//��һ��Ԫ���ڴֲ�
            
            EPHEMERIS ephem{};
            double clk{};
            int arrNum{};
            if(raw.epkObs.satObs[i].sys == GNSS::GPS && raw.gpsEphem[prn - 1].prn == prn)
            {
                ephem = raw.gpsEphem[prn - 1];
                clk = sttnClkG;
                arrNum = 3;
            }
            else if (raw.epkObs.satObs[i].sys == GNSS::BDS && raw.bdsEphem[prn - 1].prn == prn)
            {
                ephem = raw.bdsEphem[prn - 1];
                clk = sttnClkB;
                arrNum = 4;
            }
            if(ephem.prn == prn/*��������������*/)
            {
                //�����źŷ���ʱ��
                double transT = epkGfmw.gfmw[gi].PIF / constant::c;
    
                GPSTIME ttr = raw.epkObs.t/*�۲�ʱ��*/ - transT;//�źŷ���ʱ��
                if (SatPos::Overdue(ttr, ephem))
                    //��������
                    continue;
                for (int k = 0; k < 4; k++)
                {
                    satPos[i].CalSat(ttr, ephem);
                    ttr = ttr - satPos[i].clkBias;
                }
                //�����źŴ���ʱ��
                double deltat1 = raw.epkObs.t - ttr - clk / constant::c;
    
                //������ת����ǰ������λ��
                XYZ satXyzk{};
                satXyzk.x = satPos[i].satXyz.x;
                satXyzk.y = satPos[i].satXyz.y;
                satXyzk.z = satPos[i].satXyz.z;
                //������ת�����Ƕ�
                double a = wgs84.omega * deltat1;
                //������ת����������λ��
                satPos[i].satXyz.x = satXyzk.x * cos(a) + satXyzk.y * sin(a);
                satPos[i].satXyz.y = -satXyzk.x * sin(a) + satXyzk.y * cos(a);
                satPos[i].satXyz.z = satXyzk.z;
    
                //���Ǹ߶ȽǼ���
                XYZ curSttn{};
                curSttn.x = sttnX.mat[0];
                curSttn.y = sttnX.mat[1];
                curSttn.z = sttnX.mat[2];
                satPos[i].CalSatE(curSttn, wgs84);
                if (satPos[i].eleAngle < 10.0 * constant::pi / 180 && sqrt(curSttn.x * curSttn.x + curSttn.y * curSttn.y + curSttn.z * curSttn.z) > 1e+6)
                    //�߶Ƚǵ���10�ȵ����ǲ��������
                    //xyz��ֵ���Թ�Сʱ��������һ���ж�
                    continue;
    
                //���������
                satPos[i].Hopefield(curSttn, wgs84);
    
                //���ǵ����ջ��ļ��ξ���
                double lx = sttnX.mat[0] - satPos[i].satXyz.x;
                double ly = sttnX.mat[1] - satPos[i].satXyz.y;
                double lz = sttnX.mat[2] - satPos[i].satXyz.z;
                double range = sqrt(fabs(lx * lx + ly * ly + lz * lz));
                //B����ֵ
                arrB[usfNum * 3 + 0] = lx / range;
                arrB[usfNum * 3 + 1] = ly / range;
                arrB[usfNum * 3 + 2] = lz / range;
    
                //w����ֵ
                arrw[usfNum] = epkGfmw.gfmw[gi].PIF - (range + sttnX.mat[arrNum] - constant::c * satPos[i].clkBias + satPos[i].tropDelay);
                if (ephem.satSys == GNSS::GPS)
                    ++sysNum[0];
                else if (ephem.satSys == GNSS::BDS)
                    ++sysNum[1];
                usfNum++;//�������Ǽ���+1
            }
		}
		if (usfNum < 5)//������Ŀ���٣������ж�λ����
			return;
		CMatrix B(arrB, usfNum, 3);//B����(δ�����Ӳ�ϵ��������չ)
		ExtendMatB(B, usfNum);
		CMatrix w(arrw, usfNum, 1);
		CMatrix BT = B.Trans();
		CMatrix BTB(B.cols, B.cols);//BT * B
		CMatrix BTw(B.cols, 1);//BT * w
		BTB = BT * B;
		BTw = BT * w;
		deltaX = BTB.Inv() * BTw;//�õ���С���˽� ������������Ϊ0��ϵͳ
		ExtendDeltaX(deltaX);

		//��ֹ�����ɢ����λ��Ư�������
		for (int g = 0; g < 5; g++)
		{
			if (deltaX.mat[g] > 8e+7)
				deltaX.mat[g] = 1e-8;
		}
		calTimes++;//��������+1
	} while (sqrt(deltaX.mat[0] * deltaX.mat[0] + deltaX.mat[1] * deltaX.mat[1] + deltaX.mat[2] * deltaX.mat[2]) > 1e-4 && calTimes < 10);

	//���ն�λ���
	this->sttnXyz.x = sttnX.mat[0];
	this->sttnXyz.y = sttnX.mat[1];
	this->sttnXyz.z = sttnX.mat[2];

	this->sttnBlh = XYZ2BLH(sttnXyz, wgs84);

	//���о�������
	CMatrix B(arrB, usfNum, 3);//B����
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

	//���ݴ洢
	this->sttnClkG = sttnX.mat[3];
	this->sttnClkB = sttnX.mat[4];
}

void SPP::StdPntVel(RAWDATA& raw, EPKGFMW& epkGfmw)
{
	int usfNum = 0;//�������Ǽ���
	double arrB[MAXCHANNELNUM * 4] = {};//���ó���ô�����������ٴ��������ȡ���õ�����������������
	double arrw[MAXCHANNELNUM * 1] = {};//w����
	//��վ���پ���
	CMatrix sttnv(4, 1);
    sttnv.mat[0] = 0; sttnv.mat[1] = 0; sttnv.mat[2] = 0;
    sttnv.mat[3] = 0;
	
	for (int i = 0; i < raw.epkObs.satNum; i++)
	{
		int prn = raw.epkObs.satObs[i].prn;
		raw.epkObs.satObs[i].check();
		if (!raw.epkObs.satObs[i].valid || !prn)
			//���˫Ƶ�۲�ֵ�Ƿ�����
			continue;
		int gi = epkGfmw.FindSatObsIndex(prn, raw.epkObs.satObs[i].sys);
		if (gi == 114514 || !epkGfmw.gfmw[gi].valid)
			continue;//��һ��Ԫ���ڴֲ�
		if (satPos[i].eleAngle < 10.0 * constant::pi / 180)
			//�߶Ƚǵ���10�ȵ����ǲ��������
			continue;

		double lx = this->satPos[i].satXyz.x - this->sttnXyz.x;
		double ly = this->satPos[i].satXyz.y - this->sttnXyz.y;
		double lz = this->satPos[i].satXyz.z - this->sttnXyz.z;
		double range = sqrt(fabs(lx * lx + ly * ly + lz * lz));
		double rhoDot = (lx * this->satPos[i].satV[0] + ly * this->satPos[i].satV[1] + lz * this->satPos[i].satV[2]) / range;

		arrB[usfNum * 4 + 0] = lx / range;
		arrB[usfNum * 4 + 1] = ly / range;
		arrB[usfNum * 4 + 2] = lz / range;
		arrB[usfNum * 4 + 3] = 1;

		if (raw.epkObs.satObs[i].sys == GNSS::GPS && raw.gpsEphem[prn - 1].prn == prn/*��������������*/)
		{
			double lambda1 = constant::c / 1575.42e+6;
			//w����ֵ
			arrw[usfNum] = -lambda1 * raw.epkObs.satObs[i].D[0] + constant::c * this->satPos[i].clkRate - rhoDot;
		}
		else if (raw.epkObs.satObs[i].sys == GNSS::BDS && raw.bdsEphem[prn - 1].prn == prn/*��������������*/)
		{
			double lambda1 = constant::c / 1561.098e+6;
			//w����ֵ
			arrw[usfNum] = -lambda1 * raw.epkObs.satObs[i].D[0] + constant::c * this->satPos[i].clkRate - rhoDot;
		}

		usfNum++;
	}
	if (usfNum < 5)//������Ŀ���٣������ж�λ����
		return;
	CMatrix B(arrB, usfNum, 4);//B����
	CMatrix w(arrw, usfNum, 1);
	CMatrix BT = B.Trans();
	CMatrix BTB(4, 4);//BT * B
	CMatrix BTw(4, 1);//BT * w
	BTB = BT * B;
	BTw = BT * w;
    sttnv = BTB.Inv() * BTw;//�õ���С���˽�

	//���ղ��ٽ��
	this->sttnV[0] = sttnv.mat[0];
	this->sttnV[1] = sttnv.mat[1];
	this->sttnV[2] = sttnv.mat[2];

	//���о�������
	CMatrix V(usfNum, 1);
	V = B * sttnv - w;
	CMatrix VTV(1, 1);
	VTV = V.Trans() * V;
	this->sigmaV = sqrt(VTV.mat[0] / (usfNum - 4.0));
}

void SPP::check()
{
	if (fabs(sttnBlh.H) > 1e+4 )
	{
		memset(this, 0, sizeof(SPP));
	}
}

void SPP::CalDNEU()
{
	XYZ refXyz = { -2267794.9370, 5009345.2360, 3220980.3120 };
	BLH refBlh = XYZ2BLH(refXyz, wgs84);
	double dXyz[3] = {};
	dXyz[0] = this->sttnXyz.x - refXyz.x;
	dXyz[1] = this->sttnXyz.y - refXyz.y;
	dXyz[2] = this->sttnXyz.z - refXyz.z;

	this->dN = -sin(refBlh.B) * cos(refBlh.L) * dXyz[0] - sin(refBlh.B) * sin(refBlh.L) * dXyz[1] + cos(refBlh.B) * dXyz[2];
	this->dE = -sin(refBlh.L) * dXyz[0] + cos(refBlh.L) * dXyz[1];
	this->dU = cos(refBlh.B) * cos(refBlh.L) * dXyz[0] + cos(refBlh.B) * sin(refBlh.L) * dXyz[1] + sin(refBlh.B) * dXyz[2];
}


