#include "SPP.h"

void SPP::ExtendMatB(CMatrix& B, int total)
{
	double* gB = new double[total];
	double* bB = new double[total];
	//����ֵ
	memset(gB, 0, 8 * total);
	memset(bB, 0, 8 * total);

	//��1,����չB����
	if (this->gNum)
	{
		for (int i = 0; i < this->gNum && i < total; i++)
			gB[i] = 1;
		B.AddCol(gB, 3);
		if (this->bNum)
		{
			for (int i = this->gNum; i < total; i++)
				bB[i] = 1;
			B.AddCol(bB, 4);
		}
	}
	else if (this->bNum)
	{
		for (int i = 0; i < this->bNum && i < total; i++)
			bB[i] = 1;
		B.AddCol(bB, 3);
	}
	delete[] gB;
	delete[] bB;
}

void SPP::SglPntPos(RAWDATA& raw, EPKGFMW& epkGfmw)
{
	WGS84 wgs84;
	int usfNum = 0;//�������Ǽ���
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

	if (raw.epkObs.t.secOfWeek == 125820)
		system("Pause");

	int calTimes = 0;//��������
	do
	{
		usfNum = 0;
		this->gNum = 0;
		this->bNum = 0;

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
			if (gi == 114514)
				continue;//��һ��Ԫ���ڴֲ�
			else if (gi < raw.epkObs.satNum && gi >= 0)
			{
				if (!epkGfmw.gfmw[gi].valid)
					//�ٴμ���Ƿ��дֲ�
					continue;
			}

			if (raw.epkObs.satObs[i].sys == GNSS::GPS && raw.gpsEphem[prn - 1].prn == prn/*��������������*/)
			{
				SatPositioning satPos;

				//�����źŷ���ʱ��
				double transT = raw.epkObs.satObs[i].P[0] / constant::c;
				GPSTIME ttr = raw.epkObs.t/*�۲�ʱ��*/ - transT;//�źŷ���ʱ��
				if (!satPos.GpsOod(ttr, raw.gpsEphem[prn - 1]))
					//��������
					continue;
				for (int k = 0; k < 5; k++)
				{
					satPos.CalGps(ttr, raw.gpsEphem[prn - 1]);
					ttr = ttr - satPos.clkBias;
				}
				//�����źŴ���ʱ��
				double deltat1 = raw.epkObs.t - ttr - sttnClkG / constant::c;
				
				//������ת����ǰ������λ��
				XYZ satXyzk;
				satXyzk.x = satPos.satXyz.x;
				satXyzk.y = satPos.satXyz.y;
				satXyzk.z = satPos.satXyz.z;
				//������ת�����Ƕ�
				double a = wgs84.omega * deltat1;
				//������ת����������λ��
				satPos.satXyz.x = satXyzk.x * cos(a) + satXyzk.y * sin(a);
				satPos.satXyz.y = -satXyzk.x * sin(a) + satXyzk.y * cos(a);
				satPos.satXyz.z = satXyzk.z;

				//���Ǹ߶ȽǼ���
				XYZ curSttn;
				curSttn.x = sttnX.mat[0];
				curSttn.y = sttnX.mat[1];
				curSttn.z = sttnX.mat[2];

				satPos.calSatE(curSttn, wgs84);
				if (satPos.eleAngle < 10.0 * constant::pi / 180)
					//�߶Ƚǵ���10�ȵ����ǲ��������
					continue;

				//���������
				satPos.Hopefield(curSttn, wgs84);

				//���ǵ����ջ��ļ��ξ���
				double lx = sttnX.mat[0] - satPos.satXyz.x;
				double ly = sttnX.mat[1] - satPos.satXyz.y;
				double lz = sttnX.mat[2] - satPos.satXyz.z;
				double range = sqrt(fabs(lx*lx + ly*ly + lz*lz));
				//B����ֵ
				arrB[usfNum * 3 + 0] = lx / range;
				arrB[usfNum * 3 + 1] = ly / range;
				arrB[usfNum * 3 + 2] = lz / range;

				//w����ֵ
				arrw[usfNum] = epkGfmw.gfmw[gi].PIF - (range + sttnX.mat[3] - constant::c * satPos.clkBias + satPos.tropDelay);
				this->gNum++;
				usfNum++;//�������Ǽ���+1
			}
			else if (raw.epkObs.satObs[i].sys == GNSS::BDS && raw.bdsEphem[prn - 1].satId == prn/*��������������*/)
			{
				SatPositioning satPos;

				//�����źŷ���ʱ��
				double transT = raw.epkObs.satObs[i].P[0] / constant::c;
				GPSTIME ttr = raw.epkObs.t/*�۲�ʱ��*/ - transT;//�źŷ���ʱ��
				if (!satPos.BdsOod(ttr, raw.bdsEphem[prn - 1]))
					//��������
					continue;
				for (int k = 0; k < 5; k++)
				{
					satPos.CalBds(ttr, raw.bdsEphem[prn - 1]);
					ttr = ttr - satPos.clkBias;
				}
				//�����źŴ���ʱ��
				double deltat1 = raw.epkObs.t - ttr - sttnClkB / constant::c;

				//������ת����ǰ������λ��
				XYZ satXyzk;
				satXyzk.x = satPos.satXyz.x;
				satXyzk.y = satPos.satXyz.y;
				satXyzk.z = satPos.satXyz.z;
				//������ת�����Ƕ�
				double a = wgs84.omega * deltat1;
				//������ת����������λ��(���ĵع�����ϵ)
				satPos.satXyz.x = satXyzk.x * cos(a) + satXyzk.y * sin(a);
				satPos.satXyz.y = -satXyzk.x * sin(a) + satXyzk.y * cos(a);
				satPos.satXyz.z = satXyzk.z;

				//���Ǹ߶ȽǼ���
				XYZ curSttn;
				curSttn.x = sttnX.mat[0];
				curSttn.y = sttnX.mat[1];
				curSttn.z = sttnX.mat[2];

				satPos.calSatE(curSttn, wgs84);

				//���������
				satPos.Hopefield(curSttn, wgs84);

				//���ǵ����ջ��ļ��ξ���
				double lx = sttnX.mat[0] - satPos.satXyz.x;
				double ly = sttnX.mat[1] - satPos.satXyz.y;
				double lz = sttnX.mat[2] - satPos.satXyz.z;
				double range = sqrt(fabs(lx * lx + ly * ly + lz * lz));
				//B����ֵ
				arrB[usfNum * 3 + 0] = lx / range;
				arrB[usfNum * 3 + 1] = ly / range;
				arrB[usfNum * 3 + 2] = lz / range;

				//w����ֵ
				arrw[usfNum] = epkGfmw.gfmw[gi].PIF - (range + sttnX.mat[4] - constant::c * satPos.clkBias + satPos.tropDelay);
				this->bNum++;
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

		if (!this->gNum)
		{
			//���GPS������Ϊ0����ô�Ӳ�����һ��Ԫ�仯ֵΪ0����չ��deltaX������
			double clkG[1] = { 0.0 };
			deltaX.AddRow(clkG, 3);
			if (!this->bNum)
			{
				//���BDS������Ϊ0����ô�Ӳ�����һ��Ԫ�仯ֵΪ0����չ��deltaX������
				double clkB[1] = { 0.0 };
				deltaX.AddRow(clkB, 3);
			}
		}
		else if (!this->bNum)
		{
			//���BDS������Ϊ0����ô�Ӳ�����һ��Ԫ�仯ֵΪ0����չ��deltaX������
			double clkB[1] = { 0.0 };
			deltaX.AddRow(clkB, 4);
		}
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
	this->PDOP = sqrt(BTB.Inv().mat[0] + BTB.Inv().mat[6] + BTB.Inv().mat[12]);

	//���ݴ洢

	this->sttnClkG = sttnX.mat[3];
	this->sttnClkB = sttnX.mat[4];
	this->t = raw.epkObs.t;
	//if (fabs(this->sttnBlh.H) > 100)
	//	system("Pause");
}

void SPP::check()
{
	double x = this->sttnXyz.x;
	double y = this->sttnXyz.y;
	double z = this->sttnXyz.z;

	//XYZ xyz0 = { -2267798.6013, 5009344.4598, 3220981.8681 };
	XYZ xyz0 = { 0.0, 0.0, 0.0 };
	if (fabs(sttnBlh.H) > 1e+2 )
	{
		memset(this, 0, sizeof(SPP));
	}

}
