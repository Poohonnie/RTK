#include "SPP.h"

void SPP::ExtendMatB(CMatrix& B, int total)
{
	double* gB = new double[total];
	double* bB = new double[total];
	//赋初值
	memset(gB, 0, 8 * total);
	memset(bB, 0, 8 * total);

	//赋1,并扩展B矩阵
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
	int usfNum = 0;//可用卫星计数
	double arrB[MAXCHANNELNUM * 3] = {};//先拿出这么大来，待会再从数组里截取可用的数据下来创建矩阵
	double arrw[MAXCHANNELNUM * 1] = {};//w矩阵

	CMatrix deltaX(5, 1);
	deltaX.mat[0] = 0; deltaX.mat[1] = 0; deltaX.mat[2] = 0;
	deltaX.mat[3] = 0;//tG，GPS信号接收时刻接收机钟差
	deltaX.mat[4] = 0;//tC，BDS信号接受时刻接收机钟差

	//测站初始坐标钟差，设置为上一历元结算结果
	this->check();//上一历元结算结果自检
	CMatrix sttnX(5, 1);
	sttnX.mat[0] = this->sttnXyz.x;
	sttnX.mat[1] = this->sttnXyz.y;
	sttnX.mat[2] = this->sttnXyz.z;
	sttnX.mat[3] = this->sttnClkG;
	sttnX.mat[4] = this->sttnClkB;

	if (raw.epkObs.t.secOfWeek == 125820)
		system("Pause");

	int calTimes = 0;//迭代计数
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
				//检查双频观测值是否完整
				continue;
			int gi = epkGfmw.FindSatObsIndex(prn, raw.epkObs.satObs[i].sys);
			if (gi == 114514)
				continue;//这一历元存在粗差
			else if (gi < raw.epkObs.satNum && gi >= 0)
			{
				if (!epkGfmw.gfmw[gi].valid)
					//再次检查是否有粗差
					continue;
			}

			if (raw.epkObs.satObs[i].sys == GNSS::GPS && raw.gpsEphem[prn - 1].prn == prn/*该卫星星历存在*/)
			{
				SatPositioning satPos;

				//计算信号发射时刻
				double transT = raw.epkObs.satObs[i].P[0] / constant::c;
				GPSTIME ttr = raw.epkObs.t/*观测时刻*/ - transT;//信号发射时刻
				if (!satPos.GpsOod(ttr, raw.gpsEphem[prn - 1]))
					//星历过期
					continue;
				for (int k = 0; k < 5; k++)
				{
					satPos.CalGps(ttr, raw.gpsEphem[prn - 1]);
					ttr = ttr - satPos.clkBias;
				}
				//计算信号传输时刻
				double deltat1 = raw.epkObs.t - ttr - sttnClkG / constant::c;
				
				//地球自转改正前的卫星位置
				XYZ satXyzk;
				satXyzk.x = satPos.satXyz.x;
				satXyzk.y = satPos.satXyz.y;
				satXyzk.z = satPos.satXyz.z;
				//地球自转改正角度
				double a = wgs84.omega * deltat1;
				//地球自转改正后卫星位置
				satPos.satXyz.x = satXyzk.x * cos(a) + satXyzk.y * sin(a);
				satPos.satXyz.y = -satXyzk.x * sin(a) + satXyzk.y * cos(a);
				satPos.satXyz.z = satXyzk.z;

				//卫星高度角计算
				XYZ curSttn;
				curSttn.x = sttnX.mat[0];
				curSttn.y = sttnX.mat[1];
				curSttn.z = sttnX.mat[2];

				satPos.calSatE(curSttn, wgs84);
				if (satPos.eleAngle < 10.0 * constant::pi / 180)
					//高度角低于10度的卫星不参与解算
					continue;

				//对流层改正
				satPos.Hopefield(curSttn, wgs84);

				//卫星到接收机的几何距离
				double lx = sttnX.mat[0] - satPos.satXyz.x;
				double ly = sttnX.mat[1] - satPos.satXyz.y;
				double lz = sttnX.mat[2] - satPos.satXyz.z;
				double range = sqrt(fabs(lx*lx + ly*ly + lz*lz));
				//B矩阵赋值
				arrB[usfNum * 3 + 0] = lx / range;
				arrB[usfNum * 3 + 1] = ly / range;
				arrB[usfNum * 3 + 2] = lz / range;

				//w矩阵赋值
				arrw[usfNum] = epkGfmw.gfmw[gi].PIF - (range + sttnX.mat[3] - constant::c * satPos.clkBias + satPos.tropDelay);
				this->gNum++;
				usfNum++;//可用卫星计数+1
			}
			else if (raw.epkObs.satObs[i].sys == GNSS::BDS && raw.bdsEphem[prn - 1].satId == prn/*该卫星星历存在*/)
			{
				SatPositioning satPos;

				//计算信号发射时刻
				double transT = raw.epkObs.satObs[i].P[0] / constant::c;
				GPSTIME ttr = raw.epkObs.t/*观测时刻*/ - transT;//信号发射时刻
				if (!satPos.BdsOod(ttr, raw.bdsEphem[prn - 1]))
					//星历过期
					continue;
				for (int k = 0; k < 5; k++)
				{
					satPos.CalBds(ttr, raw.bdsEphem[prn - 1]);
					ttr = ttr - satPos.clkBias;
				}
				//计算信号传输时刻
				double deltat1 = raw.epkObs.t - ttr - sttnClkB / constant::c;

				//地球自转改正前的卫星位置
				XYZ satXyzk;
				satXyzk.x = satPos.satXyz.x;
				satXyzk.y = satPos.satXyz.y;
				satXyzk.z = satPos.satXyz.z;
				//地球自转改正角度
				double a = wgs84.omega * deltat1;
				//地球自转改正后卫星位置(地心地固坐标系)
				satPos.satXyz.x = satXyzk.x * cos(a) + satXyzk.y * sin(a);
				satPos.satXyz.y = -satXyzk.x * sin(a) + satXyzk.y * cos(a);
				satPos.satXyz.z = satXyzk.z;

				//卫星高度角计算
				XYZ curSttn;
				curSttn.x = sttnX.mat[0];
				curSttn.y = sttnX.mat[1];
				curSttn.z = sttnX.mat[2];

				satPos.calSatE(curSttn, wgs84);

				//对流层改正
				satPos.Hopefield(curSttn, wgs84);

				//卫星到接收机的几何距离
				double lx = sttnX.mat[0] - satPos.satXyz.x;
				double ly = sttnX.mat[1] - satPos.satXyz.y;
				double lz = sttnX.mat[2] - satPos.satXyz.z;
				double range = sqrt(fabs(lx * lx + ly * ly + lz * lz));
				//B矩阵赋值
				arrB[usfNum * 3 + 0] = lx / range;
				arrB[usfNum * 3 + 1] = ly / range;
				arrB[usfNum * 3 + 2] = lz / range;

				//w矩阵赋值
				arrw[usfNum] = epkGfmw.gfmw[gi].PIF - (range + sttnX.mat[4] - constant::c * satPos.clkBias + satPos.tropDelay);
				this->bNum++;
				usfNum++;//可用卫星计数+1
			}
		}
		if (usfNum < 5)//卫星数目过少，不进行定位解算
			return;
		CMatrix B(arrB, usfNum, 3);//B矩阵(未根据钟差系数进行扩展)
		ExtendMatB(B, usfNum);
		CMatrix w(arrw, usfNum, 1);
		CMatrix BT = B.Trans();
		CMatrix BTB(B.cols, B.cols);//BT * B
		CMatrix BTw(B.cols, 1);//BT * w
		BTB = BT * B;
		BTw = BT * w;
		deltaX = BTB.Inv() * BTw;//得到最小二乘解 不包含卫星数为0的系统

		if (!this->gNum)
		{
			//如果GPS卫星数为0，那么钟差与上一历元变化值为0，扩展进deltaX矩阵里
			double clkG[1] = { 0.0 };
			deltaX.AddRow(clkG, 3);
			if (!this->bNum)
			{
				//如果BDS卫星数为0，那么钟差与上一历元变化值为0，扩展进deltaX矩阵里
				double clkB[1] = { 0.0 };
				deltaX.AddRow(clkB, 3);
			}
		}
		else if (!this->bNum)
		{
			//如果BDS卫星数为0，那么钟差与上一历元变化值为0，扩展进deltaX矩阵里
			double clkB[1] = { 0.0 };
			deltaX.AddRow(clkB, 4);
		}
		//防止结果发散导致位置漂向无穷大
		for (int g = 0; g < 5; g++)
		{
			if (deltaX.mat[g] > 8e+7)
				deltaX.mat[g] = 1e-8;
		}
		calTimes++;//迭代计数+1
	} while (sqrt(deltaX.mat[0] * deltaX.mat[0] + deltaX.mat[1] * deltaX.mat[1] + deltaX.mat[2] * deltaX.mat[2]) > 1e-4 && calTimes < 10);

	//最终定位结果

	this->sttnXyz.x = sttnX.mat[0];
	this->sttnXyz.y = sttnX.mat[1];
	this->sttnXyz.z = sttnX.mat[2];

	this->sttnBlh = XYZ2BLH(sttnXyz, wgs84);

	//进行精度评定
	CMatrix B(arrB, usfNum, 3);//B矩阵
	ExtendMatB(B, usfNum);
	CMatrix BTB = B.Trans() * B;
	CMatrix w(arrw, usfNum, 1);
	CMatrix V(usfNum, 1);
	V = B * deltaX - w;
	CMatrix VTV(1, 1);
	VTV = V.Trans() * V;
	this->sigmaP = sqrt(VTV.mat[0] / (usfNum - 5.0));
	this->PDOP = sqrt(BTB.Inv().mat[0] + BTB.Inv().mat[6] + BTB.Inv().mat[12]);

	//数据存储

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
