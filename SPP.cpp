#include "SPP.h"


void SPP::ExtendMatB(CMatrix& B, int total) const
{
	auto* gB = new double[total];//GPS
	auto* bB = new double[total];//BDS
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

void SPP::ExtendDeltaX(CMatrix& deltaX) const
{
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
        //GPS卫星数不为0
		//如果BDS卫星数为0，那么钟差与上一历元变化值为0，扩展进deltaX矩阵里
		double clkB[1] = { 0.0 };
		deltaX.AddRow(clkB, 4);
	}
}

void SPP::StdPntPos(RAWDATA& raw, EPKGFMW& epkGfmw)
{
	this->t = raw.epkObs.t;

	int usfNum;//可用卫星计数
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
	memset(this->satPos, 0, MAXCHANNELNUM * sizeof(SatPos));

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
			if (!raw.epkObs.satObs[i].valid || !prn)
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
				//计算信号发射时刻
				//double transT = raw.epkObs.satObs[i].P[0] / constant::c;
				double transT = epkGfmw.gfmw[gi].PIF / constant::c;
				
				GPSTIME ttr = raw.epkObs.t/*观测时刻*/ - transT;//信号发射时刻
				if (SatPos::Overdue(ttr, raw.gpsEphem[prn - 1]))
					//星历过期
					continue;
				for (int k = 0; k < 4; k++)
				{
					satPos[i].CalSat(ttr, raw.gpsEphem[prn - 1]);
					ttr = ttr - satPos[i].clkBias;
				}
				//计算信号传输时刻
				double deltat1 = raw.epkObs.t - ttr - sttnClkG / constant::c;
				
				//地球自转改正前的卫星位置
				XYZ satXyzk{};
				satXyzk.x = satPos[i].satXyz.x;
				satXyzk.y = satPos[i].satXyz.y;
				satXyzk.z = satPos[i].satXyz.z;
				//地球自转改正角度
				double a = wgs84.omega * deltat1;
				//地球自转改正后卫星位置
				satPos[i].satXyz.x = satXyzk.x * cos(a) + satXyzk.y * sin(a);
				satPos[i].satXyz.y = -satXyzk.x * sin(a) + satXyzk.y * cos(a);
				satPos[i].satXyz.z = satXyzk.z;

				//卫星高度角计算
				XYZ curSttn{};
				curSttn.x = sttnX.mat[0];
				curSttn.y = sttnX.mat[1];
				curSttn.z = sttnX.mat[2];
				satPos[i].CalSatE(curSttn, wgs84);
				if (satPos[i].eleAngle < 10.0 * constant::pi / 180 && sqrt(curSttn.x * curSttn.x + curSttn.y * curSttn.y + curSttn.z * curSttn.z) > 1e+4)
					//xyz数值明显过小时不进行这一步判断，因为BLH会被设为0 0 0,
					//高度角低于10度的卫星不参与解算 
					continue;

				//对流层改正
				satPos[i].Hopefield(curSttn, wgs84);

				//卫星到接收机的几何距离
				double lx = sttnX.mat[0] - satPos[i].satXyz.x;
				double ly = sttnX.mat[1] - satPos[i].satXyz.y;
				double lz = sttnX.mat[2] - satPos[i].satXyz.z;
				double range = sqrt(fabs(lx*lx + ly*ly + lz*lz));
				//B矩阵赋值
				arrB[usfNum * 3 + 0] = lx / range;
				arrB[usfNum * 3 + 1] = ly / range;
				arrB[usfNum * 3 + 2] = lz / range;

				//w矩阵赋值
				arrw[usfNum] = epkGfmw.gfmw[gi].PIF - (range + sttnX.mat[3] - constant::c * satPos[i].clkBias + satPos[i].tropDelay);
				this->gNum++;
				usfNum++;//可用卫星计数+1
			}
			else if (raw.epkObs.satObs[i].sys == GNSS::BDS && raw.bdsEphem[prn - 1].prn == prn/*该卫星星历存在*/)
			{
				//计算信号发射时刻
				//double transT = raw.epkObs.satObs[i].P[0] / constant::c;
				double transT = epkGfmw.gfmw[gi].PIF / constant::c;
				GPSTIME ttr = raw.epkObs.t/*观测时刻*/ - transT;//信号发射时刻
				if (SatPos::Overdue(ttr, raw.bdsEphem[prn - 1]))
					//星历过期
					continue;
				for (int k = 0; k < 4; k++)
				{
					satPos[i].CalSat(ttr, raw.bdsEphem[prn - 1]);
					ttr = ttr - satPos[i].clkBias;
				}
				//计算信号传输时刻
				double deltat1 = raw.epkObs.t - ttr - sttnClkB / constant::c;

				//地球自转改正前的卫星位置
				XYZ satXyzk{};
				satXyzk.x = satPos[i].satXyz.x;
				satXyzk.y = satPos[i].satXyz.y;
				satXyzk.z = satPos[i].satXyz.z;
				//地球自转改正角度
				double a = wgs84.omega * deltat1;
				//地球自转改正后卫星位置(地心地固坐标系)
				satPos[i].satXyz.x = satXyzk.x * cos(a) + satXyzk.y * sin(a);
				satPos[i].satXyz.y = -satXyzk.x * sin(a) + satXyzk.y * cos(a);
				satPos[i].satXyz.z = satXyzk.z;

				//卫星高度角计算
				XYZ curSttn{};
				curSttn.x = sttnX.mat[0];
				curSttn.y = sttnX.mat[1];
				curSttn.z = sttnX.mat[2];
				satPos[i].CalSatE(curSttn, wgs84);
				if (satPos[i].eleAngle < 10.0 * constant::pi / 180 && sqrt(curSttn.x * curSttn.x + curSttn.y * curSttn.y + curSttn.z * curSttn.z) > 1e+4)
					//xyz数值明显过小时不进行这一步判断，因为BLH会被设为0 0 0,
					//高度角低于10度的卫星不参与解算 
					continue;

				//对流层改正
				satPos[i].Hopefield(curSttn, wgs84);

				//卫星到接收机的几何距离
				double lx = sttnX.mat[0] - satPos[i].satXyz.x;
				double ly = sttnX.mat[1] - satPos[i].satXyz.y;
				double lz = sttnX.mat[2] - satPos[i].satXyz.z;
				double range = sqrt(fabs(lx * lx + ly * ly + lz * lz));
				//B矩阵赋值
				arrB[usfNum * 3 + 0] = lx / range;
				arrB[usfNum * 3 + 1] = ly / range;
				arrB[usfNum * 3 + 2] = lz / range;

				//w矩阵赋值
				arrw[usfNum] = epkGfmw.gfmw[gi].PIF - (range + sttnX.mat[4] - constant::c * satPos[i].clkBias + satPos[i].tropDelay);
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
		ExtendDeltaX(deltaX);

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
}






void SPP::StdPntVel(RAWDATA& raw, EPKGFMW& epkGfmw)
{
	int usfNum = 0;//可用卫星计数
	double arrB[MAXCHANNELNUM * 4] = {};//先拿出这么大来，待会再从数组里截取可用的数据下来创建矩阵
	double arrw[MAXCHANNELNUM * 1] = {};//w矩阵
	//测站钟速矩阵
	CMatrix sttnv(4, 1);
    sttnv.mat[0] = 0; sttnv.mat[1] = 0; sttnv.mat[2] = 0;
    sttnv.mat[3] = 0;
	
	for (int i = 0; i < raw.epkObs.satNum; i++)
	{
		int prn = raw.epkObs.satObs[i].prn;
		raw.epkObs.satObs[i].check();
		if (!raw.epkObs.satObs[i].valid || !prn)
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
		if (satPos[i].eleAngle < 10.0 * constant::pi / 180)
			//高度角低于10度的卫星不参与解算
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

		if (raw.epkObs.satObs[i].sys == GNSS::GPS && raw.gpsEphem[prn - 1].prn == prn/*该卫星星历存在*/)
		{
			double lambda1 = constant::c / 1575.42e+6;
			//w矩阵赋值
			arrw[usfNum] = -lambda1 * raw.epkObs.satObs[i].D[0] + constant::c * this->satPos[i].clkRate - rhoDot;
		}
		else if (raw.epkObs.satObs[i].sys == GNSS::BDS && raw.bdsEphem[prn - 1].prn == prn/*该卫星星历存在*/)
		{
			double lambda1 = constant::c / 1561.098e+6;
			//w矩阵赋值
			arrw[usfNum] = -lambda1 * raw.epkObs.satObs[i].D[0] + constant::c * this->satPos[i].clkRate - rhoDot;
		}

		usfNum++;
	}
	if (usfNum < 5)//卫星数目过少，不进行定位解算
		return;
	CMatrix B(arrB, usfNum, 4);//B矩阵
	CMatrix w(arrw, usfNum, 1);
	CMatrix BT = B.Trans();
	CMatrix BTB(4, 4);//BT * B
	CMatrix BTw(4, 1);//BT * w
	BTB = BT * B;
	BTw = BT * w;
    sttnv = BTB.Inv() * BTw;//得到最小二乘解

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
