#include "SatPositioning.hpp"


void SatPositioning::CalculateParas(const GPSTIME t/*卫星钟表面时*/, const GPSEPHEM& gpsEphem)
{
	WGS84 wgs84;
	//中间计算量求解
	this->paras.A = gpsEphem.A;//GPS轨道长半轴
	this->paras.n0 = sqrt(wgs84.GM / (this->paras.A * this->paras.A * this->paras.A));//平均运动角速度
	this->paras.tk = SoWSubtraction(t.secOfWeek, gpsEphem.toe.secOfWeek);//相对于星历参考历元的时间
	this->paras.n = this->paras.n0 + gpsEphem.deltaN;//对平均运动角速度进行改正
	this->paras.mk = gpsEphem.m0 + this->paras.n * this->paras.tk;//平近点角
	this->paras.mk = this->paras.mk > 0 ? this->paras.mk : this->paras.mk + 2 * constant::pi;
	this->paras.ek = 0;//偏近点角，迭代求解
	double ek1 = this->paras.mk + gpsEphem.ecc * sin(this->paras.mk);
	for (int i = 0; i < 20 && fabs(this->paras.ek - ek1) > 1e-15; i++)
	{
		this->paras.ek = ek1;
		ek1 = this->paras.mk + gpsEphem.ecc * sin(this->paras.ek);
	}
	this->paras.vk = atan2(sqrt(1 - gpsEphem.ecc * gpsEphem.ecc) * sin(this->paras.ek), cos(this->paras.ek) - gpsEphem.ecc);//真近点角
	this->paras.phik = this->paras.vk + gpsEphem.omega;//升交角距(未经改正)
	//计算二阶调和改正数
	this->paras.deltaUk = gpsEphem.cus * sin(2 * this->paras.phik) + gpsEphem.cuc * cos(2 * this->paras.phik);//计算升交角距的改正数
	this->paras.deltaRk = gpsEphem.crs * sin(2 * this->paras.phik) + gpsEphem.crc * cos(2 * this->paras.phik);//计算向径的改正数
	this->paras.deltaIk = gpsEphem.cis * sin(2 * this->paras.phik) + gpsEphem.cic * cos(2 * this->paras.phik);//计算轨道倾角改证数

	//计算经过改正的升交角距，向径和轨道倾角
	this->paras.uk = this->paras.phik + this->paras.deltaUk;//改正过的升交角距
	this->paras.rk = this->paras.A * (1 - gpsEphem.ecc * cos(this->paras.ek)) + this->paras.deltaRk;//改正过的向径
	this->paras.ik = gpsEphem.i0 + this->paras.deltaIk + gpsEphem.iDot * this->paras.tk;//改正过的轨道倾角
	this->paras.omegak = gpsEphem.omega0 + (gpsEphem.omegaDot - wgs84.omega) * this->paras.tk - wgs84.omega * gpsEphem.toe.secOfWeek;//改正后的升交点经度
	this->paras.xy0[0] = this->paras.rk * cos(this->paras.uk);
	this->paras.xy0[1] = this->paras.rk * sin(this->paras.uk);;//卫星在轨道平面上的位置


	//各参变量对时间的导数
	this->paras.mkDot = this->paras.n;
	this->paras.ekDot = this->paras.mkDot / (1 - gpsEphem.ecc * cos(this->paras.ek));
	this->paras.vkDot = sqrt(1 - gpsEphem.ecc * gpsEphem.ecc) * this->paras.ekDot / (1 - gpsEphem.ecc * cos(this->paras.ek));
	this->paras.phikDot = this->paras.vkDot;

	this->paras.deltaUkDot = 2 * this->paras.phikDot * (gpsEphem.cus * cos(2 * this->paras.phik) - gpsEphem.cuc * sin(2 * this->paras.phik));
	this->paras.deltaRkDot = 2 * this->paras.phikDot * (gpsEphem.crs * cos(2 * this->paras.phik) - gpsEphem.crc * sin(2 * this->paras.phik));
	this->paras.deltaIkDot = 2 * this->paras.phikDot * (gpsEphem.cis * cos(2 * this->paras.phik) - gpsEphem.cic * sin(2 * this->paras.phik));

	this->paras.omegakDot = gpsEphem.omegaDot - wgs84.omega;
	this->paras.ikDot = gpsEphem.iDot + this->paras.deltaIkDot;
	this->paras.rkDot = this->paras.A * gpsEphem.ecc * this->paras.ekDot * sin(this->paras.ek) + this->paras.deltaRkDot;
	this->paras.ukDot = this->paras.phikDot + this->paras.deltaUkDot;

	//卫星在轨道平面内的速度
	this->paras.xkDot = this->paras.rkDot * cos(this->paras.uk) - this->paras.rk * this->paras.ukDot * sin(this->paras.uk);
	this->paras.ykDot = this->paras.rkDot * sin(this->paras.uk) + this->paras.rk * this->paras.ukDot * cos(this->paras.uk);
}

void SatPositioning::CalculateParas(const BDSTIME t/*卫星钟表面时*/, const BDSEPHEM& bdsEphem)
{
	CGCS2000 cgcs2000;
	//中间计算量求解
	this->paras.A = bdsEphem.A;//GPS轨道长半轴
	this->paras.n0 = sqrt(cgcs2000.GM / (this->paras.A * this->paras.A * this->paras.A));//平均运动角速度
	this->paras.tk = SoWSubtraction(t.secOfWeek, bdsEphem.toe.secOfWeek);//相对于星历参考历元的时间
	this->paras.n = this->paras.n0 + bdsEphem.deltaN;//对平均运动角速度进行改正
	this->paras.mk = bdsEphem.m0 + this->paras.n * this->paras.tk;//平近点角
	this->paras.mk = this->paras.mk > 0 ? this->paras.mk : this->paras.mk + 2 * constant::pi;
	this->paras.ek = 0;//偏近点角，迭代求解
	double ek1 = this->paras.mk + bdsEphem.ecc * sin(this->paras.mk);
	for (int i = 0; i < 20 && fabs(this->paras.ek - ek1) > 1e-15; i++)
	{
		this->paras.ek = ek1;
		ek1 = this->paras.mk + bdsEphem.ecc * sin(this->paras.ek);
	}
	this->paras.vk = atan2(sqrt(1 - bdsEphem.ecc * bdsEphem.ecc) * sin(this->paras.ek), cos(this->paras.ek) - bdsEphem.ecc);//真近点角
	this->paras.phik = this->paras.vk + bdsEphem.omega;//升交角距(未经改正)
	//计算二阶调和改正数
	this->paras.deltaUk = bdsEphem.cus * sin(2 * this->paras.phik) + bdsEphem.cuc * cos(2 * this->paras.phik);//计算升交角距的改正数
	this->paras.deltaRk = bdsEphem.crs * sin(2 * this->paras.phik) + bdsEphem.crc * cos(2 * this->paras.phik);//计算向径的改正数
	this->paras.deltaIk = bdsEphem.cis * sin(2 * this->paras.phik) + bdsEphem.cic * cos(2 * this->paras.phik);//计算轨道倾角改证数

	//计算经过改正的升交角距，向径和轨道倾角
	this->paras.uk = this->paras.phik + this->paras.deltaUk;//改正过的升交角距
	this->paras.rk = this->paras.A * (1 - bdsEphem.ecc * cos(this->paras.ek)) + this->paras.deltaRk;//改正过的向径
	this->paras.ik = bdsEphem.i0 + this->paras.deltaIk + bdsEphem.iDot * this->paras.tk;//改正过的轨道倾角
	this->paras.omegak = bdsEphem.omega0 + (bdsEphem.omegaDot - cgcs2000.omega) * this->paras.tk - cgcs2000.omega * bdsEphem.toe.secOfWeek;//改正后的升交点经度
	this->paras.xy0[0] = this->paras.rk * cos(this->paras.uk);
	this->paras.xy0[1] = this->paras.rk * sin(this->paras.uk);;//卫星在轨道平面上的位置


	//各参变量对时间的导数
	this->paras.mkDot = this->paras.n;
	this->paras.ekDot = this->paras.mkDot / (1 - bdsEphem.ecc * cos(this->paras.ek));
	this->paras.vkDot = sqrt(1 - bdsEphem.ecc * bdsEphem.ecc) * this->paras.ekDot / (1 - bdsEphem.ecc * cos(this->paras.ek));
	this->paras.phikDot = this->paras.vkDot;

	this->paras.deltaUkDot = 2 * this->paras.phikDot * (bdsEphem.cus * cos(2 * this->paras.phik) - bdsEphem.cuc * sin(2 * this->paras.phik));
	this->paras.deltaRkDot = 2 * this->paras.phikDot * (bdsEphem.crs * cos(2 * this->paras.phik) - bdsEphem.crc * sin(2 * this->paras.phik));
	this->paras.deltaIkDot = 2 * this->paras.phikDot * (bdsEphem.cis * cos(2 * this->paras.phik) - bdsEphem.cic * sin(2 * this->paras.phik));

	this->paras.omegakDot = bdsEphem.omegaDot - cgcs2000.omega;
	this->paras.ikDot = bdsEphem.iDot + this->paras.deltaIkDot;
	this->paras.rkDot = this->paras.A * bdsEphem.ecc * this->paras.ekDot * sin(this->paras.ek) + this->paras.deltaRkDot;
	this->paras.ukDot = this->paras.phikDot + this->paras.deltaUkDot;

	//卫星在轨道平面内的速度
	this->paras.xkDot = this->paras.rkDot * cos(this->paras.uk) - this->paras.rk * this->paras.ukDot * sin(this->paras.uk);
	this->paras.ykDot = this->paras.rkDot * sin(this->paras.uk) + this->paras.rk * this->paras.ukDot * cos(this->paras.uk);
}


//GPS
void SatPositioning::CalGps(const GPSTIME t, const GPSEPHEM& gpsEphem)
{
	GpsPosVel(t, gpsEphem);
	GpsClockBias(t, this->paras.ek, gpsEphem);
	GpsClockRate(t, this->paras.ek, this->paras.ekDot, gpsEphem);
}

void SatPositioning::GpsPosVel(const GPSTIME t/*卫星钟表面时*/, const GPSEPHEM& gpsEphem)
{
	/////////////////////////////////
	//		GPS卫星位置速度计算
	/////////////////////////////////
	CalculateParas(t, gpsEphem);
	//GPS卫星在地球地固坐标系中的位置计算
	this->satXyz.x = this->paras.xy0[0] * cos(this->paras.omegak) - this->paras.xy0[1] * cos(this->paras.ik) * sin(this->paras.omegak);
	this->satXyz.y = this->paras.xy0[0] * sin(this->paras.omegak) + this->paras.xy0[1] * cos(this->paras.ik) * cos(this->paras.omegak);
	this->satXyz.z = this->paras.xy0[1] * sin(this->paras.ik);
	
	//GPS卫星在地球地固坐标系中的速度计算
	this->satV[0] = this->paras.xkDot * cos(this->paras.omegak) - this->satXyz.y * this->paras.omegakDot - (this->paras.ykDot * cos(this->paras.ik) - this->satXyz.z * this->paras.ikDot) * sin(this->paras.omegak);
	this->satV[1] = this->paras.xkDot * sin(this->paras.omegak) + this->satXyz.x * this->paras.omegakDot + (this->paras.ykDot * cos(this->paras.ik) - this->satXyz.z * this->paras.ikDot) * cos(this->paras.omegak);
	this->satV[2] = this->paras.ykDot * sin(this->paras.ik) + this->paras.xy0[1] * this->paras.ikDot * cos(this->paras.ik);
}

void SatPositioning::GpsClockBias(const GPSTIME t/*卫星钟表面时*/, double ek, const GPSEPHEM& gpsEphem)
{
	WGS84 wgs84;//椭球参数
	double F = -2.0 * sqrt(wgs84.GM) / constant::c / constant::c;
	double delta_tr = F * gpsEphem.ecc * sqrt(gpsEphem.A) * sin(ek);//相对论效应误差改正项
	double delta_t = SoWSubtraction(t.secOfWeek, gpsEphem.toc.secOfWeek);
	this->clkBias = gpsEphem.af[0] + gpsEphem.af[1] * delta_t + gpsEphem.af[2] * delta_t * delta_t + delta_tr /*- tgd*/;
}

void SatPositioning::GpsClockRate(const GPSTIME t, double ek, double ekDot, const GPSEPHEM& gpsEphem)
{
	WGS84 wgs84;//椭球参数
	double F = -2.0 * sqrt(wgs84.GM) / constant::c / constant::c;//相对论效应误差改正
	double delta_trDot = F * gpsEphem.ecc * sqrt(gpsEphem.A) * cos(ek) * ekDot;
	this->clkRate = gpsEphem.af[1] + 2 * gpsEphem.af[2] * SoWSubtraction(t.secOfWeek, gpsEphem.toc.secOfWeek) + delta_trDot;
}

bool SatPositioning::GpsOod(const GPSTIME t, const GPSEPHEM& gpsEphem)
{
	if (fabs(t - gpsEphem.toe) > 7200.0)
		//星历过期
		return false;
	else 
		return true;
}

//BDS
void SatPositioning::CalBds(const GPSTIME t, const BDSEPHEM& bdsEphem)
{
	BDSTIME bdst = GpsTime2BdsTime(t);

	BdsPosVel(bdst, bdsEphem);
	BdsClockBias(bdst, this->paras.ek, bdsEphem);
	BdsClockRate(bdst, this->paras.ek, this->paras.ekDot, bdsEphem);
}

void SatPositioning::BdsPosVel(const BDSTIME t, const BDSEPHEM& bdsEphem)
{
	/////////////////////////////////
	//		BDS卫星位置速度计算
	/////////////////////////////////

	CGCS2000 cgcs2000;
	CalculateParas(t, bdsEphem);
	//BDS MEO/IGSO卫星在BDCS坐标系中的位置和速度计算
	if (bdsEphem.satId > 5 && bdsEphem.satId < 59)
	{

		//BDS MEO/IGSO卫星在BDCS坐标系中的位置计算
		this->satXyz.x = this->paras.xy0[0] * cos(this->paras.omegak) - this->paras.xy0[1] * cos(this->paras.ik) * sin(this->paras.omegak);
		this->satXyz.y = this->paras.xy0[0] * sin(this->paras.omegak) + this->paras.xy0[1] * cos(this->paras.ik) * cos(this->paras.omegak);
		this->satXyz.z = this->paras.xy0[1] * sin(this->paras.ik);

		//BDS MEO/IGSO卫星在BDCS坐标系中的速度计算
		this->satV[0] = this->paras.xkDot * cos(this->paras.omegak) - this->paras.xy0[0] * sin(this->paras.omegak) * this->paras.omegakDot - this->paras.ykDot * cos(this->paras.ik) * sin(this->paras.omegak) + this->paras.xy0[1] * cos(this->paras.ik) * sin(this->paras.omegak) * this->paras.ikDot - this->paras.xy0[1] * cos(this->paras.ik) * cos(this->paras.omegak) * this->paras.omegakDot;
		this->satV[1] = this->paras.xkDot * sin(this->paras.omegak) + this->paras.xy0[0] * cos(this->paras.omegak) * this->paras.omegakDot + this->paras.ykDot * cos(this->paras.ik) * cos(this->paras.omegak) - this->paras.xy0[1] * sin(this->paras.ik) * sin(this->paras.omegak) * this->paras.ikDot - this->paras.xy0[1] * cos(this->paras.ik) * sin(this->paras.omegak) * this->paras.omegakDot;
		this->satV[2] = this->paras.ykDot * sin(this->paras.ik) + this->paras.xy0[1] * sin(this->paras.ik) * this->paras.ikDot;
	}
	//BDS MEO卫星在BDCS坐标系中的位置和速度计算
	else if ((bdsEphem.satId > 0 && bdsEphem.satId <= 5) || (bdsEphem.satId >= 59 && bdsEphem.satId <= 61))
	{
		this->paras.omegak = bdsEphem.omega0 + bdsEphem.omegaDot * this->paras.tk - cgcs2000.omega * bdsEphem.toe.secOfWeek;

		//BDS GEO卫星在BDCS坐标系中的位置计算
		//GEO卫星在自定义坐标系中的坐标
		CMatrix XyzGK(3, 1);
		XyzGK.mat[0] = this->paras.xy0[0] * cos(this->paras.omegak) - this->paras.xy0[1] * cos(this->paras.ik) * sin(this->paras.omegak);
		XyzGK.mat[1] = this->paras.xy0[0] * sin(this->paras.omegak) + this->paras.xy0[1] * cos(this->paras.ik) * cos(this->paras.omegak);
		XyzGK.mat[2] = this->paras.xy0[1] * sin(this->paras.ik);

		CMatrix Rx(3, 3);
		double xRad = -5.0 * constant::pi / 180.0;
		Rx.mat[0] = 1; Rx.mat[1] = 0; Rx.mat[2] = 0;
		Rx.mat[3] = 0; Rx.mat[4] = cos(xRad); Rx.mat[5] = sin(xRad);
		Rx.mat[6] = 0; Rx.mat[7] = -sin(xRad); Rx.mat[8] = cos(xRad);

		CMatrix Rz(3, 3);
		double zRad = cgcs2000.omega * this->paras.tk;
		Rz.mat[0] = cos(zRad); Rz.mat[1] = sin(zRad); Rz.mat[2] = 0;
		Rz.mat[3] = -sin(zRad); Rz.mat[4] = cos(zRad); Rz.mat[5] = 0;
		Rz.mat[6] = 0; Rz.mat[7] = 0; Rz.mat[8] = 1;

		//GEO卫星在BDCS坐标系中的坐标
		CMatrix xyzMat(3, 3);
		xyzMat = Rz * (Rx * XyzGK);
		memcpy(&this->satXyz, xyzMat.mat, sizeof(XYZ));


		//BDS GEO卫星在BDCS坐标系中的速度计算
		//GEO卫星在自定义坐标系中的速度
		this->paras.omegakDot = bdsEphem.omegaDot;

		CMatrix vGK(3, 1);
		vGK.mat[0] = this->paras.xkDot * cos(this->paras.omegak) - this->paras.xy0[0] * sin(this->paras.omegak) * this->paras.omegakDot - this->paras.ykDot * cos(this->paras.ik) * sin(this->paras.omegak) + this->paras.xy0[1] * cos(this->paras.ik) * sin(this->paras.omegak) * this->paras.ikDot - this->paras.xy0[1] * cos(this->paras.ik) * cos(this->paras.omegak) * this->paras.omegakDot;
		vGK.mat[1] = this->paras.xkDot * sin(this->paras.omegak) + this->paras.xy0[0] * cos(this->paras.omegak) * this->paras.omegakDot + this->paras.ykDot * cos(this->paras.ik) * cos(this->paras.omegak) - this->paras.xy0[1] * sin(this->paras.ik) * sin(this->paras.omegak) * this->paras.ikDot - this->paras.xy0[1] * cos(this->paras.ik) * sin(this->paras.omegak) * this->paras.omegakDot;
		vGK.mat[2] = this->paras.ykDot * sin(this->paras.ik) + this->paras.xy0[1] * sin(this->paras.ik) * this->paras.ikDot;

		CMatrix RzDot(3, 3);
		RzDot.mat[0] = -sin(zRad); RzDot.mat[1] = cos(zRad); RzDot.mat[2] = 0;
		RzDot.mat[3] = -cos(zRad); RzDot.mat[4] = -sin(zRad); RzDot.mat[5] = 0;
		RzDot.mat[6] = 0; RzDot.mat[7] = 0; RzDot.mat[8] = 0;
		RzDot = RzDot * cgcs2000.omega;


		//GEO卫星在BDCS坐标系中的速度
		CMatrix vMat(3, 1);
		vMat = RzDot * (Rx * XyzGK) + Rz * (Rx * vGK);
		memcpy(&this->satV, vMat.mat, 3 * sizeof(double));
	}
}

void SatPositioning::BdsClockBias(const BDSTIME t, double ek, const BDSEPHEM& bdsEphem)
{
	CGCS2000 cgcs2000;//椭球参数
	double F = -2.0 * sqrt(cgcs2000.GM) / constant::c / constant::c;
	double delta_tr = F * bdsEphem.ecc * bdsEphem.rootA * sin(ek);//相对论效应误差改正项
	double delta_t = SoWSubtraction(t.secOfWeek, bdsEphem.toc);
	this->clkBias = bdsEphem.a[0] + bdsEphem.a[1] * delta_t + bdsEphem.a[2] * delta_t * delta_t + delta_tr /*- tgd*/;
}

void SatPositioning::BdsClockRate(const BDSTIME t, double ek, double ekDot, const BDSEPHEM& bdsEphem)
{
	CGCS2000 cgcs2000;//椭球参数
	double F = -2.0 * sqrt(cgcs2000.GM) / constant::c / constant::c;//相对论效应误差改正
	double delta_trDot = F * bdsEphem.ecc * bdsEphem.rootA * cos(ek) * ekDot;
	this->clkRate = bdsEphem.a[1] + 2 * bdsEphem.a[2] * SoWSubtraction(t.secOfWeek, 1e-3 * bdsEphem.toc) + delta_trDot;
}

bool SatPositioning::BdsOod(const GPSTIME t, const  BDSEPHEM& bdsEphem)
{
	BDSTIME t0 = GpsTime2BdsTime(t);
	if (fabs(t0 - bdsEphem.toe) > 3600.0)
		//星历过期
		return false;
	else
		return true;
}





