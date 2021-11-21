#include "SatPositioning.hpp"


void SatPositioning::CalculateParas(const GPSTIME t/*�����ӱ���ʱ*/, const GPSEPHEM& gpsEphem)
{
	WGS84 wgs84;
	//�м���������
	this->paras.A = gpsEphem.A;//GPS���������
	this->paras.n0 = sqrt(wgs84.GM / (this->paras.A * this->paras.A * this->paras.A));//ƽ���˶����ٶ�
	this->paras.tk = SoWSubtraction(t.secOfWeek, gpsEphem.toe.secOfWeek);//����������ο���Ԫ��ʱ��
	this->paras.n = this->paras.n0 + gpsEphem.deltaN;//��ƽ���˶����ٶȽ��и���
	this->paras.mk = gpsEphem.m0 + this->paras.n * this->paras.tk;//ƽ�����
	this->paras.mk = this->paras.mk > 0 ? this->paras.mk : this->paras.mk + 2 * constant::pi;
	this->paras.ek = 0;//ƫ����ǣ��������
	double ek1 = this->paras.mk + gpsEphem.ecc * sin(this->paras.mk);
	for (int i = 0; i < 20 && fabs(this->paras.ek - ek1) > 1e-15; i++)
	{
		this->paras.ek = ek1;
		ek1 = this->paras.mk + gpsEphem.ecc * sin(this->paras.ek);
	}
	this->paras.vk = atan2(sqrt(1 - gpsEphem.ecc * gpsEphem.ecc) * sin(this->paras.ek), cos(this->paras.ek) - gpsEphem.ecc);//������
	this->paras.phik = this->paras.vk + gpsEphem.omega;//�����Ǿ�(δ������)
	//������׵��͸�����
	this->paras.deltaUk = gpsEphem.cus * sin(2 * this->paras.phik) + gpsEphem.cuc * cos(2 * this->paras.phik);//���������Ǿ�ĸ�����
	this->paras.deltaRk = gpsEphem.crs * sin(2 * this->paras.phik) + gpsEphem.crc * cos(2 * this->paras.phik);//�����򾶵ĸ�����
	this->paras.deltaIk = gpsEphem.cis * sin(2 * this->paras.phik) + gpsEphem.cic * cos(2 * this->paras.phik);//��������Ǹ�֤��

	//���㾭�������������Ǿ࣬�򾶺͹�����
	this->paras.uk = this->paras.phik + this->paras.deltaUk;//�������������Ǿ�
	this->paras.rk = this->paras.A * (1 - gpsEphem.ecc * cos(this->paras.ek)) + this->paras.deltaRk;//����������
	this->paras.ik = gpsEphem.i0 + this->paras.deltaIk + gpsEphem.iDot * this->paras.tk;//�������Ĺ�����
	this->paras.omegak = gpsEphem.omega0 + (gpsEphem.omegaDot - wgs84.omega) * this->paras.tk - wgs84.omega * gpsEphem.toe.secOfWeek;//������������㾭��
	this->paras.xy0[0] = this->paras.rk * cos(this->paras.uk);
	this->paras.xy0[1] = this->paras.rk * sin(this->paras.uk);;//�����ڹ��ƽ���ϵ�λ��


	//���α�����ʱ��ĵ���
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

	//�����ڹ��ƽ���ڵ��ٶ�
	this->paras.xkDot = this->paras.rkDot * cos(this->paras.uk) - this->paras.rk * this->paras.ukDot * sin(this->paras.uk);
	this->paras.ykDot = this->paras.rkDot * sin(this->paras.uk) + this->paras.rk * this->paras.ukDot * cos(this->paras.uk);
}

void SatPositioning::CalculateParas(const BDSTIME t/*�����ӱ���ʱ*/, const BDSEPHEM& bdsEphem)
{
	CGCS2000 cgcs2000;
	//�м���������
	this->paras.A = bdsEphem.A;//GPS���������
	this->paras.n0 = sqrt(cgcs2000.GM / (this->paras.A * this->paras.A * this->paras.A));//ƽ���˶����ٶ�
	this->paras.tk = SoWSubtraction(t.secOfWeek, bdsEphem.toe.secOfWeek);//����������ο���Ԫ��ʱ��
	this->paras.n = this->paras.n0 + bdsEphem.deltaN;//��ƽ���˶����ٶȽ��и���
	this->paras.mk = bdsEphem.m0 + this->paras.n * this->paras.tk;//ƽ�����
	this->paras.mk = this->paras.mk > 0 ? this->paras.mk : this->paras.mk + 2 * constant::pi;
	this->paras.ek = 0;//ƫ����ǣ��������
	double ek1 = this->paras.mk + bdsEphem.ecc * sin(this->paras.mk);
	for (int i = 0; i < 20 && fabs(this->paras.ek - ek1) > 1e-15; i++)
	{
		this->paras.ek = ek1;
		ek1 = this->paras.mk + bdsEphem.ecc * sin(this->paras.ek);
	}
	this->paras.vk = atan2(sqrt(1 - bdsEphem.ecc * bdsEphem.ecc) * sin(this->paras.ek), cos(this->paras.ek) - bdsEphem.ecc);//������
	this->paras.phik = this->paras.vk + bdsEphem.omega;//�����Ǿ�(δ������)
	//������׵��͸�����
	this->paras.deltaUk = bdsEphem.cus * sin(2 * this->paras.phik) + bdsEphem.cuc * cos(2 * this->paras.phik);//���������Ǿ�ĸ�����
	this->paras.deltaRk = bdsEphem.crs * sin(2 * this->paras.phik) + bdsEphem.crc * cos(2 * this->paras.phik);//�����򾶵ĸ�����
	this->paras.deltaIk = bdsEphem.cis * sin(2 * this->paras.phik) + bdsEphem.cic * cos(2 * this->paras.phik);//��������Ǹ�֤��

	//���㾭�������������Ǿ࣬�򾶺͹�����
	this->paras.uk = this->paras.phik + this->paras.deltaUk;//�������������Ǿ�
	this->paras.rk = this->paras.A * (1 - bdsEphem.ecc * cos(this->paras.ek)) + this->paras.deltaRk;//����������
	this->paras.ik = bdsEphem.i0 + this->paras.deltaIk + bdsEphem.iDot * this->paras.tk;//�������Ĺ�����
	this->paras.omegak = bdsEphem.omega0 + (bdsEphem.omegaDot - cgcs2000.omega) * this->paras.tk - cgcs2000.omega * bdsEphem.toe.secOfWeek;//������������㾭��
	this->paras.xy0[0] = this->paras.rk * cos(this->paras.uk);
	this->paras.xy0[1] = this->paras.rk * sin(this->paras.uk);;//�����ڹ��ƽ���ϵ�λ��


	//���α�����ʱ��ĵ���
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

	//�����ڹ��ƽ���ڵ��ٶ�
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

void SatPositioning::GpsPosVel(const GPSTIME t/*�����ӱ���ʱ*/, const GPSEPHEM& gpsEphem)
{
	/////////////////////////////////
	//		GPS����λ���ٶȼ���
	/////////////////////////////////
	CalculateParas(t, gpsEphem);
	//GPS�����ڵ���ع�����ϵ�е�λ�ü���
	this->satXyz.x = this->paras.xy0[0] * cos(this->paras.omegak) - this->paras.xy0[1] * cos(this->paras.ik) * sin(this->paras.omegak);
	this->satXyz.y = this->paras.xy0[0] * sin(this->paras.omegak) + this->paras.xy0[1] * cos(this->paras.ik) * cos(this->paras.omegak);
	this->satXyz.z = this->paras.xy0[1] * sin(this->paras.ik);
	
	//GPS�����ڵ���ع�����ϵ�е��ٶȼ���
	this->satV[0] = this->paras.xkDot * cos(this->paras.omegak) - this->satXyz.y * this->paras.omegakDot - (this->paras.ykDot * cos(this->paras.ik) - this->satXyz.z * this->paras.ikDot) * sin(this->paras.omegak);
	this->satV[1] = this->paras.xkDot * sin(this->paras.omegak) + this->satXyz.x * this->paras.omegakDot + (this->paras.ykDot * cos(this->paras.ik) - this->satXyz.z * this->paras.ikDot) * cos(this->paras.omegak);
	this->satV[2] = this->paras.ykDot * sin(this->paras.ik) + this->paras.xy0[1] * this->paras.ikDot * cos(this->paras.ik);
}

void SatPositioning::GpsClockBias(const GPSTIME t/*�����ӱ���ʱ*/, double ek, const GPSEPHEM& gpsEphem)
{
	WGS84 wgs84;//�������
	double F = -2.0 * sqrt(wgs84.GM) / constant::c / constant::c;
	double delta_tr = F * gpsEphem.ecc * sqrt(gpsEphem.A) * sin(ek);//�����ЧӦ��������
	double delta_t = SoWSubtraction(t.secOfWeek, gpsEphem.toc.secOfWeek);
	this->clkBias = gpsEphem.af[0] + gpsEphem.af[1] * delta_t + gpsEphem.af[2] * delta_t * delta_t + delta_tr /*- tgd*/;
}

void SatPositioning::GpsClockRate(const GPSTIME t, double ek, double ekDot, const GPSEPHEM& gpsEphem)
{
	WGS84 wgs84;//�������
	double F = -2.0 * sqrt(wgs84.GM) / constant::c / constant::c;//�����ЧӦ������
	double delta_trDot = F * gpsEphem.ecc * sqrt(gpsEphem.A) * cos(ek) * ekDot;
	this->clkRate = gpsEphem.af[1] + 2 * gpsEphem.af[2] * SoWSubtraction(t.secOfWeek, gpsEphem.toc.secOfWeek) + delta_trDot;
}

bool SatPositioning::GpsOod(const GPSTIME t, const GPSEPHEM& gpsEphem)
{
	if (fabs(t - gpsEphem.toe) > 7200.0)
		//��������
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
	//		BDS����λ���ٶȼ���
	/////////////////////////////////

	CGCS2000 cgcs2000;
	CalculateParas(t, bdsEphem);
	//BDS MEO/IGSO������BDCS����ϵ�е�λ�ú��ٶȼ���
	if (bdsEphem.satId > 5 && bdsEphem.satId < 59)
	{

		//BDS MEO/IGSO������BDCS����ϵ�е�λ�ü���
		this->satXyz.x = this->paras.xy0[0] * cos(this->paras.omegak) - this->paras.xy0[1] * cos(this->paras.ik) * sin(this->paras.omegak);
		this->satXyz.y = this->paras.xy0[0] * sin(this->paras.omegak) + this->paras.xy0[1] * cos(this->paras.ik) * cos(this->paras.omegak);
		this->satXyz.z = this->paras.xy0[1] * sin(this->paras.ik);

		//BDS MEO/IGSO������BDCS����ϵ�е��ٶȼ���
		this->satV[0] = this->paras.xkDot * cos(this->paras.omegak) - this->paras.xy0[0] * sin(this->paras.omegak) * this->paras.omegakDot - this->paras.ykDot * cos(this->paras.ik) * sin(this->paras.omegak) + this->paras.xy0[1] * cos(this->paras.ik) * sin(this->paras.omegak) * this->paras.ikDot - this->paras.xy0[1] * cos(this->paras.ik) * cos(this->paras.omegak) * this->paras.omegakDot;
		this->satV[1] = this->paras.xkDot * sin(this->paras.omegak) + this->paras.xy0[0] * cos(this->paras.omegak) * this->paras.omegakDot + this->paras.ykDot * cos(this->paras.ik) * cos(this->paras.omegak) - this->paras.xy0[1] * sin(this->paras.ik) * sin(this->paras.omegak) * this->paras.ikDot - this->paras.xy0[1] * cos(this->paras.ik) * sin(this->paras.omegak) * this->paras.omegakDot;
		this->satV[2] = this->paras.ykDot * sin(this->paras.ik) + this->paras.xy0[1] * sin(this->paras.ik) * this->paras.ikDot;
	}
	//BDS MEO������BDCS����ϵ�е�λ�ú��ٶȼ���
	else if ((bdsEphem.satId > 0 && bdsEphem.satId <= 5) || (bdsEphem.satId >= 59 && bdsEphem.satId <= 61))
	{
		this->paras.omegak = bdsEphem.omega0 + bdsEphem.omegaDot * this->paras.tk - cgcs2000.omega * bdsEphem.toe.secOfWeek;

		//BDS GEO������BDCS����ϵ�е�λ�ü���
		//GEO�������Զ�������ϵ�е�����
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

		//GEO������BDCS����ϵ�е�����
		CMatrix xyzMat(3, 3);
		xyzMat = Rz * (Rx * XyzGK);
		memcpy(&this->satXyz, xyzMat.mat, sizeof(XYZ));


		//BDS GEO������BDCS����ϵ�е��ٶȼ���
		//GEO�������Զ�������ϵ�е��ٶ�
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


		//GEO������BDCS����ϵ�е��ٶ�
		CMatrix vMat(3, 1);
		vMat = RzDot * (Rx * XyzGK) + Rz * (Rx * vGK);
		memcpy(&this->satV, vMat.mat, 3 * sizeof(double));
	}
}

void SatPositioning::BdsClockBias(const BDSTIME t, double ek, const BDSEPHEM& bdsEphem)
{
	CGCS2000 cgcs2000;//�������
	double F = -2.0 * sqrt(cgcs2000.GM) / constant::c / constant::c;
	double delta_tr = F * bdsEphem.ecc * bdsEphem.rootA * sin(ek);//�����ЧӦ��������
	double delta_t = SoWSubtraction(t.secOfWeek, bdsEphem.toc);
	this->clkBias = bdsEphem.a[0] + bdsEphem.a[1] * delta_t + bdsEphem.a[2] * delta_t * delta_t + delta_tr /*- tgd*/;
}

void SatPositioning::BdsClockRate(const BDSTIME t, double ek, double ekDot, const BDSEPHEM& bdsEphem)
{
	CGCS2000 cgcs2000;//�������
	double F = -2.0 * sqrt(cgcs2000.GM) / constant::c / constant::c;//�����ЧӦ������
	double delta_trDot = F * bdsEphem.ecc * bdsEphem.rootA * cos(ek) * ekDot;
	this->clkRate = bdsEphem.a[1] + 2 * bdsEphem.a[2] * SoWSubtraction(t.secOfWeek, 1e-3 * bdsEphem.toc) + delta_trDot;
}

bool SatPositioning::BdsOod(const GPSTIME t, const  BDSEPHEM& bdsEphem)
{
	BDSTIME t0 = GpsTime2BdsTime(t);
	if (fabs(t0 - bdsEphem.toe) > 3600.0)
		//��������
		return false;
	else
		return true;
}





