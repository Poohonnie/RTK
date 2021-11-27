#include "SatPositioning.hpp"

PARAS SatPositioning::CalculateParas(const GPSTIME t/*�����ӱ���ʱ*/, const GPSEPHEM& gpsEphem)
{
	WGS84 wgs84;
	PARAS paras;
	//�м���������
	paras.A = gpsEphem.A;//GPS���������
	paras.n0 = sqrt(wgs84.GM / (paras.A * paras.A * paras.A));//ƽ���˶����ٶ�
	paras.tk = SoWSubtraction(t.secOfWeek, gpsEphem.toe.secOfWeek);//����������ο���Ԫ��ʱ��
	paras.n = paras.n0 + gpsEphem.deltaN;//��ƽ���˶����ٶȽ��и���
	paras.mk = gpsEphem.m0 + paras.n * paras.tk;//ƽ�����
	paras.mk = paras.mk > 0 ? paras.mk : paras.mk + 2 * constant::pi;
	paras.ek = 0;//ƫ����ǣ��������
	double ek1 = paras.mk + gpsEphem.ecc * sin(paras.mk);
	for (int i = 0; i < 20 && fabs(paras.ek - ek1) > 1e-15; i++)
	{
		paras.ek = ek1;
		ek1 = paras.mk + gpsEphem.ecc * sin(paras.ek);
	}
	paras.vk = atan2(sqrt(1 - gpsEphem.ecc * gpsEphem.ecc) * sin(paras.ek), cos(paras.ek) - gpsEphem.ecc);//������
	paras.phik = paras.vk + gpsEphem.omega;//�����Ǿ�(δ������)
	//������׵��͸�����
	paras.deltaUk = gpsEphem.cus * sin(2 * paras.phik) + gpsEphem.cuc * cos(2 * paras.phik);//���������Ǿ�ĸ�����
	paras.deltaRk = gpsEphem.crs * sin(2 * paras.phik) + gpsEphem.crc * cos(2 * paras.phik);//�����򾶵ĸ�����
	paras.deltaIk = gpsEphem.cis * sin(2 * paras.phik) + gpsEphem.cic * cos(2 * paras.phik);//��������Ǹ�֤��

	//���㾭�������������Ǿ࣬�򾶺͹�����
	paras.uk = paras.phik + paras.deltaUk;//�������������Ǿ�
	paras.rk = paras.A * (1 - gpsEphem.ecc * cos(paras.ek)) + paras.deltaRk;//����������
	paras.ik = gpsEphem.i0 + paras.deltaIk + gpsEphem.iDot * paras.tk;//�������Ĺ�����
	paras.omegak = gpsEphem.omega0 + (gpsEphem.omegaDot - wgs84.omega) * paras.tk - wgs84.omega * gpsEphem.toe.secOfWeek;//������������㾭��
	paras.xy0[0] = paras.rk * cos(paras.uk);
	paras.xy0[1] = paras.rk * sin(paras.uk);;//�����ڹ��ƽ���ϵ�λ��


	//���α�����ʱ��ĵ���
	paras.mkDot = paras.n;
	paras.ekDot = paras.mkDot / (1 - gpsEphem.ecc * cos(paras.ek));
	paras.vkDot = sqrt(1 - gpsEphem.ecc * gpsEphem.ecc) * paras.ekDot / (1 - gpsEphem.ecc * cos(paras.ek));
	paras.phikDot = paras.vkDot;

	paras.deltaUkDot = 2 * paras.phikDot * (gpsEphem.cus * cos(2 * paras.phik) - gpsEphem.cuc * sin(2 * paras.phik));
	paras.deltaRkDot = 2 * paras.phikDot * (gpsEphem.crs * cos(2 * paras.phik) - gpsEphem.crc * sin(2 * paras.phik));
	paras.deltaIkDot = 2 * paras.phikDot * (gpsEphem.cis * cos(2 * paras.phik) - gpsEphem.cic * sin(2 * paras.phik));

	paras.omegakDot = gpsEphem.omegaDot - wgs84.omega;
	paras.ikDot = gpsEphem.iDot + paras.deltaIkDot;
	paras.rkDot = paras.A * gpsEphem.ecc * paras.ekDot * sin(paras.ek) + paras.deltaRkDot;
	paras.ukDot = paras.phikDot + paras.deltaUkDot;

	//�����ڹ��ƽ���ڵ��ٶ�
	paras.xkDot = paras.rkDot * cos(paras.uk) - paras.rk * paras.ukDot * sin(paras.uk);
	paras.ykDot = paras.rkDot * sin(paras.uk) + paras.rk * paras.ukDot * cos(paras.uk);
	return paras;
}

PARAS SatPositioning::CalculateParas(const BDSTIME t/*�����ӱ���ʱ*/, const BDSEPHEM& bdsEphem)
{
	CGCS2000 cgcs2000;
	PARAS paras;
	//�м���������
	paras.A = bdsEphem.A;//GPS���������
	paras.n0 = sqrt(cgcs2000.GM / (paras.A * paras.A * paras.A));//ƽ���˶����ٶ�
	paras.tk = SoWSubtraction(t.secOfWeek, bdsEphem.toe.secOfWeek);//����������ο���Ԫ��ʱ��
	paras.n = paras.n0 + bdsEphem.deltaN;//��ƽ���˶����ٶȽ��и���
	paras.mk = bdsEphem.m0 + paras.n * paras.tk;//ƽ�����
	paras.mk = paras.mk > 0 ? paras.mk : paras.mk + 2 * constant::pi;
	paras.ek = 0;//ƫ����ǣ��������
	double ek1 = paras.mk + bdsEphem.ecc * sin(paras.mk);
	for (int i = 0; i < 20 && fabs(paras.ek - ek1) > 1e-15; i++)
	{
		paras.ek = ek1;
		ek1 = paras.mk + bdsEphem.ecc * sin(paras.ek);
	}
	paras.vk = atan2(sqrt(1 - bdsEphem.ecc * bdsEphem.ecc) * sin(paras.ek), cos(paras.ek) - bdsEphem.ecc);//������
	paras.phik = paras.vk + bdsEphem.omega;//�����Ǿ�(δ������)
	//������׵��͸�����
	paras.deltaUk = bdsEphem.cus * sin(2 * paras.phik) + bdsEphem.cuc * cos(2 * paras.phik);//���������Ǿ�ĸ�����
	paras.deltaRk = bdsEphem.crs * sin(2 * paras.phik) + bdsEphem.crc * cos(2 * paras.phik);//�����򾶵ĸ�����
	paras.deltaIk = bdsEphem.cis * sin(2 * paras.phik) + bdsEphem.cic * cos(2 * paras.phik);//��������Ǹ�֤��

	//���㾭�������������Ǿ࣬�򾶺͹�����
	paras.uk = paras.phik + paras.deltaUk;//�������������Ǿ�
	paras.rk = paras.A * (1 - bdsEphem.ecc * cos(paras.ek)) + paras.deltaRk;//����������
	paras.ik = bdsEphem.i0 + paras.deltaIk + bdsEphem.iDot * paras.tk;//�������Ĺ�����
	paras.omegak = bdsEphem.omega0 + (bdsEphem.omegaDot - cgcs2000.omega) * paras.tk - cgcs2000.omega * bdsEphem.toe.secOfWeek;//������������㾭��
	paras.xy0[0] = paras.rk * cos(paras.uk);
	paras.xy0[1] = paras.rk * sin(paras.uk);;//�����ڹ��ƽ���ϵ�λ��


	//���α�����ʱ��ĵ���
	paras.mkDot = paras.n;
	paras.ekDot = paras.mkDot / (1 - bdsEphem.ecc * cos(paras.ek));
	paras.vkDot = sqrt(1 - bdsEphem.ecc * bdsEphem.ecc) * paras.ekDot / (1 - bdsEphem.ecc * cos(paras.ek));
	paras.phikDot = paras.vkDot;

	paras.deltaUkDot = 2 * paras.phikDot * (bdsEphem.cus * cos(2 * paras.phik) - bdsEphem.cuc * sin(2 * paras.phik));
	paras.deltaRkDot = 2 * paras.phikDot * (bdsEphem.crs * cos(2 * paras.phik) - bdsEphem.crc * sin(2 * paras.phik));
	paras.deltaIkDot = 2 * paras.phikDot * (bdsEphem.cis * cos(2 * paras.phik) - bdsEphem.cic * sin(2 * paras.phik));

	paras.omegakDot = bdsEphem.omegaDot - cgcs2000.omega;
	paras.ikDot = bdsEphem.iDot + paras.deltaIkDot;
	paras.rkDot = paras.A * bdsEphem.ecc * paras.ekDot * sin(paras.ek) + paras.deltaRkDot;
	paras.ukDot = paras.phikDot + paras.deltaUkDot;

	//�����ڹ��ƽ���ڵ��ٶ�
	paras.xkDot = paras.rkDot * cos(paras.uk) - paras.rk * paras.ukDot * sin(paras.uk);
	paras.ykDot = paras.rkDot * sin(paras.uk) + paras.rk * paras.ukDot * cos(paras.uk);
	return paras;
}


//GPS
void SatPositioning::CalGps(const GPSTIME t, const GPSEPHEM& gpsEphem)
{
	PARAS gParas = CalculateParas(t, gpsEphem);
	GpsPosVel(t, gpsEphem, gParas);
	GpsClockBias(t, gParas.ek, gpsEphem);
	GpsClockRate(t, gParas.ek, gParas.ekDot, gpsEphem);
}

void SatPositioning::GpsPosVel(const GPSTIME t, const GPSEPHEM& gpsEphem, PARAS& gPara)
{
	/////////////////////////////////
	//		GPS����λ���ٶȼ���
	/////////////////////////////////
	//GPS�����ڵ���ع�����ϵ�е�λ�ü���
	this->satXyz.x = gPara.xy0[0] * cos(gPara.omegak) - gPara.xy0[1] * cos(gPara.ik) * sin(gPara.omegak);
	this->satXyz.y = gPara.xy0[0] * sin(gPara.omegak) + gPara.xy0[1] * cos(gPara.ik) * cos(gPara.omegak);
	this->satXyz.z = gPara.xy0[1] * sin(gPara.ik);
	
	//GPS�����ڵ���ع�����ϵ�е��ٶȼ���
	this->satV[0] = gPara.xkDot * cos(gPara.omegak) - this->satXyz.y * gPara.omegakDot - (gPara.ykDot * cos(gPara.ik) - this->satXyz.z * gPara.ikDot) * sin(gPara.omegak);
	this->satV[1] = gPara.xkDot * sin(gPara.omegak) + this->satXyz.x * gPara.omegakDot + (gPara.ykDot * cos(gPara.ik) - this->satXyz.z * gPara.ikDot) * cos(gPara.omegak);
	this->satV[2] = gPara.ykDot * sin(gPara.ik) + gPara.xy0[1] * gPara.ikDot * cos(gPara.ik);
}

void SatPositioning::GpsClockBias(const GPSTIME t, double ek, const GPSEPHEM& gpsEphem)
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
	PARAS bPara = CalculateParas(bdst, bdsEphem);
	BdsPosVel(bdst, bdsEphem, bPara);
	BdsClockBias(bdst, bPara.ek, bdsEphem);
	BdsClockRate(bdst, bPara.ek, bPara.ekDot, bdsEphem);
}

void SatPositioning::BdsPosVel(const BDSTIME t, const BDSEPHEM& bdsEphem, PARAS& bPara)
{
	/////////////////////////////////
	//		BDS����λ���ٶȼ���
	/////////////////////////////////

	CGCS2000 cgcs2000;
	//BDS MEO/IGSO������BDCS����ϵ�е�λ�ú��ٶȼ���
	if (!bdsEphem.isGeo())
	{
		//BDS MEO/IGSO������BDCS����ϵ�е�λ�ü���
		this->satXyz.x = bPara.xy0[0] * cos(bPara.omegak) - bPara.xy0[1] * cos(bPara.ik) * sin(bPara.omegak);
		this->satXyz.y = bPara.xy0[0] * sin(bPara.omegak) + bPara.xy0[1] * cos(bPara.ik) * cos(bPara.omegak);
		this->satXyz.z = bPara.xy0[1] * sin(bPara.ik);

		//BDS MEO/IGSO������BDCS����ϵ�е��ٶȼ���
		this->satV[0] = bPara.xkDot * cos(bPara.omegak) - bPara.xy0[0] * sin(bPara.omegak) * bPara.omegakDot - bPara.ykDot * cos(bPara.ik) * sin(bPara.omegak) + bPara.xy0[1] * cos(bPara.ik) * sin(bPara.omegak) * bPara.ikDot - bPara.xy0[1] * cos(bPara.ik) * cos(bPara.omegak) * bPara.omegakDot;
		this->satV[1] = bPara.xkDot * sin(bPara.omegak) + bPara.xy0[0] * cos(bPara.omegak) * bPara.omegakDot + bPara.ykDot * cos(bPara.ik) * cos(bPara.omegak) - bPara.xy0[1] * sin(bPara.ik) * sin(bPara.omegak) * bPara.ikDot - bPara.xy0[1] * cos(bPara.ik) * sin(bPara.omegak) * bPara.omegakDot;
		this->satV[2] = bPara.ykDot * sin(bPara.ik) + bPara.xy0[1] * sin(bPara.ik) * bPara.ikDot;
	}
	//BDS MEO������BDCS����ϵ�е�λ�ú��ٶȼ���
	else if (bdsEphem.isGeo())
	{
		bPara.omegak = bdsEphem.omega0 + bdsEphem.omegaDot * bPara.tk - cgcs2000.omega * bdsEphem.toe.secOfWeek;

		//BDS GEO������BDCS����ϵ�е�λ�ü���
		//GEO�������Զ�������ϵ�е�����
		CMatrix XyzGK(3, 1);
		XyzGK.mat[0] = bPara.xy0[0] * cos(bPara.omegak) - bPara.xy0[1] * cos(bPara.ik) * sin(bPara.omegak);
		XyzGK.mat[1] = bPara.xy0[0] * sin(bPara.omegak) + bPara.xy0[1] * cos(bPara.ik) * cos(bPara.omegak);
		XyzGK.mat[2] = bPara.xy0[1] * sin(bPara.ik);

		CMatrix Rx(3, 3);
		double xRad = -5.0 * constant::pi / 180.0;
		Rx.mat[0] = 1; Rx.mat[1] = 0; Rx.mat[2] = 0;
		Rx.mat[3] = 0; Rx.mat[4] = cos(xRad); Rx.mat[5] = sin(xRad);
		Rx.mat[6] = 0; Rx.mat[7] = -sin(xRad); Rx.mat[8] = cos(xRad);

		CMatrix Rz(3, 3);
		double zRad = cgcs2000.omega * bPara.tk;
		Rz.mat[0] = cos(zRad); Rz.mat[1] = sin(zRad); Rz.mat[2] = 0;
		Rz.mat[3] = -sin(zRad); Rz.mat[4] = cos(zRad); Rz.mat[5] = 0;
		Rz.mat[6] = 0; Rz.mat[7] = 0; Rz.mat[8] = 1;

		//GEO������BDCS����ϵ�е�����
		CMatrix xyzMat(3, 3);
		xyzMat = Rz * (Rx * XyzGK);
		memcpy(&this->satXyz, xyzMat.mat, sizeof(XYZ));

		//BDS GEO������BDCS����ϵ�е��ٶȼ���
		//GEO�������Զ�������ϵ�е��ٶ�
		bPara.omegakDot = bdsEphem.omegaDot;

		CMatrix vGK(3, 1);
		vGK.mat[0] = bPara.xkDot * cos(bPara.omegak) - bPara.xy0[0] * sin(bPara.omegak) * bPara.omegakDot - bPara.ykDot * cos(bPara.ik) * sin(bPara.omegak) + bPara.xy0[1] * cos(bPara.ik) * sin(bPara.omegak) * bPara.ikDot - bPara.xy0[1] * cos(bPara.ik) * cos(bPara.omegak) * bPara.omegakDot;
		vGK.mat[1] = bPara.xkDot * sin(bPara.omegak) + bPara.xy0[0] * cos(bPara.omegak) * bPara.omegakDot + bPara.ykDot * cos(bPara.ik) * cos(bPara.omegak) - bPara.xy0[1] * sin(bPara.ik) * sin(bPara.omegak) * bPara.ikDot - bPara.xy0[1] * cos(bPara.ik) * sin(bPara.omegak) * bPara.omegakDot;
		vGK.mat[2] = bPara.ykDot * sin(bPara.ik) + bPara.xy0[1] * sin(bPara.ik) * bPara.ikDot;

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
	this->clkRate = bdsEphem.a[1] + 2 * bdsEphem.a[2] * SoWSubtraction(t.secOfWeek, bdsEphem.toc) + delta_trDot;
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





