#include "SatPositioning.h"

void SatPositioning::GpsPositioning(const GPSEPHEM& gpsEphem)
{
	//GPS����λ�ü���
	WGS84 wgs84;//�������
	double A = gpsEphem.A;//���������
	double n0 = sqrt(wgs84.GM / (A * A * A));//ƽ���˶����ٶ�
	/*****************************
	*		t �ݶ�Ϊ�۲�ʱ��
	*****************************/
	double tk  = 0.0/*= t - gpsEphem.toe*/;//����������ο���Ԫ��ʱ��
	double n = n0 + gpsEphem.deltaN;//��ƽ���˶����ٶȽ��и���
	double mk = gpsEphem.m0 + n * tk;//ƽ�����
	double ek = 0;//ƫ����ǣ��������
	double ek1 = mk + wgs84.e * sin(mk);
	for (int i = 0; i < 20 && fabs(ek - ek1) < 1e-15; i++)
	{
		ek = ek1;
		ek1 = mk + wgs84.e * sin(ek);
	}

	double vk = atan2(sqrt(1 - wgs84.eSquare) * sin(ek) / (1 - wgs84.e * cos(ek)), (cos(ek) - wgs84.e) / (1 - wgs84.e * cos(ek)));//������
	double phik = vk + gpsEphem.omega;//�����Ǿ�
	//������׵��͸�����
	double deltaUk = gpsEphem.cus * sin(2 * phik) + gpsEphem.cuc * cos(2 * phik);//���������Ǿ�ĸ�����
	double deltaRk = gpsEphem.crs * sin(2 * phik) + gpsEphem.crc * cos(2 * phik);//�����򾶵ĸ�����
	double deltaIk = gpsEphem.cis * sin(2 * phik) + gpsEphem.cic * cos(2 * phik);//��������Ǹ�֤��

	//���㾭�������������Ǿ࣬�򾶺͹�����
	double uk = phik + deltaUk;//�������������Ǿ�
	double rk = A * (1 - wgs84.e * cos(ek)) + deltaRk;//����������
	double ik = gpsEphem.i0 + deltaIk + gpsEphem.iDot * tk;//�������Ĺ�����

	double xy0[2] = { rk * cos(uk), rk * sin(uk) };//�����ڹ��ƽ���ϵ�λ��
	double omegak = gpsEphem.omega0 + (gpsEphem.omegaDot - wgs84.omega) * tk - wgs84.omega * gpsEphem.toe.secOfWeek;//������������㾭��

	this->xyzk.x = xy0[0] * cos(omegak) - xy0[1] * cos(ik) * sin(omegak);
	this->xyzk.y = xy0[0] * sin(omegak) + xy0[1] * cos(ik) * cos(omegak);
	this->xyzk.z = xy0[1] * sin(ik);

}
