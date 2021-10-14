#include "SatPositioning.h"

void SatPositioning::GpsPositioning(const GPSEPHEM& gpsEphem)
{
	//GPS卫星位置计算
	WGS84 wgs84;//椭球参数
	double A = gpsEphem.A;//轨道长半轴
	double n0 = sqrt(wgs84.GM / (A * A * A));//平均运动角速度
	/*****************************
	*		t 暂定为观测时间
	*****************************/
	double tk  = 0.0/*= t - gpsEphem.toe*/;//相对于星历参考历元的时间
	double n = n0 + gpsEphem.deltaN;//对平均运动角速度进行改正
	double mk = gpsEphem.m0 + n * tk;//平近点角
	double ek = 0;//偏近点角，迭代求解
	double ek1 = mk + wgs84.e * sin(mk);
	for (int i = 0; i < 20 && fabs(ek - ek1) < 1e-15; i++)
	{
		ek = ek1;
		ek1 = mk + wgs84.e * sin(ek);
	}

	double vk = atan2(sqrt(1 - wgs84.eSquare) * sin(ek) / (1 - wgs84.e * cos(ek)), (cos(ek) - wgs84.e) / (1 - wgs84.e * cos(ek)));//真近点角
	double phik = vk + gpsEphem.omega;//升交角距
	//计算二阶调和改正数
	double deltaUk = gpsEphem.cus * sin(2 * phik) + gpsEphem.cuc * cos(2 * phik);//计算升交角距的改正数
	double deltaRk = gpsEphem.crs * sin(2 * phik) + gpsEphem.crc * cos(2 * phik);//计算向径的改正数
	double deltaIk = gpsEphem.cis * sin(2 * phik) + gpsEphem.cic * cos(2 * phik);//计算轨道倾角改证数

	//计算经过改正的升交角距，向径和轨道倾角
	double uk = phik + deltaUk;//改正过的升交角距
	double rk = A * (1 - wgs84.e * cos(ek)) + deltaRk;//改正过的向径
	double ik = gpsEphem.i0 + deltaIk + gpsEphem.iDot * tk;//改正过的轨道倾角

	double xy0[2] = { rk * cos(uk), rk * sin(uk) };//卫星在轨道平面上的位置
	double omegak = gpsEphem.omega0 + (gpsEphem.omegaDot - wgs84.omega) * tk - wgs84.omega * gpsEphem.toe.secOfWeek;//改正后的升交点经度

	this->xyzk.x = xy0[0] * cos(omegak) - xy0[1] * cos(ik) * sin(omegak);
	this->xyzk.y = xy0[0] * sin(omegak) + xy0[1] * cos(ik) * cos(omegak);
	this->xyzk.z = xy0[1] * sin(ik);

}
