#pragma once
#include "Constant.h"
#include "CDecode.h"
#include "CMatrix.h"
#include <cmath>
#include <iostream>

struct PARAS
{
	double A{};//轨道长半轴
	double n0{};//平均运动角速度
	double tk{};//相对于星历参考历元的时间
	double n{};//对平均运动角速度进行改正
	double mk{};//平近点角
	double ek{};//偏近点角，迭代求解
	double vk{};//真近点角
	double phik{};//升交角距

	//二阶调和改正数
	double deltaUk{};//计算升交角距的改正数
	double deltaRk{};//计算向径的改正数
	double deltaIk{};//计算轨道倾角改证数

	//计算经过改正的升交角距，向径和轨道倾角
	double uk{};//改正过的升交角距
	double rk{};//改正过的向径
	double ik{};//改正过的轨道倾角
	double omegak{};//改正后的升交点经度
	double xy0[2]{};//卫星在轨道平面上的位置


	//各参变量对时间的导数
	double mkDot{};
	double ekDot{};
	double vkDot{};
	double phikDot{};

	double deltaUkDot{};
	double deltaRkDot{};
	double deltaIkDot{};

	double omegakDot{};
	double ikDot{};
	double rkDot{};
	double ukDot{};

	//卫星在轨道平面内的速度
	double xkDot{};
	double ykDot{};
	
	PARAS()
	{
		memset(this, 0, sizeof(PARAS));
	}
};

class SatPositioning
{
protected:
	XYZ satXyz{};//卫星在地心地固坐标系下的坐标
	double satV[3]{};//卫星运动速度
	double clkBias{};//卫星钟差
	double clkRate{};//卫星钟速
	double eleAngle{};//卫星高度角
	double tropDelay{};//对流层延迟
	
public:
	friend class SPP;
	static PARAS CalculateParas(GPSTIME t/*卫星钟表面时*/, const GPSEPHEM& gpsEphem);
	static PARAS CalculateParas(BDSTIME t/*卫星钟表面时*/, const BDSEPHEM& bdsEphem);

	void CalGps(GPSTIME t/*卫星钟表面时*/, const GPSEPHEM& gpsEphem);
	void GpsPosVel(GPSTIME t/*卫星钟表面时*/, const GPSEPHEM& gpsEphem, PARAS& gPara);//GPS卫星位置计算
	void GpsClockBias(GPSTIME t/*卫星钟表面时*/, double ek, const GPSEPHEM& gpsEphem);//GPS钟差计算
	void GpsClockRate(GPSTIME t/*卫星钟表面时*/, double ek, double ekDot, const GPSEPHEM& gpsEphem);//GPS钟速计算
	static bool GpsOod(GPSTIME t, const GPSEPHEM& gpsEphem);//判断星历过期情况

	void CalBds(GPSTIME t/*卫星钟表面时*/, const BDSEPHEM& bdsEphem);
	void BdsPosVel(BDSTIME t/*卫星钟表面时*/, const BDSEPHEM& bdsEphem, PARAS& bPara);//BDS卫星位置计算
	void BdsClockBias(BDSTIME t/*卫星钟表面时*/, double ek, const BDSEPHEM& bdsEphem);//BDS钟差计算
	void BdsClockRate(BDSTIME t/*卫星钟表面时*/, double ek, double ekDot, const BDSEPHEM& bdsEphem);//BDS钟速计算
	static bool BdsOod(GPSTIME t, const BDSEPHEM& bdsEphem);//判断星历过期情况

	template<typename T>
	void CalSatE(const XYZ& rcvr/*接收机地心地固坐标*/, T& coor/*坐标系*/);//卫星高度角的计算
	template<typename T1>
	void Hopefield(const XYZ& rcvr/*接收机地心地固坐标*/, T1& coor/*坐标系*/);//对流层Hopefield模型改正

	SatPositioning()
	{
		memset(this, 0, sizeof(SatPositioning));
	}
};

template<typename T>
void SatPositioning::CalSatE(const XYZ& rcvrXyz, T& coor)
{
	//卫星高度角计算

	BLH rcvrBlh = XYZ2BLH(rcvrXyz, coor);//接收机的大地坐标
	CMatrix trans(3, 3);//转换矩阵
	CMatrix line(3, 1);//接收机与卫星的连线
	CMatrix satxyz(3, 1);//卫星在站心坐标系中的坐标

	trans.mat[0] = -sin(rcvrBlh.B) * cos(rcvrBlh.L);
	trans.mat[1] = -sin(rcvrBlh.B) * sin(rcvrBlh.L);
	trans.mat[2] = cos(rcvrBlh.B);
	trans.mat[3] = -sin(rcvrBlh.L);
	trans.mat[4] = cos(rcvrBlh.L);
	trans.mat[5] = 0;
	trans.mat[6] = cos(rcvrBlh.B) * cos(rcvrBlh.L);
	trans.mat[7] = cos(rcvrBlh.B) * sin(rcvrBlh.L);
	trans.mat[8] = sin(rcvrBlh.B);
	trans.check();//B=0 L=0 H=0的情况

	line.mat[0] = this->satXyz.x - rcvrXyz.x;
	line.mat[1] = this->satXyz.y - rcvrXyz.y;
	line.mat[2] = this->satXyz.z - rcvrXyz.z;

	satxyz = trans * line;//卫星在站心坐标系中的坐标

	this->eleAngle = atan(satxyz.mat[2] / sqrt(satxyz.mat[0] * satxyz.mat[0] + satxyz.mat[1] * satxyz.mat[1]));//卫星高度角
}

template<typename T1>
void SatPositioning::Hopefield(const XYZ& rcvrXyz, T1& coor)
{
	BLH rcvrBlh = XYZ2BLH(rcvrXyz, coor);
	double H = rcvrBlh.H;//接收机大地高，方便后面书写
	if (H > 1e+4)
	{
		this->tropDelay = 10;
		return ;//测站高度不在对流层范围
	}
	double H0 = 0.0;//m 海平面
	double T0 = 20.0 + 273.16;//K 温度
	double p0 = 1013.25;//mbar 气压
	double RH0 = 0.5;//相对湿度

	double RH = RH0 * exp(-0.0006369 * (H - H0));
	double p = p0 * pow(1 - 2.26e-5 * (H - H0), 5.225);
	double T = T0 - 0.0065 * (H - H0);
	double e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
	double hw = 11000.0;
	double hd = 40136.0 + 148.72 * (T0 - 273.16);
	double Kw = 155.2e-7 * 4810.0 / (T * T) * e * (hw - H);
	double Kd = 155.2e-7 * p / T * (hd - H);

	double E = this->eleAngle * 180.0 / constant::pi;
	double deltaD = Kd / sin(sqrt(E * E + 6.25) / 180.0 * constant::pi);
	double deltaW = Kw / sin(sqrt(E * E + 2.25) / 180.0 * constant::pi);
	this->tropDelay = deltaD + deltaW;
}
