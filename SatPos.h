#pragma once
#include "lib.h"
#include "CDecode.h"
#include "CMatrix.h"
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
 
	void CalSatE(const XYZ& rcvr/*接收机地心地固坐标*/, CoorSys& coor/*坐标系*/);//卫星高度角的计算
	void Hopefield(const XYZ& rcvr/*接收机地心地固坐标*/, CoorSys& coor/*坐标系*/);//对流层Hopefield模型改正

	SatPositioning()
	{
		memset(this, 0, sizeof(SatPositioning));
	}
};
