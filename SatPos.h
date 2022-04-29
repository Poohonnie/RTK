#pragma once

#include "lib.h"
#include "CDecode.h"

struct PARAS
{
	double A{};  // 轨道长半轴
	double n0{};  // 平均运动角速度
	double tk{};  // 相对于星历参考历元的时间
	double n{};  // 对平均运动角速度进行改正
	double mk{};  // 平近点角
	double ek{};  // 偏近点角，迭代求解
	double vk{};  // 真近点角
	double phik{};  // 升交角距
	// 二阶调和改正数
	double deltaUk{};  // 计算升交角距的改正数
	double deltaRk{};  // 计算向径的改正数
	double deltaIk{};  // 计算轨道倾角改证数
	// 计算经过改正的升交角距，向径和轨道倾角
	double uk{};  // 改正过的升交角距
	double rk{};  // 改正过的向径
	double ik{};  // 改正过的轨道倾角
	double omegak{};  // 改正后的升交点经度
	double xy0[2]{};  // 卫星在轨道平面上的位置
 
	// 各参变量对时间的导数
	double mkDot{};
	double ekDot{}, vkDot{}, phikDot{};

	double deltaUkDot{}, deltaRkDot{}, deltaIkDot{};

	double omegakDot{};
	double ikDot{}, rkDot{}, ukDot{};

	// 卫星在轨道平面内的速度
	double xkDot{}, ykDot{};
};

class SatPos
{
protected:
    GNSS sys{};
    unsigned short prn{};
	XYZ satXyz{};  // 卫星在地心地固坐标系下的坐标
	double satV[3]{};  // 卫星运动速度
	double clkBias{};  // 卫星钟差
	double clkRate{};  // 卫星钟速
	double eleAngle{};  // 卫星高度角
	double tropDelay{};  // 对流层延迟
	double obsTime{};  // 观测历元数
 
public:
	friend class SPP;
    friend class RTK;
    friend class EpkPos;
    
    bool valid{};  // 定位结果是否可用 例如高度角限制等等
    
    static PARAS CalParas(GPSTIME t/*卫星钟表面时*/, const EPHEMERIS& ephem);

    void CalSat(GPSTIME t, const EPHEMERIS& ephem);  // 计算卫星相关数据
    void CalPosVel(const EPHEMERIS& ephem, PARAS& para);  // 卫星位置速度计算
    void ClockBias(GPSTIME t, double ek, const EPHEMERIS& ephem);  // 钟差计算
    void ClockRate(GPSTIME t, double ek, double ekDot, const EPHEMERIS& ephem);  // 钟速计算
    static bool Overdue(GPSTIME t, const EPHEMERIS& ephem);  // 判断星历是否过期
    
	void CalSatEl(const XYZ& rcvr/*接收机地心地固坐标*/, CoorSys& coor/*坐标系*/);  // 卫星高度角的计算  rad
	void Hopefield(const XYZ& rcvr/*接收机地心地固坐标*/, CoorSys& coor/*坐标系*/);  // 对流层Hopefield模型改正
 
};

class EpkPos
{
protected:
    unsigned short satNum{};
    SatPos satPos[MAXCHANNELNUM]{};
    
public:
    friend class SPP;
    friend class RTK;
    friend class SDObs;
    
    int FindSatPosIndex(int prn, GNSS sys);  // 查找卫星
};