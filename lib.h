#pragma once
#include <cmath>

namespace constant
{
	const double pi = 3.14159265358979323846;//圆周率，小数点后20位
	const double c = 2.99792458e+8;//光速c, 单位 m/s
}

enum class GNSS { GPS, BDS, GLONASS, Galileo, QZSS };

struct CoorSys
{
	double a;//长半轴 单位m
	double b;//短半轴 单位m
	double eSquare;//第一偏心率的平方
	double e2Square;//第二偏心率的平方
	double f;//扁率
	double GM;//地心引力常数 单位m³s^-2
	double omega;//自转角速度 单位rad/s
};

//两个大地坐标系统参数
CoorSys wgs84
{
    6378137.0,
    6356752.3142,
    0.00669437999013,
    0.00673949674227,
    1 / 298.257223563,
    3.986004418e+14,
    7.292115e-5
};

CoorSys cgcs2000
{
    6378137.0,
    6356752.3142,
    0.00669437999013,
    0.00673949674227,
    1 / 298.257223563,
    3.986004418e+14,
    7.292115e-5
};

/***********************************************************************
 *                              时间系统
 ***********************************************************************/

struct COMMONTIME
{
    short year;
    unsigned short month;
    unsigned short day;
    unsigned short hour;
    unsigned short minute;
    double second;
};

struct MJDTIME
{
    int day;
    double fracDay;//简化儒略日的小数部分(天)
    double secOfDay;//一天内的具体秒数
};

struct GPSTIME
{
    unsigned short week;//GPS周
    double secOfWeek;//GPS周秒
    
    void check();//自检
    double operator-(const GPSTIME& sub) const;
    GPSTIME operator-(double sub) const;
};

struct BDSTIME : public GPSTIME
{
};

double SoWSubtraction(double t, double toc);//两个GPS周内秒的减法

double CommonTime2UT(const COMMONTIME&);//通用时转世界时

MJDTIME CommonTime2MjdTime(const COMMONTIME&);//通用时转简化儒略日
COMMONTIME MjdTime2CommonTime(const MJDTIME&);//简化儒略日转通用时

GPSTIME MjdTime2GpsTime(const MJDTIME&);//简化儒略日转GPS时
MJDTIME GpsTime2MjdTime(const GPSTIME&);//GPS时转简化儒略日

GPSTIME CommonTime2GpsTime(const COMMONTIME&);//通用时转GPS时
COMMONTIME GpsTime2CommonTime(const GPSTIME&);//GPS时转通用时


BDSTIME GpsTime2BdsTime(const GPSTIME&);//GPS时 转 北斗时
GPSTIME BdsTime2GpsTime(const BDSTIME&);//北斗时 转 GPS时


BDSTIME MjdTime2BdsTime(const MJDTIME&);//简化儒略日转BDS时
MJDTIME BdsTime2MjdTime(const BDSTIME&);//BDS时转简化儒略日

BDSTIME CommonTime2BdsTime(const COMMONTIME&);//通用时转BDS时
COMMONTIME BdsTime2CommonTime(const BDSTIME&);//BDS时转通用时

/***********************************************************************
 *                              坐标系统
 ***********************************************************************/
 
struct XYZ
{
    double x;
    double y;
    double z;
};
struct BLH
{
    double L;//经度 单位为弧度rad
    double B;//纬度 单位为弧度rad
    double H;//高度 单位为米
};

XYZ BLH2XYZ(const BLH&, const CoorSys&);

BLH XYZ2BLH(const XYZ&, const CoorSys&);

double Deg2Rad(double deg, double min, double sec);
