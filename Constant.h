#pragma once
#include <cmath>

const double PI = 3.14159265358979323846;//圆周率，小数点后20位
const double C = 2.99792458e+8;//光速c, 单位 m/s

struct CoorSys
{
	double a;//长半轴 单位m
	double b;//短半轴 单位m
	double c;//极曲率半径 单位m
	double e;//第一偏心率
	double eSquare;//第一偏心率的平方
	double e2;//第二偏心率
	double e2Square;//第二偏心率的平方
	double f;//扁率
	double GM;//地心引力常数 单位m³s^-2
	double omega;//自转角速度 单位rad/s
	CoorSys();
};

struct CGCS2000:public CoorSys
{
	CGCS2000();
};

struct WGS84:public CoorSys
{
	WGS84();
};

enum class GNSS { GPS, BDS, GLONASS, Galileo, QZSS };
