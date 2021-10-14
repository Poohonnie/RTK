#pragma once
#include "TimeConversion.h"
#include "CoordinateConversion.hpp"
#include "Constant.h"
#include <fstream>

constexpr auto MAXCHANNELNUM = 36;
constexpr auto MAXGPSNUM = 32;
constexpr auto MAXBDSNUM = 63;
constexpr auto POLYCRC32 = 0xEDB88320u;//CRC32 polynomial
constexpr auto CRC32_POLYNOMIAL = 0xEDB88320L;

struct SATOBS
{
	//单个卫星的观测值
	GNSS sys;//卫星所属类型
	short prn;//卫星编号
	double P[2];//双频伪距，GPS为L1，L2；北斗为B1，B3
	double L[2];//双频载波相位
	double D[2];//多普勒频移
	double cnr;//载噪比
	double psrSigma[2];//伪距精度
	double cpSigma[2];//载波精度
	bool Valid;//通道状态
};

struct EPCOBS
{
	//一段时间内全部观测卫星的数据
	GPSTIME t;
	int satNum;//卫星总数
	SATOBS satObs[MAXCHANNELNUM];//单个历元所有卫星的观测值 
};

struct GPSEPHEM
{
	//GPS卫星星历
	unsigned long prn;//卫星编号
	double tow;
	unsigned long health;//卫星健康状态
	unsigned long iode[2];//issue of ephemeris data
	GPSTIME toe;//GPS时  包含week 和 toe
	double A;//长半轴 A
	double deltaN;//Mean anomaly correction (rad/sec)
	double m0;//Mean anomaly at reference time (semicircles)
	double ecc;//Eccentricity

	double omega;//近地点角距
	double omega0;//升交点经度
	double omegaDot;//赤经率

	double cuc, cus;//谐波修正项 单位
	double crc, crs;//谐波修正项 单位rad   m
	double cic, cis;//谐波修正项 单位

	double i0, iDot;//倾角 磁倾角变化率
	unsigned long iodc;//issue of data clock BDS为AODC
	double toc;//SV clock correction term
	double tgd;//Equipment group delay differential
	double af[3];//时钟改正值 单位s s/s s/s²
	double N;//corrected mean motion

	bool as;//Anti-spoofing
	double ura;//用户测距精度
};

struct BDSEPHEM
{
	//BDS卫星星历
	unsigned long satId;//卫星编号
	BDSTIME toe;//GPS时  包含week 和 toe
	double ura;//用户测距精度
	unsigned long health;//卫星健康状态
	double tgd[2];//Equipment group delay differential
	unsigned long aodc;//age of data, clock
	unsigned long toc;//reference time of clock parameters
	double a[3];//时钟改正值 单位s s/s s/s²

	unsigned long aode;//age of data, ephemeris
	double rootA;//长半轴的根
	double ecc;//Eccentricity
	double omega;//近地点角距
	double omega0;//升交点经度
	double omegaDot;//赤经率

	double deltaN;//Mean anomaly correction (rad/sec)
	double m0;//Mean anomaly at reference time (semicircles)

	double i0, iDot;//倾角 磁倾角变化率

	double cuc, cus;//谐波修正项 单位
	double crc, crs;//谐波修正项 单位rad   m
	double cic, cis;//谐波修正项 单位

};

struct PSRPOS
{
	//伪距单点定位位置信息
	GPSTIME time;
	BLH blh;
	double ura;//用户测距精度
};

class CDecode
{
protected:
	EPCOBS epcObs;//一段时间内全部观测卫星的数据
	GPSEPHEM gpsEphem;//GPS卫星星历
	BDSEPHEM bdsEphem;//BDS卫星星历
	PSRPOS psrPos;//伪距单点定位位置信息

public:
	friend class SatPositioning;//声明友元类，方便数据解算

	int DecodeOem719Msg(FILE* fp);//文件解码
	void DecodeOem719Obs(unsigned char* buf);//MsgID 43 range 解码
	void DecodeOem719GpsEphem(unsigned char* buf);//MsgID 7 GPS ephemeris解码
	void DecodeOem719BdsEphem(unsigned char* buf);//MsgID 1696 BDS ephemeris解码
	void DecodeOem719Psrpos(unsigned char* buf);//MsgID 47 PSRPOS解码

	int FindSatObsIndex(const int prn, const GNSS sys);

	//NovAtel CRC校验程序
	unsigned int crc32(const unsigned char* buf, int lenth);
	unsigned long CRC32Value(int i);
	unsigned long CalculateBloackCRC32(unsigned long ulCount, unsigned char* ucBuffer);

	unsigned short U2(unsigned char* buf);//读取两个字节
	unsigned int U4(unsigned char* buf);//读取四个字节
	double R8(unsigned char* buf);//读取八个字节

	//外部接口
	EPCOBS EpcObs();//一段时间内全部观测卫星的数据  外部接口
	GPSEPHEM GpsEphem();//GPS卫星星历  外部接口
	BDSEPHEM BdsEphem();//BDS卫星星历  外部接口
	PSRPOS PsrPos();//伪距单点定位位置信息  外部接口

};

class CFileDecode :public CDecode
{
protected:

public:
	FILE* FileRead(const char*);

};