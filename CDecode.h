#pragma once
#pragma comment(lib,"WS2_32.lib")
#pragma warning(disable:4996)

#include "TimeConversion.h"
#include "CoordinateConversion.hpp"
#include "Constant.h"
#include <fstream>
#include <windows.h>

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
	bool valid = false;//通道状态

	SATOBS()
	{
		memset(this, sizeof(SATOBS), 0);
	};
	void check();//检测双频伪距和相位数据是否完整有效
};

struct EPKOBS
{
	//一段时间内全部观测卫星的数据
	GPSTIME t;
	int satNum;//卫星总数
	SATOBS satObs[MAXCHANNELNUM];//单个历元所有卫星的观测值 

	EPKOBS()
	{
		memset(this, sizeof(EPKOBS), 0);
	}
	int FindSatObsIndex(const int prn, const GNSS sys);//搜索某个prn号的卫星在epkObs中的下标
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
	double ecc;//卫星轨道偏心率

	double omega;//近地点角距
	double omega0;//升交点经度
	double omegaDot;//赤经率

	double cuc, cus;//谐波修正项 单位
	double crc, crs;//谐波修正项 单位rad   m
	double cic, cis;//谐波修正项 单位

	double i0, iDot;//倾角 磁倾角变化率
	unsigned long iodc;//issue of data clock BDS为AODC
	GPSTIME toc;//SV clock correction term
	double tgd;//Equipment group delay differential
	double af[3];//时钟改正值 单位s s/s s/s²
	double N;//corrected mean motion

	bool as;//Anti-spoofing
	double ura;//用户测距精度
	GPSEPHEM()
	{
		memset(this, sizeof(GPSEPHEM), 0);
	}
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
	double A;
	double ecc;//卫星轨道偏心率
	double omega;//近地点角距
	double omega0;//升交点经度
	double omegaDot;//赤经率

	double deltaN;//Mean anomaly correction (rad/sec)
	double m0;//Mean anomaly at reference time (semicircles)

	double i0, iDot;//倾角 磁倾角变化率

	double cuc, cus;//谐波修正项 单位rad   m
	double crc, crs;//谐波修正项 单位
	double cic, cis;//谐波修正项 单位

	BDSEPHEM()
	{
		memset(this, sizeof(BDSEPHEM), 0);
	}
	bool isGeo() const;//判断是否为GEO
};

struct BESTPOS
{
	//伪距单点定位位置信息
	GPSTIME time;
	BLH blh;
	double ura;//用户测距精度
	BESTPOS()
	{
		memset(this, sizeof(BESTPOS), 0);
	}
};

struct RAWDATA
{
	EPKOBS epkObs;
	GPSEPHEM gpsEphem[MAXGPSNUM];
	BDSEPHEM bdsEphem[MAXBDSNUM];
	BESTPOS bestPos;
};

class CDecode
{
protected:
	GPSTIME t;
	RAWDATA raw;

public:
	void Reset();
	friend class Client;

	void DecodeOem719Obs(unsigned char* buf);//MsgID 43 range 解码
	void DecodeOem719GpsEphem(unsigned char* buf);//MsgID 7 GPS ephemeris解码
	void DecodeOem719BdsEphem(unsigned char* buf);//MsgID 1696 BDS ephemeris解码
	void DecodeOem719Bestpos(unsigned char* buf);//MsgID 47 PSRPOS解码


	//NovAtel CRC校验程序
	unsigned int crc32(const unsigned char* buf, int lenth);

	unsigned short U2(unsigned char* buf);//读取两个字节
	unsigned int U4(unsigned char* buf);//读取四个字节
	float R4(unsigned char* buf);//读取四个字节
	double R8(unsigned char* buf);//读取八个字节

	//外部接口
	RAWDATA Raw();//SPP原始数据 外部接口

};

class CFileDecode :public CDecode
{
protected:

public:
	FILE* FileRead(const char*);
	int DecodeOem719Msg(FILE* fp);//文件解码

	CFileDecode()
	{
		memset(this, 0, sizeof(CFileDecode));
	}

};

class CSocketDecode : public CDecode
{
protected:

public:
	int DecodeOem719Msg(unsigned char* buf, int curRem, int& lastRem);//解码



	bool OpenSocket(SOCKET& sock, const char IP[], const unsigned short Port);//打开套接字
	void CloseSocket(SOCKET& sock);//关闭套接字

	CSocketDecode()
	{
		memset(this, 0, sizeof(CSocketDecode));
	}
};