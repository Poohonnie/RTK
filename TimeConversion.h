#pragma once

struct COMMONTIME
{
	short year;
	unsigned short month;
	unsigned short day;
	unsigned short hour;
	unsigned short minute;
	double second;
	COMMONTIME()
	{
		year = 0;
		month = 0;
		day = 0;
		hour = 0;
		minute = 0;
		second = 0.0;
	}//默认构造函数
};

struct MJDTIME
{
	int day;
	double fracDay;//简化儒略日的小数部分(天)
	double secOfDay;//一天内的具体秒数
	MJDTIME()
	{
		day = 0;
		secOfDay = 0.0;
		fracDay = 0.0;
	}//默认构造函数
};

struct GPSTIME
{
	unsigned short week;//GPS周
	double secOfWeek;//GPS周秒

	GPSTIME()
	{
		week = 0;
		secOfWeek = 0.0;
	}//默认构造函数

	void check();//自检
	double operator-(const GPSTIME& sub) const;//GPS时减法，仅支持大减小
};

struct BDSTIME : public GPSTIME
{

};

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