#include "lib.h"

double SoWSubtraction(double t, double toc)
{
	double result = t - toc;
	if (result > 302400)
		result -= 604800;
	else if (result < -302400)
		result += 604800;
	return result;
}

double CommonTime2UT(const COMMONTIME& commonTime)
{
	/*以小时为单位的UT*/
	double UT = commonTime.hour + commonTime.minute / 60.0 + commonTime.second / 3600.0;
	return UT;
}

MJDTIME CommonTime2MjdTime(const COMMONTIME& commonTime)
{
	//通用时转简化儒略日
	MJDTIME mjdTime{};
	int y;//年
	int m;//月
	if (commonTime.month <= 2)
	{
		y = commonTime.year - 1;
		m = commonTime.month + 12;
	}
	else
	{
		y = commonTime.year;
		m = commonTime.month;
	}
	double MJD = int(365.25 * y) + int(30.6001 * (m + 1)) + commonTime.day + CommonTime2UT(commonTime) / 24 + 1720981.5 - 2400000.5;//PPT1-4第 9 页MJD计算公式，单位不出意外应该为 天
	mjdTime.day = int(MJD);
	mjdTime.fracDay = MJD - int(MJD);//mjdTime 的小数部分，单位为天
	mjdTime.secOfDay = commonTime.hour * 3600.0 + commonTime.minute * 60.0 + commonTime.second;//mjdTime 一天中的具体秒数
	return mjdTime;
}

COMMONTIME MjdTime2CommonTime(const MJDTIME& mjdTime)
{
	//简化儒略日转通用时
	COMMONTIME commonTime{};//作为返回值
	int a, b, c, d, e;//转换中间参数，见PPT 1-4 第 10 页
	double JD = mjdTime.day + mjdTime.fracDay + 2400000.5;
	a = lround(JD + 0.5); b = a + 1537;
	c = int((b - 122.1) / 365.25);
	d = int(365.25 * c);
	e = int((b - d) / 30.6001);
	double D = b - d - int(30.6001 * e) + JD + 0.5 - lround(JD + 0.5);//中间参数一年中的日，单位天
	int M = e - 1 - 12 * int(e / 14);//月
	int Y = c - 4715 - int((M + 7) / 10);//年

	commonTime.day = (unsigned short)D;//D取整

	commonTime.hour = (unsigned short)(mjdTime.secOfDay / 3600.0);//秒数除以3600再取整，得小时数
	commonTime.minute = (unsigned short)((mjdTime.secOfDay - commonTime.hour * 3600.0) / 60.0);//一小时内剩余秒数除以60取整，得分钟数
	commonTime.second = mjdTime.secOfDay - commonTime.hour * 3600.0 - commonTime.minute * 60.0;//秒数

	commonTime.month = (unsigned char)M;
	commonTime.year = short(Y);

	return commonTime;
}

GPSTIME MjdTime2GpsTime(const MJDTIME& mjdTime)
{
	//简化儒略日转GPS时
	GPSTIME gpsTime{};
	gpsTime.week = (unsigned short)((mjdTime.day - 44244) / 7);
	gpsTime.secOfWeek = int(mjdTime.day - 44244 - int((mjdTime.day - 44244) / 7) * 7)/*这是算出GPS周内天数的整数部分*/ * 86400 + mjdTime.secOfDay;
	return gpsTime;
}

MJDTIME GpsTime2MjdTime(const GPSTIME& gpsTime)
{
	//GPS时转简化儒略日
	MJDTIME mjdTime{};
	mjdTime.day = 44244 + gpsTime.week * 7 + int(gpsTime.secOfWeek / 86400);
	mjdTime.fracDay = fmod(gpsTime.secOfWeek, 86400.0) / 86400.0;
	/*gpsTime.secOfWeek / 86400 - int(gpsTime.secOfWeek / 86400); 可用上面代码替换*/
	mjdTime.secOfDay = gpsTime.secOfWeek - int(gpsTime.secOfWeek / 86400) * 86400.0;
	return mjdTime;
}

GPSTIME CommonTime2GpsTime(const COMMONTIME& commonTime)
{
	//通用时转GPS时
	MJDTIME mjdTime = CommonTime2MjdTime(commonTime);
	GPSTIME gpsTime = MjdTime2GpsTime(mjdTime);
	return gpsTime;
}

COMMONTIME GpsTime2CommonTime(const GPSTIME& gpsTime)
{
	//GPS时转通用时
	MJDTIME mjdTime = GpsTime2MjdTime(gpsTime);
	COMMONTIME commonTime = MjdTime2CommonTime(mjdTime);
	return commonTime;
}

BDSTIME GpsTime2BdsTime(const GPSTIME& gpsTime)
{
	//GPS时 转 北斗时
	BDSTIME bdsTime{};
	bdsTime.week = gpsTime.week - 1356;
	bdsTime.secOfWeek = gpsTime.secOfWeek - 14.0;
	bdsTime.check();
	return bdsTime;
}

GPSTIME BdsTime2GpsTime(const BDSTIME& bdsTime)
{
	//北斗时 转 GPS时
	GPSTIME gpsTime{};
	gpsTime.week = bdsTime.week + 1356;
	gpsTime.secOfWeek = bdsTime.secOfWeek + 14.0;
	gpsTime.check();
	return gpsTime;
}

BDSTIME MjdTime2BdsTime(const MJDTIME& mjdTime)
{
	//简化儒略日转BDS时
	GPSTIME gpsTime = MjdTime2GpsTime(mjdTime);
	BDSTIME bdsTime = GpsTime2BdsTime(gpsTime);
	return bdsTime;
}

MJDTIME BdsTime2MjdTime(const BDSTIME& bdsTime)
{
	//BDS时转简化儒略日
	GPSTIME gpsTime = BdsTime2GpsTime(bdsTime);
	MJDTIME mjdTime = GpsTime2MjdTime(gpsTime);
	return mjdTime;
}

BDSTIME CommonTime2BdsTime(const COMMONTIME& commonTime)
{
	//通用时转BDS时
	GPSTIME gpsTime = CommonTime2GpsTime(commonTime);
	BDSTIME bdsTime = GpsTime2BdsTime(gpsTime);
	return bdsTime;
}

COMMONTIME BdsTime2CommonTime(const BDSTIME& bdsTime)
{
	//BDS时转通用时
	GPSTIME gpsTime = BdsTime2GpsTime(bdsTime);
	COMMONTIME commonTime = GpsTime2CommonTime(gpsTime);
	return commonTime;
}

void GPSTIME::check()
{
	//自检
	for (int i = 0; i < 52 && this->secOfWeek < 0; i++)
	{
		//最多循环52次，防止死循环
		this->secOfWeek += 604800.0;
		this->week--;
	}
	for(int i = 0; i < 52 && this->secOfWeek >= 604800.0; i++)
	{
		this->secOfWeek -= 604800.0;
		this->week++;
	}
}

double GPSTIME::operator-(const GPSTIME& sub) const
{
	double result{};
	result = SoWSubtraction(this->week * 604800.0 + this->secOfWeek, sub.week * 604800.0 + sub.secOfWeek);
	return result;
}

GPSTIME GPSTIME::operator-(const double sub) const
{
	GPSTIME result{};
	if (this->secOfWeek - sub < 0)
	{
		result.secOfWeek = this->secOfWeek - sub + 604800.0;
		result.week = this->week - 1;
	}
	else
	{	
		result.secOfWeek = this->secOfWeek - sub;
		result.week = this->week;
	}
	return result;
}
