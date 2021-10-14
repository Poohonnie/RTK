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
	}//Ĭ�Ϲ��캯��
};

struct MJDTIME
{
	int day;
	double fracDay;//�������յ�С������(��)
	double secOfDay;//һ���ڵľ�������
	MJDTIME()
	{
		day = 0;
		secOfDay = 0.0;
		fracDay = 0.0;
	}//Ĭ�Ϲ��캯��
};

struct GPSTIME
{
	unsigned short week;//GPS��
	double secOfWeek;//GPS����

	GPSTIME()
	{
		week = 0;
		secOfWeek = 0.0;
	}//Ĭ�Ϲ��캯��

	void check();//�Լ�
	double operator-(const GPSTIME& sub) const;//GPSʱ��������֧�ִ��С
};

struct BDSTIME : public GPSTIME
{

};

double CommonTime2UT(const COMMONTIME&);//ͨ��ʱת����ʱ

MJDTIME CommonTime2MjdTime(const COMMONTIME&);//ͨ��ʱת��������
COMMONTIME MjdTime2CommonTime(const MJDTIME&);//��������תͨ��ʱ

GPSTIME MjdTime2GpsTime(const MJDTIME&);//��������תGPSʱ
MJDTIME GpsTime2MjdTime(const GPSTIME&);//GPSʱת��������

GPSTIME CommonTime2GpsTime(const COMMONTIME&);//ͨ��ʱתGPSʱ
COMMONTIME GpsTime2CommonTime(const GPSTIME&);//GPSʱתͨ��ʱ


BDSTIME GpsTime2BdsTime(const GPSTIME&);//GPSʱ ת ����ʱ
GPSTIME BdsTime2GpsTime(const BDSTIME&);//����ʱ ת GPSʱ


BDSTIME MjdTime2BdsTime(const MJDTIME&);//��������תBDSʱ
MJDTIME BdsTime2MjdTime(const BDSTIME&);//BDSʱת��������

BDSTIME CommonTime2BdsTime(const COMMONTIME&);//ͨ��ʱתBDSʱ
COMMONTIME BdsTime2CommonTime(const BDSTIME&);//BDSʱתͨ��ʱ