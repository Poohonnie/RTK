#pragma once
#include "CDecode.h"

struct GFMW
{
	GNSS sys;
	unsigned short prn;
	double LMW;
	double LGF;
	double LIF;
	double PIF;
	int n;//��Ԫ����1��ʾ��һ����Ԫ

	bool valid;

};

struct EPKGFMW
{
	GFMW gfmw[MAXCHANNELNUM];//������Ԫ����GFMW��Ϲ۲�ֵ
	int FindSatObsIndex(const int prn, const GNSS sys);//����ĳ��prn�ŵ�������epkGfmw�е��±�
};

class CDetectOutlier
{
private:
	EPKGFMW lastEpk;//��һ��ԪGF��MW���
	EPKGFMW curEpk;//��ǰ��ԪGF��MW���

public:
	friend class Client;
	friend class SPP;
	void DetectOutlier(RAWDATA raw);//�ֲ�̽������

	CDetectOutlier()
	{
		memset(this, 0, sizeof(CDetectOutlier));
	};
};

