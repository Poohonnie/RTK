#pragma once
#include "Constant.h"
#include "CDecode.h"
#include <cmath>

class SatPositioning
{
protected:
	XYZ xyzk;//�����ڵ��ĵع�����ϵ�µ�����

public:
	void GpsPositioning(const GPSEPHEM& gpsEphem);//GPS����λ�ü���
};

