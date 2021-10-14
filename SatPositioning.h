#pragma once
#include "Constant.h"
#include "CDecode.h"
#include <cmath>

class SatPositioning
{
protected:
	XYZ xyzk;//卫星在地心地固坐标系下的坐标

public:
	void GpsPositioning(const GPSEPHEM& gpsEphem);//GPS卫星位置计算
};

