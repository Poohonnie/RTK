#include "RTKlib.h"
#include <cmath>

//两个大地坐标系统参数
CoorSys wgs84
{
    6378137.0,
    6356752.3142,
    0.00669437999013,
    0.006739496742227,
    1 / 298.257223563,
    3.986004418e+14,
    7.292115e-5
};

CoorSys cgcs2000
{
    6378137.0,
    6356752.3142,
    0.00669437999013,
    0.006739496742227,
    1 / 298.257223563,
    3.986004418e+14,
    7.292115e-5
};

double Deg2Rad(const double deg, const double min, const double sec)
{
    double rad{};
    if (deg > 0 || min > 0 || sec > 0)
    {
        //角度大于0
        rad = (deg + min / 60.0 + sec / 3600.0) * constant::pi / 180.0;
        return rad;
    }
    else
        return -114514.0;
}

XYZ XYZ::operator+(const XYZ &add) const
{
    XYZ result{};
    result.x = this->x + add.x;
    result.y = this->y + add.y;
    result.z = this->z + add.z;
    return result;
}

XYZ XYZ::operator-(const XYZ& sub) const
{
    XYZ result{};
    result.x = this->x - sub.x;
    result.y = this->y - sub.y;
    result.z = this->z - sub.z;
    return result;
}

XYZ BLH2XYZ(const BLH& blh, const CoorSys& coorSys)
{
    XYZ xyz{};
    double N = coorSys.a / sqrt(1 - coorSys.eSquare * sin(blh.B) * sin(blh.B));  // 卯酉圈曲率半径
    //PPT 1-4 20页 公式
    xyz.x = (N + blh.H) * cos(blh.B) * cos(blh.L);
    xyz.y = (N + blh.H) * cos(blh.B) * sin(blh.L);
    xyz.z = (N * (1 - coorSys.eSquare) + blh.H) * sin(blh.B);
    
    return xyz;
}

BLH XYZ2BLH(const XYZ& xyz, const CoorSys& coorSys)
{
    BLH blh{};
    if (sqrt(xyz.x * xyz.x + xyz.y * xyz.y + xyz.z * xyz.z) < 1e+6)
    {
        //不在地球表面，认定为异常数值
        blh.B = 0;
        blh.L = 0;
        blh.H = 0;
        return blh;
    }
    double x2 = xyz.x * xyz.x;
    double y2 = xyz.y * xyz.y;
    double z2 = xyz.z * xyz.z;  // 把各值平方求出来方便后面计算
    
    double deltaZ = 0;  // 赋初值
    double deltaZ1 = coorSys.eSquare * xyz.z;  // deltaZ n+1
    double sinB = ((xyz.z + deltaZ1) + 1e-6) / (1e-6 + sqrt(x2 + y2 + (xyz.z + deltaZ1) * (xyz.z + deltaZ1)));
    double N = coorSys.a / sqrt(1 - coorSys.eSquare * sinB * sinB);
    
    blh.L = atan2(xyz.y, xyz.x);  // 注意L取值范围，这里要用atan2
    
    for (int i = 0; i < 12 && fabs(deltaZ - deltaZ1) > 1e-10; ++i)
    {
        deltaZ = deltaZ1;
        sinB = (xyz.z + deltaZ) / sqrt(x2 + y2 + (xyz.z + deltaZ) * (xyz.z + deltaZ));
        N = coorSys.a / sqrt(1 - coorSys.eSquare * sinB * sinB);
        deltaZ1 = N * coorSys.eSquare * sinB;
        
    }  // 迭代求ΔZ
    blh.B = atan2(xyz.z + deltaZ, sqrt(x2 + y2));
    blh.H = sqrt(x2 + y2 + (xyz.z + deltaZ) * (xyz.z + deltaZ)) - N;
    
    return blh;
}

void CalDNEU(const XYZ &refXyz, const XYZ &sttnXyz, double *dNEU)
{
    BLH refBlh = XYZ2BLH(refXyz, wgs84);
    double dXyz[3]{};
    dXyz[0] = sttnXyz.x - refXyz.x;
    dXyz[1] = sttnXyz.y - refXyz.y;
    dXyz[2] = sttnXyz.z - refXyz.z;
    
    dNEU[0] = -sin(refBlh.B) * cos(refBlh.L) * dXyz[0] - sin(refBlh.B) * sin(refBlh.L) * dXyz[1]
              + cos(refBlh.B) * dXyz[2];
    dNEU[1] = -sin(refBlh.L) * dXyz[0] + cos(refBlh.L) * dXyz[1];
    dNEU[2] = cos(refBlh.B) * cos(refBlh.L) * dXyz[0] + cos(refBlh.B) * sin(refBlh.L) * dXyz[1]
              + sin(refBlh.B) * dXyz[2];
}
