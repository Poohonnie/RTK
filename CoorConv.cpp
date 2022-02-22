#include "lib.h"

double Deg2Rad(const double deg, const double min, const double sec)
{
    double rad{};
    if (deg > 0 || min > 0 || sec > 0)
    {
        //�Ƕȴ���0
        rad = (deg + min / 60.0 + sec / 3600.0) * constant::pi / 180.0;
        return rad;
    }
    else
        return -114514.0;
}

XYZ BLH2XYZ(const BLH& blh, const CoorSys& coorSys)
{
    XYZ xyz{};
    double N = coorSys.a / sqrt(1 - coorSys.eSquare * sin(blh.B) * sin(blh.B));//î��Ȧ���ʰ뾶
    //PPT 1-4 20ҳ ��ʽ
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
        //���ڵ�����棬�϶�Ϊ�쳣��ֵ
        blh.B = 0;
        blh.L = 0;
        blh.H = 0;
        return blh;
    }
    double x2 = xyz.x * xyz.x;
    double y2 = xyz.y * xyz.y;
    double z2 = xyz.z * xyz.z;//�Ѹ�ֵƽ�����������������
    
    double deltaZ = 0;//����ֵ
    double deltaZ1 = coorSys.eSquare * xyz.z;//deltaZ n+1
    double sinB = ((xyz.z + deltaZ1) + 1e-6) / (1e-6 + sqrt(x2 + y2 + (xyz.z + deltaZ1) * (xyz.z + deltaZ1)));
    double N = coorSys.a / sqrt(1 - coorSys.eSquare * sinB * sinB);
    
    blh.L = atan2(xyz.y, xyz.x);//ע��Lȡֵ��Χ������Ҫ��atan2
    
    for (int i = 0; i < 12 && fabs(deltaZ - deltaZ1) > 1e-10; i++)
    {
        deltaZ = deltaZ1;
        sinB = (xyz.z + deltaZ) / sqrt(x2 + y2 + (xyz.z + deltaZ) * (xyz.z + deltaZ));
        N = coorSys.a / sqrt(1 - coorSys.eSquare * sinB * sinB);
        deltaZ1 = N * coorSys.eSquare * sinB;
        
    }//������Z
    blh.B = atan2(xyz.z + deltaZ, sqrt(x2 + y2));
    blh.H = sqrt(x2 + y2 + (xyz.z + deltaZ) * (xyz.z + deltaZ)) - N;
    
    return blh;
}
