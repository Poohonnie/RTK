#pragma once

#include "Constant.h"
#include <cmath>


struct XYZ
{
	double x;
	double y;
	double z;
};

struct BLH
{
	double L;//���� ��λΪ����rad
	double B;//γ�� ��λΪ����rad
	double H;//�߶� ��λΪ��
};
template<typename T1>
XYZ BLH2XYZ(const BLH&, const T1&);

template<typename T2>
BLH XYZ2BLH(const XYZ&, const T2&);

double Deg2Rad(const double deg, const double min, const double sec);


template<typename T1>
XYZ BLH2XYZ(const BLH& blh, const T1& coorSys)
{
    XYZ xyz;
    double N = coorSys.a / sqrt(1 - coorSys.eSquare * sin(blh.B) * sin(blh.B));//î��Ȧ���ʰ뾶
    //PPT 1-4 20ҳ ��ʽ
    xyz.x = (N + blh.H) * cos(blh.B) * cos(blh.L);
    xyz.y = (N + blh.H) * cos(blh.B) * sin(blh.L);
    xyz.z = (N * (1 - coorSys.eSquare) + blh.H) * sin(blh.B);

    return xyz;
}

template<typename T2>
BLH XYZ2BLH(const XYZ& xyz, const T2& coorSys)
{
    BLH blh;
    double x2 = xyz.x * xyz.x;
    double y2 = xyz.y * xyz.y;
    double z2 = xyz.z * xyz.z;//�Ѹ�ֵƽ�����������������

    double deltaZ = 0;//����ֵ
    double deltaZ1 = coorSys.eSquare * xyz.z;//deltaZ n+1
    double sinB = (xyz.z + deltaZ1) / sqrt(x2 + y2 + (xyz.z + deltaZ1) * (xyz.z + deltaZ1));
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


