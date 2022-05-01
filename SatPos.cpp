#include "RTKLib.h"
#include "CDecode.h"
#include <iostream>
#include <cmath>
#include "SatPos.h"

PARAS SatPos::CalParas(GPSTIME t/*卫星钟表面时*/, const EPHEMERIS& ephem)
{
    PARAS paras;
    //中间计算量求解
    paras.A = ephem.A;  // 轨道长半轴
    GPSTIME toe{};
    double omega{};
    if(ephem.satSys == GNSS::GPS)
    {
        toe = ephem.toeG;
        omega = wgs84.omega;
    }
    else if(ephem.satSys == GNSS::BDS)
    {
        toe = ephem.toeB;
        omega = cgcs2000.omega;
    }
    paras.n0 = sqrt(wgs84.GM / (paras.A * paras.A * paras.A));  // 平均运动角速度
    paras.tk = SoWSubtraction(t.secOfWeek, toe.secOfWeek);  // 相对于星历参考历元的时间
    
    paras.n = paras.n0 + ephem.deltaN;  // 对平均运动角速度进行改正
    paras.mk = ephem.m0 + paras.n * paras.tk;  // 平近点角
    paras.mk = paras.mk > 0 ? paras.mk : paras.mk + 2 * constant::pi;
    
    paras.ek = 0;  // 偏近点角，迭代求解
    double ek1 = paras.mk + ephem.ecc * sin(paras.mk);
    for (int i = 0; i < 20 && fabs(paras.ek - ek1) > 1e-15; ++i)
    {
        paras.ek = ek1;
        ek1 = paras.mk + ephem.ecc * sin(paras.ek);
    }
    paras.vk = atan2(sqrt(1 - ephem.ecc * ephem.ecc) * sin(paras.ek), cos(paras.ek) - ephem.ecc);  // 真近点角
    paras.phik = paras.vk + ephem.omega;  // 升交角距(未经改正)
    //计算二阶调和改正数
    paras.deltaUk = ephem.cus * sin(2 * paras.phik) + ephem.cuc * cos(2 * paras.phik);  // 计算升交角距的改正数
    paras.deltaRk = ephem.crs * sin(2 * paras.phik) + ephem.crc * cos(2 * paras.phik);  // 计算向径的改正数
    paras.deltaIk = ephem.cis * sin(2 * paras.phik) + ephem.cic * cos(2 * paras.phik);  // 计算轨道倾角改证数
    
    //计算经过改正的升交角距，向径和轨道倾角
    paras.uk = paras.phik + paras.deltaUk;  // 改正过的升交角距
    paras.rk = paras.A * (1 - ephem.ecc * cos(paras.ek)) + paras.deltaRk;  // 改正过的向径
    paras.ik = ephem.i0 + paras.deltaIk + ephem.iDot * paras.tk;  // 改正过的轨道倾角
    
    paras.omegak = ephem.omega0 + (ephem.omegaDot - omega) * paras.tk - omega * toe.secOfWeek;  // 改正后的升交点经度

    paras.xy0[0] = paras.rk * cos(paras.uk);
    paras.xy0[1] = paras.rk * sin(paras.uk);  // 卫星在轨道平面上的位置
    
    //各参变量对时间的导数
    paras.mkDot = paras.n;
    paras.ekDot = paras.mkDot / (1 - ephem.ecc * cos(paras.ek));
    paras.vkDot = sqrt(1 - ephem.ecc * ephem.ecc) * paras.ekDot / (1 - ephem.ecc * cos(paras.ek));
    paras.phikDot = paras.vkDot;
    
    paras.deltaUkDot = 2 * paras.phikDot * (ephem.cus * cos(2 * paras.phik) - ephem.cuc * sin(2 * paras.phik));
    paras.deltaRkDot = 2 * paras.phikDot * (ephem.crs * cos(2 * paras.phik) - ephem.crc * sin(2 * paras.phik));
    paras.deltaIkDot = 2 * paras.phikDot * (ephem.cis * cos(2 * paras.phik) - ephem.cic * sin(2 * paras.phik));
    
    paras.omegakDot = ephem.omegaDot - omega;
    paras.ikDot = ephem.iDot + paras.deltaIkDot;
    paras.rkDot = paras.A * ephem.ecc * paras.ekDot * sin(paras.ek) + paras.deltaRkDot;
    paras.ukDot = paras.phikDot + paras.deltaUkDot;
    
    //卫星在轨道平面内的速度
    paras.xkDot = paras.rkDot * cos(paras.uk) - paras.rk * paras.ukDot * sin(paras.uk);
    paras.ykDot = paras.rkDot * sin(paras.uk) + paras.rk * paras.ukDot * cos(paras.uk);
    return paras;
}

void SatPos::CalSat(GPSTIME t, const  EPHEMERIS& ephem)
{
    GPSTIME t0 = t;
    if(ephem.satSys == GNSS::BDS)
        //BDS使用的是BDSTIME，所以这里要转换
        t0 = GpsTime2BdsTime(t);
    
    PARAS Para = CalParas(t0, ephem);
    CalPosVel(ephem, Para);
    ClockBias(t0, Para.ek, ephem);
    ClockRate(t0, Para.ek, Para.ekDot, ephem);
}

void SatPos::CalPosVel(const EPHEMERIS& ephem, PARAS& para)
{
    /////////////////////////////////
    //		卫星位置速度计算
    /////////////////////////////////
    if(ephem.satSys == GNSS::GPS)
    {
        //GPS卫星和BDS MEO/IGSO卫星
        //卫星在地球地固坐标系中的位置计算
        satXyz.x = para.xy0[0] * cos(para.omegak) - para.xy0[1] * cos(para.ik) * sin(para.omegak);
        satXyz.y = para.xy0[0] * sin(para.omegak) + para.xy0[1] * cos(para.ik) * cos(para.omegak);
        satXyz.z = para.xy0[1] * sin(para.ik);
        //卫星在地球地固坐标系中的速度计算
        satV[0] = para.xkDot * cos(para.omegak) - satXyz.y * para.omegakDot -
                  (para.ykDot * cos(para.ik) - satXyz.z * para.ikDot) * sin(para.omegak);
        satV[1] = para.xkDot * sin(para.omegak) + satXyz.x * para.omegakDot +
                  (para.ykDot * cos(para.ik) - satXyz.z * para.ikDot) * cos(para.omegak);
        satV[2] = para.ykDot * sin(para.ik) + para.xy0[1] * para.ikDot * cos(para.ik);
    }
    else if(ephem.satSys == GNSS::BDS && !ephem.isGeo())
    {
        //BDS MEO/IGSO卫星在BDCS坐标系中的位置计算
        satXyz.x = para.xy0[0] * cos(para.omegak) - para.xy0[1] * cos(para.ik) * sin(para.omegak);
        satXyz.y = para.xy0[0] * sin(para.omegak) + para.xy0[1] * cos(para.ik) * cos(para.omegak);
        satXyz.z = para.xy0[1] * sin(para.ik);
    
        //BDS MEO/IGSO卫星在BDCS坐标系中的速度计算
        satV[0] = para.xkDot * cos(para.omegak) - para.xy0[0] * sin(para.omegak) * para.omegakDot - para.ykDot * cos(para.ik) * sin(para.omegak) + para.xy0[1] * cos(para.ik) * sin(para.omegak) * para.ikDot - para.xy0[1] * cos(para.ik) * cos(para.omegak) * para.omegakDot;
        satV[1] = para.xkDot * sin(para.omegak) + para.xy0[0] * cos(para.omegak) * para.omegakDot + para.ykDot * cos(para.ik) * cos(para.omegak) - para.xy0[1] * sin(para.ik) * sin(para.omegak) * para.ikDot - para.xy0[1] * cos(para.ik) * sin(para.omegak) * para.omegakDot;
        satV[2] = para.ykDot * sin(para.ik) + para.xy0[1] * sin(para.ik) * para.ikDot;
    }
    else if(ephem.satSys == GNSS::BDS && ephem.isGeo())
    {
        para.omegak = ephem.omega0 + ephem.omegaDot * para.tk - cgcs2000.omega * ephem.toeB.secOfWeek;
    
        //BDS GEO卫星在BDCS坐标系中的位置计算
        //GEO卫星在自定义坐标系中的坐标
        CMatrix XyzGK(3, 1);
        XyzGK.mat[0] = para.xy0[0] * cos(para.omegak) - para.xy0[1] * cos(para.ik) * sin(para.omegak);
        XyzGK.mat[1] = para.xy0[0] * sin(para.omegak) + para.xy0[1] * cos(para.ik) * cos(para.omegak);
        XyzGK.mat[2] = para.xy0[1] * sin(para.ik);
    
        CMatrix Rx(3, 3);
        double xRad = -5.0 * constant::pi / 180.0;
        Rx.mat[0] = 1; Rx.mat[1] = 0; Rx.mat[2] = 0;
        Rx.mat[3] = 0; Rx.mat[4] = cos(xRad); Rx.mat[5] = sin(xRad);
        Rx.mat[6] = 0; Rx.mat[7] = -sin(xRad); Rx.mat[8] = cos(xRad);
    
        CMatrix Rz(3, 3);
        double zRad = cgcs2000.omega * para.tk;
        Rz.mat[0] = cos(zRad); Rz.mat[1] = sin(zRad); Rz.mat[2] = 0;
        Rz.mat[3] = -sin(zRad); Rz.mat[4] = cos(zRad); Rz.mat[5] = 0;
        Rz.mat[6] = 0; Rz.mat[7] = 0; Rz.mat[8] = 1;
    
        //GEO卫星在BDCS坐标系中的坐标
        CMatrix xyzMat(3, 3);
        xyzMat = Rz * (Rx * XyzGK);
        memcpy(&this->satXyz, xyzMat.mat, sizeof(XYZ));
    
        //BDS GEO卫星在BDCS坐标系中的速度计算
        //GEO卫星在自定义坐标系中的速度
        para.omegakDot = ephem.omegaDot;
    
        CMatrix vGK(3, 1);
        vGK.mat[0] = para.xkDot * cos(para.omegak) - para.xy0[0] * sin(para.omegak) * para.omegakDot - para.ykDot * cos(para.ik) * sin(para.omegak) + para.xy0[1] * cos(para.ik) * sin(para.omegak) * para.ikDot - para.xy0[1] * cos(para.ik) * cos(para.omegak) * para.omegakDot;
        vGK.mat[1] = para.xkDot * sin(para.omegak) + para.xy0[0] * cos(para.omegak) * para.omegakDot + para.ykDot * cos(para.ik) * cos(para.omegak) - para.xy0[1] * sin(para.ik) * sin(para.omegak) * para.ikDot - para.xy0[1] * cos(para.ik) * sin(para.omegak) * para.omegakDot;
        vGK.mat[2] = para.ykDot * sin(para.ik) + para.xy0[1] * sin(para.ik) * para.ikDot;
    
        CMatrix RzDot(3, 3);
        RzDot.mat[0] = -sin(zRad); RzDot.mat[1] = cos(zRad); RzDot.mat[2] = 0;
        RzDot.mat[3] = -cos(zRad); RzDot.mat[4] = -sin(zRad); RzDot.mat[5] = 0;
        RzDot.mat[6] = 0; RzDot.mat[7] = 0; RzDot.mat[8] = 0;
        RzDot = RzDot * cgcs2000.omega;
    
        //GEO卫星在BDCS坐标系中的速度
        CMatrix vMat(3, 1);
        vMat = RzDot * (Rx * XyzGK) + Rz * (Rx * vGK);
        memcpy(&this->satV, vMat.mat, 3 * sizeof(double));
    }
}

void SatPos::ClockBias(const GPSTIME t, double ek, const EPHEMERIS &ephem)
{
    //北斗用的是BDSTIME，但是在Cal函数已经转换过了，所以这里不用再转换。
    double GM;
    if(ephem.satSys == GNSS::GPS)
        GM = wgs84.GM;
    else if(ephem.satSys == GNSS::BDS)
        GM = cgcs2000.GM;
    
    double F = -2.0 * sqrt(GM) / constant::c / constant::c;
    double delta_tr = F * ephem.ecc * sqrt(ephem.A) * sin(ek);  // 相对论效应误差改正项
    double delta_t = SoWSubtraction(t.secOfWeek, ephem.toc);
    this->clkBias = ephem.af[0] + ephem.af[1] * delta_t + ephem.af[2] * delta_t * delta_t + delta_tr /*- tgd*/;
}

void SatPos::ClockRate(const GPSTIME t, double ek, double ekDot, const EPHEMERIS &ephem)
{
    double GM;
    if(ephem.satSys == GNSS::GPS)
        GM = wgs84.GM;
    else if(ephem.satSys == GNSS::BDS)
        GM = cgcs2000.GM;
    
    double F = -2.0 * sqrt(GM) / constant::c / constant::c;  // 相对论效应误差改正
    double delta_trDot = F * ephem.ecc * sqrt(ephem.A) * cos(ek) * ekDot;
    this->clkRate = ephem.af[1] + 2 * ephem.af[2] * SoWSubtraction(t.secOfWeek, ephem.toc) + delta_trDot;
}

bool SatPos::Overdue(const GPSTIME t, const  EPHEMERIS& ephem)
{
    if(ephem.satSys == GNSS::GPS)
    {
        if (fabs(t - ephem.toeG) < 7500)
            //星历未过期
            return false;
    }
    else if(ephem.satSys == GNSS::BDS)
    {
        BDSTIME t0 = GpsTime2BdsTime(t);
        if (fabs(t0 - ephem.toeB) < 3900)
            //星历未过期
            return false;
    }
    return true;
}

void SatPos::CalSatEl(const XYZ& rcvrXyz, CoorSys& coor)
{
    //卫星高度角计算
    BLH rcvrBlh = XYZ2BLH(rcvrXyz, coor);  // 接收机的大地坐标
    CMatrix trans(3, 3);  // 转换矩阵
    CMatrix line(3, 1);  // 接收机与卫星的连线
    CMatrix satxyz(3, 1);  // 卫星在站心坐标系中的坐标
    
    trans.mat[0] = -sin(rcvrBlh.B) * cos(rcvrBlh.L);
    trans.mat[1] = -sin(rcvrBlh.B) * sin(rcvrBlh.L);
    trans.mat[2] = cos(rcvrBlh.B);
    trans.mat[3] = -sin(rcvrBlh.L);
    trans.mat[4] = cos(rcvrBlh.L);
    trans.mat[5] = 0;
    trans.mat[6] = cos(rcvrBlh.B) * cos(rcvrBlh.L);
    trans.mat[7] = cos(rcvrBlh.B) * sin(rcvrBlh.L);
    trans.mat[8] = sin(rcvrBlh.B);
    trans.check();  // B=0 L=0 H=0的情况
    
    line.mat[0] = this->satXyz.x - rcvrXyz.x;
    line.mat[1] = this->satXyz.y - rcvrXyz.y;
    line.mat[2] = this->satXyz.z - rcvrXyz.z;
    
    satxyz = trans * line;  // 卫星在站心坐标系中的坐标
    
    this->eleAngle = atan(satxyz.mat[2] / sqrt(satxyz.mat[0] * satxyz.mat[0] + satxyz.mat[1] * satxyz.mat[1]));  // 卫星高度角
}

void SatPos::Hopefield(const XYZ& rcvrXyz, CoorSys& coor)
{
    BLH rcvrBlh = XYZ2BLH(rcvrXyz, coor);
    double H = rcvrBlh.H;  // 接收机大地高，方便后面书写
    if (H > 1e+4)
    {
        this->tropDelay = 10;
        return ;  // 测站高度不在对流层范围
    }
    double H0 = 0.0;  // m 海平面
    double T0 = 20.0 + 273.16;  // K 温度
    double p0 = 1013.25;  // mbar 气压
    double RH0 = 0.5;  // 相对湿度
    
    double RH = RH0 * exp(-0.0006369 * (H - H0));
    double p = p0 * pow(1 - 2.26e-5 * (H - H0), 5.225);
    double T = T0 - 0.0065 * (H - H0);
    double e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
    double hw = 11000.0;
    double hd = 40136.0 + 148.72 * (T0 - 273.16);
    double Kw = 155.2e-7 * 4810.0 / (T * T) * e * (hw - H);
    double Kd = 155.2e-7 * p / T * (hd - H);
    
    double E = this->eleAngle * 180.0 / constant::pi;
    double deltaD = Kd / sin(sqrt(E*E + 6.25) * constant::D2R);
    double deltaW = Kw / sin(sqrt(E*E + 2.25) * constant::D2R);
    this->tropDelay = deltaD + deltaW;
}


int EpkPos::FindSatPosIndex(int prn, GNSS sys)
{
    for (int i = 0; i < MAXCHANNELNUM; ++i)
    {
        if (satPos[i].prn == prn && satPos[i].sys == sys)
            return i;  // 返回所找到的satObs数组下标
    }
    return 114514;  // 没找到就返回114514;
}
