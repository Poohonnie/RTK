#pragma once

namespace constant
{
    const double pi = 3.14159265358979323846;  // 圆周率，小数点后20位
    const int c = 299792458;  // 光速c, 单位 m/s
    const double D2R = constant::pi / 180.0;
    const double R2D = 180.0 / constant::pi;
    const double fq_l1 = 1575.42e+6;  // GPS L1
    const double wl_l1 = c / fq_l1;
    const double fq_l2 = 1227.60e+6;  // GPS L2
    const double wl_l2 = c / fq_l2;
    const double fq_b1 = 1561.098e+6;  // BDS B1
    const double wl_b1 = c / fq_b1;
    const double fq_b3 = 1268.52e+6;  // BDS B3
    const double wl_b3 = c / fq_b3;
}

enum class GNSS { GPS, BDS, GLONASS, Galileo };

/***********************************************************************
 *                              配置表
 ***********************************************************************/

struct CONFIG
{
    unsigned short posMode{};  // 0 single; 1 kinematic;
    unsigned short iptStream{};  // 0 file; 1 server;
    double elmin{};  // Elvertion mask.  rad
    double ratioThres{};  // ratio值阈值
    char iptFileName[2][200]{};  // 输入文件名
    char optFileName[200]{};  // 输出文件名
    
    char iptIP[2][20]{};  // 输入IP
    
    unsigned short port[2]{};
};

/***********************************************************************
 *                              时间系统
 ***********************************************************************/

struct COMMONTIME
{
    short year;
    unsigned short month;
    unsigned short day;
    unsigned short hour;
    unsigned short minute;
    double second;
};

struct MJDTIME
{
    int day;
    double fracDay;  // 简化儒略日的小数部分(天)
    double secOfDay;  // 一天内的具体秒数
};

struct GPSTIME
{
    unsigned short week;  // GPS周
    double secOfWeek;  // GPS周秒
    
    void check();  // 自检
    double operator-(const GPSTIME& sub) const;
    GPSTIME operator-(double sub) const;
};

struct BDSTIME : public GPSTIME
{
};

double SoWSubtraction(double t, double toc);  // 两个GPS周内秒的减法

double CommonTime2UT(const COMMONTIME&);  // 通用时转世界时

MJDTIME CommonTime2MjdTime(const COMMONTIME&);  // 通用时转简化儒略日
COMMONTIME MjdTime2CommonTime(const MJDTIME&);  // 简化儒略日转通用时

GPSTIME MjdTime2GpsTime(const MJDTIME&);  // 简化儒略日转GPS时
MJDTIME GpsTime2MjdTime(const GPSTIME&);  // GPS时转简化儒略日

GPSTIME CommonTime2GpsTime(const COMMONTIME&);  // 通用时转GPS时
COMMONTIME GpsTime2CommonTime(const GPSTIME&);  // GPS时转通用时


BDSTIME GpsTime2BdsTime(const GPSTIME&);  // GPS时 转 北斗时
GPSTIME BdsTime2GpsTime(const BDSTIME&);  // 北斗时 转 GPS时


BDSTIME MjdTime2BdsTime(const MJDTIME&);  // 简化儒略日转BDS时
MJDTIME BdsTime2MjdTime(const BDSTIME&);  // BDS时转简化儒略日

BDSTIME CommonTime2BdsTime(const COMMONTIME&);  // 通用时转BDS时
COMMONTIME BdsTime2CommonTime(const BDSTIME&);  // BDS时转通用时

/***********************************************************************
 *                              坐标系统
 ***********************************************************************/

struct CoorSys
{
    double a;  // 长半轴 单位m
    double b;  // 短半轴 单位m
    double eSquare;  // 第一偏心率的平方
    double e2Square;  // 第二偏心率的平方
    double f;  // 扁率
    double GM;  // 地心引力常数 单位m³s^-2
    double omega;  // 自转角速度 单位rad/s
};

extern CoorSys wgs84;
extern CoorSys cgcs2000;

struct XYZ
{
    double x;
    double y;
    double z;
    
    XYZ operator+(const XYZ &add) const;  // 坐标加法
    XYZ operator-(const XYZ &sub) const;  // 坐标减法
};
struct BLH
{
    double L;  // 经度 单位为弧度rad
    double B;  // 纬度 单位为弧度rad
    double H;  // 高度 单位为米
};

XYZ BLH2XYZ(const BLH&, const CoorSys&);

BLH XYZ2BLH(const XYZ&, const CoorSys&);

double Deg2Rad(double deg, double min, double sec);

void CalDNEU(const XYZ &refXyz, const XYZ &sttnXyz, double *dNEU);  // 计算测站在NEU系下的定位误差

/*********************************************************************
 *                              矩阵
 *********************************************************************/
 
class CMatrix
{
public:
    int rows, cols;
    double* mat;
    
    CMatrix();  // 默认构造函数
    CMatrix(const double* Mat, int Rows, int Cols);  // 构造函数
    CMatrix(int Rows, int Cols);  // 全0矩阵构造函数
    CMatrix(const CMatrix&);  // 拷贝构造函数
    virtual ~CMatrix();  // 析构函数
    static CMatrix Eye(int n);  // 单位阵
    static CMatrix Zeros(int m, int n);  // 全零阵
    
    void Show() const;  // 最原始的矩阵的显示函数
    void Show(int m, int n) const;  // 按照位宽和精度显示矩阵
    void Write(double val, int m, int n) const;  // 写入 m 行 n 列的值
    double Read(int m, int n) const;  // 读取 m 行 n 列的值
    // 判断是否为方阵
    void check() const;  // 关于+0 -0的判断
    
    CMatrix operator+(const CMatrix& addMat) const;  // 矩阵加法
    CMatrix operator-(const CMatrix& subMat) const;  // 矩阵减法
    CMatrix& operator=(const CMatrix& origin);  // 矩阵赋值
    CMatrix& operator+=(const CMatrix& addMat);  // +=
    CMatrix& operator-=(const CMatrix& subMat);  // -=
    CMatrix operator*(const CMatrix& multiplierMat) const;  // 矩阵乘法
    CMatrix operator*(double num) const;  // 矩阵数乘
    
    CMatrix Inv() const;  // 矩阵求逆，返回值为矩阵的逆
    CMatrix Trans() const;  // 矩阵转置，返回值为转置矩阵
    void SetZero() const;  // 将矩阵置零
    void AddRow(double *vec/*需要添加的行数组*/, int aimRow/*添加到第几行*/);  // 矩阵扩展，加一行
    void AddCol(double* vec/*需要添加的列数组*/, int aimCol/*添加到第几列*/);  // //矩阵扩展，加一列
    void SubRow(int aimRow/*去掉第几行*/);  // 去掉一行
    void SubCol(int aimCol/*去掉第几列*/);  // 去掉一列
};

int lambda(int n, int m, const double *a, const double *Q, double *F, double *s);