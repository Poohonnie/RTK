#include "CoordinateConversion.hpp"

double Deg2Rad(const double deg, const double min, const double sec)
{

    double rad = 0.0;
    if (!(deg * min * sec))
    {
        rad = (deg + min / 60.0 + sec / 3600.0) * constant::pi / 180.0;
        return rad;
    }
    else
        return -114.514;
}
