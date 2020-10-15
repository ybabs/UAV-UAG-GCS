#ifndef UTILS_H
#define UTILS_H

#include <cmath>

#define PI (double) 3.141592653589793
#define C_EARTH (double)6378137.0


static double DegToRad(double degree)
{
    return degree * (PI/180.0);
}

static double RadToDeg(double rad)
{
    return rad *  (180.0/PI);
}


#endif //UTILS_H
