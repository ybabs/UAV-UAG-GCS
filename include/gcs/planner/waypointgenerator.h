#ifndef WAYPOINTGENERATOR_H
#define WAYPOINTGENERATOR_H

#include <QGeoCoordinate>
#include <vector>
#include <cmath>
#include <iostream>
#include "gcs/utils/utils.h"
#include <sensor_msgs/NavSatFix.h>


class GpsUtils
{
public:
    GpsUtils();
    double GetPathLength(sensor_msgs::NavSatFix start_coord, sensor_msgs::NavSatFix end_coord);
    sensor_msgs::NavSatFix GetDestinationCoordinate(sensor_msgs::NavSatFix start_coord, double azimuth, double distance);
    double ComputeBearing(sensor_msgs::NavSatFix start_point, sensor_msgs::NavSatFix end_point);
    std::vector<sensor_msgs::NavSatFix> returnInterpolatedPoints(int  distance, double bearing, sensor_msgs::NavSatFix start, sensor_msgs::NavSatFix end);
    std::vector<sensor_msgs::NavSatFix> returnPositionsBasedOnLocations(int locations, double bearing, sensor_msgs::NavSatFix start, sensor_msgs::NavSatFix end);

private:
    std::vector<sensor_msgs::NavSatFix> route;
};
#endif //WAYPOINTGENERATOR_H
