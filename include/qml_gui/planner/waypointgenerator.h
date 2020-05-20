#ifndef WAYPOINTGENERATOR_H
#define WAYPOINTGENERATOR_H

#include <QGeoCoordinate>
#include <vector>
#include <cmath>
#include <iostream>
#include "qml_gui/utils/utils.h"


class GpsUtils
{
public:
    GpsUtils();
    double GetPathLength(QGeoCoordinate start_coord, QGeoCoordinate end_coord);
    QGeoCoordinate GetDestinationCoordinate(QGeoCoordinate start_coord, double azimuth, double distance);
    double ComputeBearing(QGeoCoordinate start_point, QGeoCoordinate end_point);
    std::vector<QGeoCoordinate> returnInterpolatedPoints(int  distance, double bearing, QGeoCoordinate start, QGeoCoordinate end);
    std::vector<QGeoCoordinate> returnPositionsBasedOnLocations(int locations, double bearing, QGeoCoordinate start, QGeoCoordinate end);

private:
    std::vector<QGeoCoordinate> route;
};
#endif //WAYPOINTGENERATOR_H
