#ifndef WAYPOINTGENERATOR_H
#define WAYPOINTGENERATOR_H

#include <vector>
#include <cmath>
#include <iostream>
#include "gcs/utils/utils.h"
#include <ros/ros.h>
#include "gcs/Waypoint.h"
#include <sensor_msgs/NavSatFix.h>
#include <random>

/// Class to handle Waypoint Optimisation and Route generation for GPS Waypoints.
///
/// Class contains helper functions for GPS coordinate manipulation and code required to
/// implement a genetic algorithm to solve the Travelling Salesman Problem. This is based on
/// the solution presented by Daniel Shiffman in the Nature of Code Series on TSP as seen on
/// https://thecodingtrain.com/CodingChallenges/035.1-tsp.html
class GpsUtils
{
public:
    GpsUtils();
    double GetPathLength(gcs::Waypoint start_coord, gcs::Waypoint end_coord);
    gcs::Waypoint GetDestinationCoordinate(gcs::Waypoint start_coord, double azimuth, double distance);
    double ComputeBearing(gcs::Waypoint start_point, gcs::Waypoint end_point);
    std::vector<gcs::Waypoint> returnInterpolatedPoints(int  distance, double bearing, gcs::Waypoint start, gcs::Waypoint end);
    std::vector<gcs::Waypoint> returnPositionsBasedOnLocations(int locations, double bearing, gcs::Waypoint start, gcs::Waypoint end);
    double GetRouteDistance(std::vector<gcs::Waypoint>&gps_list);
    double GetRouteDistance(std::vector<gcs::Waypoint>&gps_list, std::vector<int>&order);
    

private:

    
};
#endif //WAYPOINTGENERATOR_H
