#ifndef WAYPOINTGENERATOR_H
#define WAYPOINTGENERATOR_H

#include <vector>
#include <cmath>
#include <iostream>
#include "gcs/utils/utils.h"
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
    double GetPathLength(sensor_msgs::NavSatFix start_coord, sensor_msgs::NavSatFix end_coord);
    sensor_msgs::NavSatFix GetDestinationCoordinate(sensor_msgs::NavSatFix start_coord, double azimuth, double distance);
    double ComputeBearing(sensor_msgs::NavSatFix start_point, sensor_msgs::NavSatFix end_point);
    std::vector<sensor_msgs::NavSatFix> returnInterpolatedPoints(int  distance, double bearing, sensor_msgs::NavSatFix start, sensor_msgs::NavSatFix end);
    std::vector<sensor_msgs::NavSatFix> returnPositionsBasedOnLocations(int locations, double bearing, sensor_msgs::NavSatFix start, sensor_msgs::NavSatFix end);
    void swap(std::vector<sensor_msgs::NavSatFix>&gps_list, int i, int j);
    void swap(std::vector<int>&route, int i, int j);
    void shuffle(std::vector<int>&route);
    double GetRouteDistance(std::vector<sensor_msgs::NavSatFix>&gps_list);
    double GetRouteDistance(std::vector<sensor_msgs::NavSatFix>&gps_list, std::vector<int>&order);
    void normalizeFitness();
    void computeFitness();
    void nextGeneration();
    void initialiseGA(std::vector<sensor_msgs::NavSatFix>& waypoints, int MAX_POP);
    std::vector<int> pickRandomParent(std::vector<std::vector<int> >& population, std::vector<float>& fitness);
    void mutate(std::vector<int> &order, float mutationRate);
    std::vector<int> crossover(std::vector<int> &parentA, std::vector<int> &parentB);

private:
    std::vector<sensor_msgs::NavSatFix> route;
    std::vector<sensor_msgs::NavSatFix> waypoints;
    std::vector<float> fitness;
    double recordDistance;
    std::vector<int>bestOrder;
    std::vector<int>currentBestOrder;
    std::vector<int> order;
    std::vector<std::vector<int> > population;
};
#endif //WAYPOINTGENERATOR_H
