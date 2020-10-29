#ifndef TSP_H
#define TSP_H

#include <QObject>
#include <vector>
#include <cmath>
#include <iostream>
#include "gcs/utils/utils.h"
#include <ros/ros.h>
#include "gcs/Waypoint.h"
#include "gcs/planner/waypointgenerator.h"
#include <sensor_msgs/NavSatFix.h>
#include <random>

class TSP : public QObject
{
    Q_OBJECT

public:
    TSP();
    void initialiseGA(std::vector<gcs::Waypoint>& waypoints, int MAX_POP);
    std::vector<int> pickRandomParent(std::vector<std::vector<int> >& population, std::vector<float>& fitness);
    void mutate(std::vector<int> &order, float mutationRate);
    std::vector<int> crossover(std::vector<int> &parentA, std::vector<int> &parentB);
    void resetVariables();
    double getBestDistance();
    std::vector<int> getBestOrder();
    void swap(std::vector<gcs::Waypoint>&gps_list, int i, int j);
    void swap(std::vector<int>&route, int i, int j);
    void shuffle(std::vector<int>&route);
    void normalizeFitness();
    void computeFitness();
    void nextGeneration();
   

private:
    std::vector<gcs::Waypoint> route;
    std::vector<gcs::Waypoint> waypoints;
    std::vector<float> fitness;
    double recordDistance;
    std::vector<int>bestOrder;
    std::vector<int>currentBestOrder;
    std::vector<int> order;
    std::vector<std::vector<int> > population;
    GpsUtils routeModel;



};

#endif