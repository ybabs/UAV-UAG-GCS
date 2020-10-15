#include "gcs/planner/waypointgenerator.h"

GpsUtils::GpsUtils()
{

}

/// Calculates the distance between two latlong coordinate pairs.
///
/// @param start_coord Start GPS Position
/// @param end_coord  End GPS Position
/// @returns distance between both coordinates
double GpsUtils::GetPathLength(gcs::Waypoint start_coord, gcs::Waypoint end_coord)
{
    
    double lat1_rad = DegToRad(start_coord.latitude);
    double lat2_rad = DegToRad(end_coord.latitude);
    double delta_lat_deg = end_coord.latitude - start_coord.latitude;
    double delta_lat = DegToRad(delta_lat_deg);
    double delta_lon_deg = end_coord.longitude - start_coord.longitude;
    double delta_lon = DegToRad(delta_lon_deg);
    double a = sin(delta_lat/2) * sin(delta_lat/2) + cos(lat1_rad) * cos(lat2_rad) * sin(delta_lon/2) * sin(delta_lon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = C_EARTH * c;

    return d;

}

/// Calculates latitude and longitude of a destination point given, start coordinate, distance and bearing.
/// @param start_coord Starting GPS location
/// @param azimuth bearing angle in degrees
/// @param distance from starting position
/// @returns destination GPS coordinate
gcs::Waypoint GpsUtils::GetDestinationCoordinate(gcs::Waypoint start_coord, double azimuth, double distance)
{
    double radius_km = C_EARTH/1000; // Earth's Radius in Km
    double bearing = DegToRad(azimuth); // degrees converted to Rad
    double d = distance / 1000; // convert distance in metres to km
    double lat1 = DegToRad(start_coord.latitude);
    double lon1 = DegToRad(start_coord.longitude);
    double lat2 = asin(sin(lat1) * cos(d/radius_km) + cos(lat1) * sin(d/radius_km) * cos(bearing));
    double lon2 = lon1 + atan2(sin(bearing) * sin(d/radius_km) * cos(lat1), cos(d/radius_km) -  sin(lat1) * sin(lat2));

    lat2 = RadToDeg(lat2);
    lon2 = RadToDeg(lon2);

    gcs::Waypoint destination;

    destination.latitude = lat2;
    destination.longitude  = lon2;

    return destination;

}

/// Calculates bearing in degrees from start point to endpoint.
/// @param start_point starting GPS location
/// @param end_point destination GPS location
/// @returns Bearing between start point and end point
double GpsUtils::ComputeBearing(gcs::Waypoint start_point, gcs::Waypoint end_point)
{
    double start_lat = DegToRad(start_point.latitude);
    double start_lon = DegToRad(start_point.longitude);
    double end_lat = DegToRad(end_point.latitude);
    double end_lon = DegToRad(end_point.longitude);
    double dLon = end_lon - start_lon;
    double dPhi = log(tan((end_lat / 2.0) +( PI/4)) / tan((start_lat/2.0) + (PI / 4)));

    if(std::abs(dLon) > PI)
    {
        if(dLon > 0)
        {
            dLon = -(2 * PI - dLon);
        }
        else
        {
            dLon = 2 * PI * dLon;
        }
    }

    double bearing = fmod(RadToDeg(atan2(dLon, dPhi)) + 360.0, 360.0);

    return bearing;

}

/// Computes a series of GPS waypoints based on a starting position, end point, distance between points and initial bearing.
/// @param interval distance between interpolated waypoints in meters
/// @param bearing  bearing angle between start and end points
/// @param start Start GPS position
/// @param end Final GPS position
/// @returns vector containing GPS waypoints between start and end GPS coordinates
std::vector<gcs::Waypoint>GpsUtils::returnInterpolatedPoints(int interval, double bearing, gcs::Waypoint start, gcs::Waypoint end)
{
     std::vector<gcs::Waypoint> route;
    double d = GetPathLength(start, end);
    std::cout<< "Distance: " << d << std::endl;
    int distance;
    if(interval != 0)
    {
      int distance = (int) d / interval;
    }
    int distance_covered = interval;
    // Add first location.
    route.push_back(start);
    for (int i = 0; i < distance; i++)
    {
        gcs::Waypoint new_pos = GetDestinationCoordinate(start, bearing, distance_covered);
        distance_covered+=interval;
        route.push_back(new_pos);
    }
    // add final location
    route.push_back(end);

    return route;
}

/// Returns a number of GPS positions based on an initial bearing, number of points and start and end GPS Coordinates.
/// @param loc_count number of intermediate GPS positions
/// @param bearing angle between coordinates
/// @param start starting GPS coordinates
/// @param end  Final GPS coordinates
/// @returns a vector containing intermediate locations.
std::vector<gcs::Waypoint>GpsUtils::returnPositionsBasedOnLocations(int loc_count, double bearing, gcs::Waypoint start, gcs::Waypoint end)
{
    std::vector<gcs::Waypoint> route;
    double total_distance = GetPathLength(start, end);
    double dist = total_distance / loc_count; // return number of points.
    int interval = (int) dist;
    int counter = interval;

    for(int i = 0; i < loc_count; i++)
    {
        gcs::Waypoint new_pos = GetDestinationCoordinate(start, bearing, counter);
        counter+=interval;
        route.push_back(new_pos);
    }
    return route;
}

/// swap positions of coordinates in an array based on the element position
/// @param gps_list reference to GPS coordinate vector containing elements
/// @param i First position to be swapped
/// @param j second position to be swapped
/// @returns nothing
void GpsUtils::swap(std::vector<gcs::Waypoint> &gps_list, int i, int j)
{
    gcs::Waypoint temp = gps_list[i];
    gps_list[i] = gps_list[j];
    gps_list[j] = temp;
}

/// returns the distance between GPS coordinates in a vector
/// @param gps_list vector containing GPS positions
/// @returns distance between GPS positions in the vector
double GpsUtils::GetRouteDistance(std::vector<gcs::Waypoint> &gps_list)
{
   double sum = 0;
   for(std::size_t i = 0; i < gps_list.size()-1; i++)
   {
       double d = GetPathLength(gps_list[i], gps_list[i+1]);
       sum+=d;
   }
   return sum;
}

/// swap positions of route orders in an array based on the element position
/// @param route reference to order of positions containing elements
/// @param i First position to be swapped
/// @param j second position to be swapped
/// @returns nothing
 void GpsUtils::swap(std::vector<int>&route, int i, int j)
 {
     int temp = route[i];
     route[i] = route[j];
     route[j] = temp;
 }

 /// shuffles the position of elements within the vector
/// @param route vector to be shuffled
/// @returns nothing
 void GpsUtils::shuffle(std::vector<int> &route)
 {
     // shuffle only the elements that contain the route and
     // skip the starting points
     for (std::size_t i = route.size()-2; i >1; --i)
     {
         int indexA = rand() % (route.size()-2) + 1;
         int indexB = rand() % (route.size()-2) + 1;
         swap(route, indexA, indexB);
     }
 }

 /// returns the distance of a set of GPS points based on the order list
 /// @param gps_list List of GPS positions to be shuffled
 /// @param order list of order of gps routes to be shuffled
 /// @returns distance of each list of GPS positions based on order
 double GpsUtils::GetRouteDistance(std::vector<gcs::Waypoint>&gps_list, std::vector<int>&order)
 {
     double sum = 0;
     for(std::size_t i = 0; i < order.size()-1; i++)
     {
         int waypointAIndex = order[i];
         gcs::Waypoint waypointA = gps_list[waypointAIndex];
         int waypointBIndex = order[i+1];
         gcs::Waypoint waypointB = gps_list[waypointBIndex];

         double distance = GetPathLength(waypointA, waypointB);
         sum+=distance;
     }
     return sum;
 }

  /// normalises the fitness value
 /// @returns nothing
 void GpsUtils::normalizeFitness()
 {
     double sum = 0;
     for(std::size_t i = 0; i < fitness.size(); i++)
     {
         sum+= fitness.at(i);
     }
     for(std::size_t i = 0; i< fitness.size(); i++)
     {
         fitness.at(i) = fitness.at(i)/sum;
     }
 }

 /// computes a fitness value based on the distance of each population
 /// @returns nothing
 void GpsUtils::computeFitness()
 {
     double currentRecord = 100000;
     for(std::size_t i =0; i < population.size(); i++)
     {
        double distance = GetRouteDistance(waypoints, population[i]);
        if(distance < recordDistance)
        {
            recordDistance = distance;
            bestOrder = population[i];
        }
        if(distance < currentRecord)
        {
            currentRecord = distance;
            currentBestOrder = population[i];
            //ROS_INFO("Current Best Distance %f ", currentRecord);
        }
        fitness.at(i) = 1/(pow(distance, 8) + 1);
     }

 }

 /// initialises and populates the fitness, order and ppopulation
 /// @param wp vector containing GPS waypoints
 /// @param MAX_POP maximum number of members of the population
 /// @returns nothing
 void GpsUtils::initialiseGA(std::vector<gcs::Waypoint>& wp, int MAX_POP )
 {
     // delete previous instances of the run
     resetVariables();
     waypoints = wp;
     recordDistance = GetRouteDistance(waypoints);

     
     // create a first set of routes based on initial waypoints
     for(std::size_t i = 0; i < waypoints.size(); i++)
     {
         order.push_back(i);
         bestOrder.push_back(0);
        //  ROS_INFO("Lat: %f", waypoints.at(i).latitude);
     }

     // initialise fitness and best route
     // to zero
     for(int i = 0; i < MAX_POP; i++)
     {
         fitness.push_back(0);
     }

     // add list of random routes
     // and shuffle population
     for(int i = 0; i < MAX_POP; i++ )
     {
         population.push_back(order);
         shuffle(population.at(i));
     }
 }

/// Compute the next generation of population.
///
/// Uses a randomised method to pick a parents
/// based on fitness to create a child which is
/// inserted into the next generation of population
 void GpsUtils::nextGeneration()
 {
     // create a new array for the next generation;
     std::vector<std::vector<int>> new_pop;
     // initialise new population to and set previous orders
     for(std::size_t i = 0; i < population.size(); i++)
     {
         new_pop.push_back(order);
     }

     // pick random parents well, bias towards parents with a higher
     // fitness. Then perform crossover and mutation. Mutation rate
     // set to 1% for the moment.
     for(std::size_t i = 0; i < population.size(); i++)
     {
         std::vector<int> parentA = pickRandomParent(population, fitness);
         std::vector<int> parentB = pickRandomParent(population, fitness);
         std::vector<int> child = crossover(parentA, parentB);
         mutate(child, 0.01);

         // reinsert the starting points into the first and last elements
         // of the vector and make circular to fulfill TSP requirements
         child.insert(child.begin(), 0);
         child.insert(child.end(), parentA.size()-1);
         new_pop[i] = child;
     }
     population = new_pop;
 }

 /// pick random parent based on fitness
 /// @param population Population of list of route orders
 /// @param fitness vector containing corresponding fitness values of route orders
 /// @returns a Parent which is made up of an order.
 std::vector<int> GpsUtils::pickRandomParent(std::vector<std::vector<int> > &population, std::vector<float>& fitness)
 {
     int index = 0;
     // returns a random number between 0 and 1;
     double r = ((double) rand() / (RAND_MAX));
     while (r > 0)
     {
         r = r - fitness.at(index);
         index++;
     }
     index--;
     return population.at(index);
 }

  /// performs mutation on the child using a given mutation rate
 /// @param order Child to be mutated
 /// @param mutationRate mutation rate
 /// @returns nothing
 void GpsUtils::mutate(std::vector<int> &order, float mutationRate)
 {
     for(std::size_t i = 0; i < waypoints.size(); i++)
     {
        double r = ((double) rand() / (RAND_MAX));
        if(r < mutationRate)
        {
            // omit the first and last indexes to preserve the
            // circular requirement of home and finishing
            // positions
            int indexA = rand() % (order.size()-2) + 1;
            int indexB = (indexA + 1) % (waypoints.size()-2);
            swap(order, indexA, indexB);
        }
     }
 }

 /// creates a new child based on both parents
 /// @param parentA the first parent
 /// @param parentB the second parent
 /// @returns a child with a route missing two elements.
 /// @see nextGeneration()
 std::vector<int> GpsUtils::crossover(std::vector<int> &parentA, std::vector<int> &parentB)
 {
     // pick a random number from 1 and omit the last element
     // in the array
     int start = rand () % (parentA.size()-2) + 1 ;
     // pick a random number from "start" and omit the last
     // element in the array
     int end = start + 1 + rand() % ((parentA.size()-2) - start +1);

     //create a new vector based from the parent with index (start, end)
     std::vector<int>::const_iterator first = parentA.begin() + start;
     std::vector<int>::const_iterator second = parentA.begin() + end;
     std::vector<int> newOrder(first, second);

     // Add remaining chromosomes from parentB. Check to see if they
     // are missing from ParentA first
     for(std::size_t i = 1; i <= parentB.size()-2; i++)
     {
         int city = parentB.at(i);
         if(std::find(newOrder.begin(), newOrder.end(), city) == newOrder.end())
         {
             //std::cout<< "Move on" <<std::endl;
             newOrder.push_back(city);
         }
     }
     return newOrder;
 }

/// Accessor method to return best order after running 
/// TSP Algorithm
/// @returns vector containing best route. 
std::vector<int> GpsUtils::getBestOrder()
{
    // for(int i = 0; i < bestOrder.size(); i++)
    // {
    //     ROS_INFO("Order --- %d", bestOrder[i]);
    // }
     return bestOrder;
     
}

double GpsUtils::getBestDistance()
{
    return recordDistance;
}

/// Clears all variables from previous run
/// @returns nothing
void GpsUtils::resetVariables()
{
    bestOrder.clear();
    route.clear();
    waypoints.clear();
    fitness.clear();  
    currentBestOrder.clear();
    order.clear();
    population.clear();

}
