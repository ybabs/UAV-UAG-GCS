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

double GpsUtils::GetPathLength(QGeoCoordinate start_coord, QGeoCoordinate end_coord)
{
    double lat1_rad = DegToRad(start_coord.latitude());
    double lat2_rad = DegToRad(end_coord.latitude());
    double delta_lat_deg = end_coord.latitude() - start_coord.latitude();
    double delta_lat = DegToRad(delta_lat_deg);
    double delta_lon_deg = end_coord.longitude() - start_coord.longitude();
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

 
