#include "qml_gui/planner/waypointgenerator.h"

GpsUtils::GpsUtils()
{

}

// Calauclate the distance between two lat, long coordinate pairs
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

// return latitude and longitude of a destination point give, start coordinate and distance
QGeoCoordinate GpsUtils::GetDestinationCoordinate(QGeoCoordinate start_coord, double azimuth, double distance)
{
    double radius_km = C_EARTH/1000; // Earth's Radius in Km
    double bearing = DegToRad(azimuth); // degrees converted to Rad
    double d = distance / 1000; // convert distance in metres to km
    double lat1 = DegToRad(start_coord.latitude());
    double lon1 = DegToRad(start_coord.longitude());
    double lat2 = asin(sin(lat1) * cos(d/radius_km) + cos(lat1) * sin(d/radius_km) * cos(bearing));
    double lon2 = lon1 + atan2(sin(bearing) * sin(d/radius_km) * cos(lat1), cos(d/radius_km) -  sin(lat1) * sin(lat2));

    lat2 = RadToDeg(lat2);
    lon2 = RadToDeg(lon2);

    QGeoCoordinate destination;

    destination.setLatitude(lat2);
    destination.setLongitude(lon2);

    return destination;

}

// calculate azimuth/bearing in degrees from start point to endpoint
double GpsUtils::ComputeBearing(QGeoCoordinate start_point, QGeoCoordinate end_point)
{
    double start_lat = DegToRad(start_point.latitude());
    double start_lon = DegToRad(start_point.longitude());
    double end_lat = DegToRad(end_point.latitude());
    double end_lon = DegToRad(end_point.longitude());
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

std::vector<QGeoCoordinate>GpsUtils::returnInterpolatedPoints(int interval, double bearing, QGeoCoordinate start, QGeoCoordinate end)
{
    double d = GetPathLength(start, end);
    std::cout<< "Distance: " << d << std::endl;
    int distance = (int) d / interval;
    int distance_covered = interval;
    // Add first location.
    route.push_back(start);
    for (int i = 0; i < distance; i++)
    {
        QGeoCoordinate new_pos = GetDestinationCoordinate(start, bearing, distance_covered);
        distance_covered+=interval;
        route.push_back(new_pos);
    }
    // add final location
    route.push_back(end);

    return route;
}

std::vector<QGeoCoordinate>GpsUtils::returnPositionsBasedOnLocations(int loc_count, double bearing, QGeoCoordinate start, QGeoCoordinate end)
{
    double total_distance = GetPathLength(start, end);
    double dist = total_distance / loc_count; // return number of points.
    int interval = (int) dist;
    int counter = interval;

    for(int i = 0; i < loc_count; i++)
    {
        QGeoCoordinate new_pos = GetDestinationCoordinate(start, bearing, counter);
        counter+=interval;
        route.push_back(new_pos);
    }
    return route;
}

