#include "gcs/planner/planner.h"


GCS::GCS():
active_mavs(4,0)
{

  initPublishers();

  ROS_INFO("Con Planner");

    
}

GCS::~GCS()
{
    
}

void GCS::initPublishers()
{

  mission_param_publisher = nh.advertise<gcs::Missionparameters>("gcs/params", 10);
  waypoint_publisher = nh.advertise<gcs::Waypoint>("gcs/waypoint", 10);
  mission_pause_publisher = nh.advertise<std_msgs::UInt8>("gcs/mission_pause", 1);
  drone_action_publisher = nh.advertise<gcs::Action>("gcs/mission_action", 1);
  active_mav_publisher = nh.advertise<std_msgs::Int32MultiArray>("gcs/active_mavs", 10);

}

void GCS::land()
{
  armMav();
  gcs::Action msg;
  
  publishMessage(msg, 2);
  ROS_INFO("Landing");

}

void GCS::goHome()
{
  armMav();
  gcs::Action msg;
  
  publishMessage(msg, 3);
  
}

void GCS::publishMessage(gcs::Action &msg, int action)
{
  std::vector<int> active_mav_copy;  // copy contents of active UAVs. 
  active_mav_copy = active_mavs;
  int index;

   int n = std::count(active_mav_copy.begin(), active_mav_copy.end(), 1);
   ROS_INFO("Active Drones %d", n);
  for(int i = 0; i < n; i++)
  {
      std::vector<int>::iterator it = std::find(active_mav_copy.begin(), active_mav_copy.end(), 1);

      if(it != active_mav_copy.end())
      {
        index = std::distance(active_mav_copy.begin(), it);
        ROS_INFO("UAV %d activated", index + 1 );
        active_mav_copy[index] = 0;
      }
      else
      {
        ROS_WARN("No active UAVs found");
      }
     
      msg.header.stamp = ros::Time::now();
      msg.id = index;
      msg.droneaction = action;
      drone_action_publisher.publish(msg);
  } 

}

void GCS:: publishMessage(gcs::Waypoint &msg)
{
  std::vector<int> active_mav_copy;  // copy contents of active UAVs. 
  active_mav_copy = active_mavs;
  int index;

  int n = std::count(active_mav_copy.begin(), active_mav_copy.end(), 1);
  ROS_INFO("Active Drones %d", n);
  for(int i = 0; i < n; i++)
  {
      std::vector<int>::iterator it = std::find(active_mav_copy.begin(), active_mav_copy.end(), 1);

      if(it != active_mav_copy.end())
      {
        index = std::distance(active_mav_copy.begin(), it);
        ROS_INFO("UAV %d activated", index + 1 );
        active_mav_copy[index] = 0;
      }
      else
      {
        ROS_WARN("No active UAVs found");
      }
     
      msg.id = index;

       waypoint_publisher.publish(msg);   
  } 
}

void GCS::takeoff()
{
  armMav();
  gcs::Action msg;

  publishMessage(msg, 1);

}

int GCS::getMavId()
{
  return mav_id;
}

void GCS::setMavId(int id)
{
  if(mav_id != id)
  {
    mav_id = id;

    switch(id)
    {
      case 1:
        active_mavs[0] = 1;
        for(auto &mav: active_mavs )
        {
          ROS_INFO( "Mav %d", mav);
        }
        break;

        case 2:
          active_mavs[1] = 1; 
          for(auto &mav: active_mavs )
          {
            ROS_INFO( "Mav %d", mav);
          }
          break;

        case 3:
          active_mavs[2] = 1;
          for(auto &mav: active_mavs )
          {
            ROS_INFO( "Mav %d", mav);
          }
          break;
         
         case 4:
          active_mavs[3] = 1;
          for(auto &mav: active_mavs )
          {
            ROS_INFO( "Mav %d", mav);
          }
          break;

          case 10:
            active_mavs[0] = 0;
            for(auto &mav: active_mavs )
            {
              ROS_INFO( "Mav %d", mav);
            }
            break;

          case 20:
            active_mavs[1] = 0;
            for(auto &mav: active_mavs )
            {
              ROS_INFO( "Mav %d", mav);
            }   
            break;

          case 30:
            active_mavs[2] = 0;
            for(auto &mav: active_mavs )
            {
              ROS_INFO( "Mav %d", mav);
            }
            break;

          case 40:
            active_mavs[3] = 0;
            for(auto &mav: active_mavs )
            {
              ROS_INFO( "Mav %d", mav);
            }
            break;

            default:
            break;
    }
    mavIdSet();
  }
 
}

/// sets active UAVs and publishes a message
/// to drones
/// @returns nothing
void GCS::armMav()
{
   std_msgs::Int32MultiArray activeMav_msg;

   activeMav_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
   activeMav_msg.layout.dim[0].size = active_mavs.size();
   activeMav_msg.layout.dim[0].stride = 1;

   for( auto &itr : active_mavs)
   {
     activeMav_msg.data.push_back(itr);
   }

   active_mav_publisher.publish(activeMav_msg);
  
  // call this after uploading?
}

/// uploads waypoints to each UAV based 
/// on the number of active UAVs set
/// Also uses splitWatpoints to divide 
/// waypoints between UAVs
/// @see splitWaypoints()
void GCS::uploadWaypoints()
{
  // set active UAVs for drones
  armMav();
  std::vector<int> active_mav_copy;  // copy contents of active UAVs. 
  active_mav_copy = active_mavs;
  // set active key required to search which UAVs are active
  int active_key = 1;
  int index;
  
  //return number of active mavs
  int active_uav_count  = std::count(active_mav_copy.begin(), active_mav_copy.end(), active_key); 

  ROS_INFO("Transect List-------");

  // for(int i = 0; i < transect_list.size(); i++)
  // {
  //   ROS_INFO("Lat: %f, Lon %f:", transect_list.at(i).latitude, transect_list.at(i).longitude);
  // }

  // If there are active UAVs
  if(active_uav_count > 0)
  {
  
    // split waypoints based on the number of active uavs.
    std::vector<std::vector<gcs::Waypoint>> truncated_waypoints = mtspTour(transect_list, active_uav_count);      
    int wp_size = truncated_waypoints.size();

    ROS_INFO("Waypoint Routes split %d", wp_size);
    ROS_INFO("Number of active drones %d", active_uav_count);

    // for each set of routes within the waypoint vectors
      for (auto& row: truncated_waypoints)
      {
        // find first active UAV index
        std::vector<int>::iterator it = std::find(active_mav_copy.begin(), active_mav_copy.end(), active_key);
        if(it != active_mav_copy.end())
        {
          index = std::distance(active_mav_copy.begin(), it);
          ROS_INFO("\nUAV %d activated", index + 1 );
          // deactivate this index and set to zero to enable
          // us to find the next active index
          active_mav_copy[index] = 0;
        }
        else
        {
          ROS_WARN("No active UAVs found");
        }
        // For each set of waypoints within the row.  
        for(auto& waypoint: row)
        {          
          waypoint.id = index;
          waypoint_publisher.publish(waypoint);
          ROS_INFO("ID: %d, Lat: %f, Lon: %f", waypoint.id, waypoint.latitude, waypoint.longitude);
          ros::Duration(0.001).sleep();
        }
      }
      // clear the list of waypoints.
      transect_list.clear();
  }

  else
  {
    ROS_ERROR("No Drones active, Check to see if any were selected");
  }
}



void GCS::tspTour(std::vector<gcs::Waypoint> &tsp_wp)
{
  
  std::vector<int> routeOrder;
  std::vector<gcs::Waypoint> new_route;
  int NUM_GEN = 100;
  int num_generations = 0;
  //TODO determine Home position based on the Drone selected here
  std::vector<QGeoCoordinate> home_positions = model.getUavPositions();
  gcs::Waypoint uav_home;
  uav_home.latitude = home_positions[0].latitude();
  uav_home.longitude =home_positions[0].longitude();
  tsp_wp.insert(tsp_wp.begin(), uav_home);
  tsp_wp.insert(tsp_wp.end(), uav_home);

  // for(int i = 0; i < tsp_wp.size(); i++)
  // {
  //   ROS_INFO("Lat: %f", tsp_wp.at(i).latitude);
  // }

  tsp.initialiseGA(tsp_wp, 300);

      // find best route for each set of waypoints. 
    // can tweak number of generations here. 
    while(num_generations < NUM_GEN)
    {
          // ROS_INFO("computing fitness ");
          tsp.computeFitness();
         // ROS_INFO("Fitness Done ");
          tsp.normalizeFitness();
          // ROS_INFO("Fitness normalised ");
          tsp.nextGeneration();
          //ROS_INFO("Next Generation populated ");
          num_generations++;
          //ROS_INFO("GEn..... %d", num_generations);
    }

    routeOrder = tsp.getBestOrder();
    ROS_INFO("Best distance is %f", tsp.getBestDistance());
    ROS_INFO("TSP Size %lu", tsp_wp.size());
     // shuffle elements according to route order 
    // and add new route to vector
    for(std::size_t n = 1; n < routeOrder.size()-1; n++)
    {
      int indexA = routeOrder[n];
     // ROS_INFO("index = %d", indexA);
      new_route.push_back(tsp_wp.at(indexA));
    }

  num_generations = 0;
  tsp_wp = new_route;


}


std::vector<std::vector<gcs::Waypoint>> GCS::mtspTour(std::vector<gcs::Waypoint> &tsp_wp,
                                                     std::size_t nd)
{
  // Use position of drone 1 to set the starting and
  // end points of the drones

 std::vector<QGeoCoordinate> home_positions = model.getUavPositions();

 gcs::Waypoint start_location;
 start_location.latitude = home_positions[0].latitude();
 start_location.longitude = home_positions[0].longitude();
 tsp_wp.insert(tsp_wp.begin(), start_location);
 tsp_wp.insert(tsp_wp.end(), start_location);

   mtsp.initialiseGA(tsp_wp, nd);

  std::vector<std::vector<size_t>> sol = mtsp.getBestOrder();
  std::vector<std::vector<gcs::Waypoint>> route(sol.size(), std::vector<gcs::Waypoint>());


  // Create a new route for the UAVs 
    for(std::size_t i = 0; i < sol.size(); i++)
    {
        for(std::size_t j = 0; j < sol.at(i).size(); j++)
        {
            int indexA = sol.at(i).at(j);
            route.at(i).push_back(tsp_wp.at(indexA));
        }
    }

    return route;  
   
}


int GCS::getDroneSpeed()
{
  return uav_speed;
}

void GCS::setDroneSpeed(int speed)
{
  if(uav_speed != speed)
  {
    uav_speed = speed;
    speedTextChanged();
  }

}

void GCS::setHoverFlag(int flag)
{
  if(hover_flag != flag)
  {
    hover_flag = flag;
    hoverFlagSet();
  }

}
    
int GCS::getHoverFlag()
{
  return hover_flag;
}

void GCS::setRthFlag(int flag)
{
  if(rth_flag != flag)
  {
    rth_flag = flag;
    rthFlagSet();
  }

}

int GCS::getRthFlag()
{
   return rth_flag;
}

void GCS::setLandFlag(int flag)
{
  if(land_flag !=flag)
  {
    land_flag = flag;
    landFlagSet();
  }
}

int GCS::getLandFlag()
{
  return land_flag;
}

void GCS::setMissionParams()
{
  gcs::Missionparameters msg;
  msg.header.stamp = ros::Time::now();
 // msg.uavSpeed = uav_speed;
  msg.uavSpeed = 5;
  
  if(hover_flag == 1 & rth_flag == 0 & land_flag == 0)
  {
      msg.missionEndAction = 1;
  }
  if(hover_flag == 0 & rth_flag == 1 & land_flag == 0)
  {
    msg.missionEndAction = 2;    
  }
  if(hover_flag == 0 & rth_flag == 0 & land_flag == 1)
  {
    msg.missionEndAction = 3;  
  }

  mission_param_publisher.publish(msg);

}

void GCS::setAltitude(float alt)
{
  if (target_altitude != alt)
  {
    target_altitude = alt;
    altitudeValueChanged();
  }
}

float GCS::getAltitude()
{
  return target_altitude;
}

void GCS::setSamplingFlag(int flag)
{

  if(sampling_flag != flag)
  {
    sampling_flag = flag;
    samplingFlagSet();
  }

}

int GCS::getSamplingFlag()
{
  return sampling_flag;
}

QVariantList GCS::getPointVector()
{
    QVariantList l;
    for (QGeoCoordinate &p : qml_gps_points)
    {
        l << QVariant::fromValue(p);
    }

    return l;
}

void GCS::setSamplingTime(float sample_time)
{

  if(sampling_time != sample_time)
  {
    sampling_time = sample_time;
    samplingTimeSet();
  }

}
float GCS::getSamplingTime()
{

  return sampling_time;

}

void GCS::startMission()
{
  armMav();
  gcs::Action msg;
  // Set and publish mission parameters
  setMissionParams();
  // ToDO only process single Mission
  // Waypoints if we are running missions for a single drone
  processMissionWaypoints();
  publishMessage(msg, 4);
  ROS_INFO("Starting Mission");
}

void GCS::processMissionWaypoints()
{  
   // tsp tour
   tspTour(single_mission_list);

   for(auto msg : single_mission_list)
   {
     publishMessage(msg);
   }
 
}

void GCS::abortMission()
{
  armMav();
  gcs::Action msg;
  publishMessage(msg, 5);
  ROS_WARN("Aborting Mission");
  
}

void GCS::addWaypoint(double lat, double lon,  float alt, int sample, float sampleTime)
{
  armMav();

  gcs::Waypoint msg;
  msg.header.stamp = ros::Time::now();
  msg.latitude = lat;
  msg.longitude = lon;
  msg.altitude = alt;
  msg.sample = sample;
  msg.sampleTime = sampleTime;
  // Add each waypoint to a vector
  single_mission_list.push_back(msg);
  ROS_INFO("Waypoint Added:, Lat: %f, Lon: %f, Alt: %f , Sample: %d, SampleTime: %f",  lat, lon, alt, sample, sampleTime);                                                                                 

}

void GCS::setPlayPause(bool pause)
{

  std_msgs::UInt8 msg;
  if(mission_pause != pause)
  {
    mission_pause = pause;
    pauseSet();
    ROS_INFO("Pause %d", mission_pause);
  }

  msg.data = mission_pause;
  mission_pause_publisher.publish(msg);

}
bool GCS::getPlayPause()
{
   return mission_pause;
}

///Generates a list of waypoints when the "Generate" Button is clicked on 
/// the GUI based on the start and end poisitons and number of waypoints
/// required. 
/// @param start Start GPS position in string format. This parameter is converted from
/// a string to a gcs::Waypoint type in order to extract the Latitude and Longitude
/// @param end End GPS position in string format. This parameter is converted from
/// a string to a gcs::Waypoint type in order to extract the Latitude and Longitude
/// @param num_locations number of intermediate waypoints between the start and end 
/// coordinates
/// @returns nothing
void GCS::addGeneratedWaypoints(QString start, QString end, int num_locations)
{
  // clear UI waypoints 
  qml_gps_points.clear();
    // Process incoming strings
    gcs::Waypoint start_pos;
    gcs::Waypoint end_pos;
    // used to store the number of interpolated waypoints
    std::vector<gcs::Waypoint> uav_route;
    // check if there is valid text in arguments
    if(start.isEmpty() || end.isEmpty() || num_locations == 0)
    {
      ROS_ERROR("check input arguments");
    }
    else
    {
      start_pos = convertTextToWaypoint(start.toStdString());
      end_pos = convertTextToWaypoint(end.toStdString());

      // calculate bearing of start position to end position
      // and then calculate the number of waypoints recquired
      double bearing = gpsGenerator.ComputeBearing(start_pos, end_pos);
      uav_route = gpsGenerator.returnPositionsBasedOnLocations(num_locations, bearing, start_pos, end_pos);
     
      QGeoCoordinate start_coord;
      QGeoCoordinate end_coord;

      // reinsert conversion of text waypoints
      // to qml waypoints to be displayed in UI 
      start_coord.setLatitude(start_pos.latitude);
      start_coord.setLongitude(start_pos.longitude);
      end_coord.setLatitude(end_pos.latitude);
      end_coord.setLongitude(end_pos.longitude);

      qml_gps_points.append(start_coord);

      // Add waypoints to the:
      // QML UI Vector
      // Add altitude and sample times to 
      // each waypoint and add to the transect list which
      // is to be published

      for(int i = 0; i < uav_route.size(); i++)
      {
          QGeoCoordinate p_single ;
          p_single.setLatitude(uav_route[i].latitude) ;
          p_single.setLongitude(uav_route[i].longitude);
          qml_gps_points.append(p_single);

          uav_route[i].altitude = 10; // TODO define this later
          uav_route[i].sample = 1; // TODO Define this later
          uav_route[i].sampleTime = 20; // TODO define this later
          
          transect_list.push_back(uav_route[i]);

        }
        qml_gps_points.append(end_coord);
        uav_route.clear();
    }  

}

/// Generates recording locations based on the hydrophone recording range 
/// and total distance set by user.
/// @param center A center waypoint string text which is converted to GPS 
/// coordinates
/// @param distance A user defined maximum diamter of the coverage area
/// @param samplingTime Sampling time at each waypoint
/// @returns nothing
void GCS:: generateDisks(QString center, double distance, double samplingTime)
{
  // clear the UI waypoints as this variable is used 
  // by addgeneratedWaypoints as well. 
  qml_gps_points.clear();
  
  // declare variables to be used for bounding boxes and 
  // center points etc
  QVector<QGeoCoordinate>bounding_box;
  gcs::Waypoint center_point;

  // check input parameters are valid
  if(center.isEmpty() || distance == 0)
  {
    ROS_ERROR("Check input arguments again");
  }

  else
  {

    ROS_INFO ("Distance is %f", distance);
   center_point = convertTextToWaypoint(center.toStdString());
  
  // bounding boxes are in a clockwise dfirction
   gcs::Waypoint top_right = gpsGenerator.GetDestinationCoordinate(center_point, 45, distance);
   gcs::Waypoint bottom_right = gpsGenerator.GetDestinationCoordinate(center_point, 135, distance);
   gcs::Waypoint top_left = gpsGenerator.GetDestinationCoordinate(center_point, 315, distance);
   gcs::Waypoint bottom_left = gpsGenerator.GetDestinationCoordinate(center_point, 225, distance);

   // first 4 elements in the vector represents the bouding box.
   QGeoCoordinate q_top_right = convertWaypointToQGeoCoordinate(top_right);
   QGeoCoordinate q_top_left = convertWaypointToQGeoCoordinate(top_left);
   QGeoCoordinate q_bottom_right = convertWaypointToQGeoCoordinate(bottom_right);
   QGeoCoordinate q_bottom_left = convertWaypointToQGeoCoordinate(bottom_left);

   //push into UI vector
    qml_gps_points.push_back(q_top_right);
    qml_gps_points.push_back(q_bottom_right);
    qml_gps_points.push_back(q_top_left);
    qml_gps_points.push_back(q_bottom_left);

  // find the distance between two points to determine the length of the square
  double l = gpsGenerator.GetPathLength(bottom_left, bottom_right);
  ROS_INFO("DIstance: %f", l);
  int arr_size;

  // determine range of hydrophone //TODO from experiments
  double h_radius = 38 * sqrt(2); // multiply 
  double d = sqrt(2) * h_radius;
  double n_points = l/d;
  arr_size = floor(n_points) + 1;

  // Use bottom left as starting position
  gcs::Waypoint initial_position;
  initial_position.latitude = bottom_left.latitude;
  initial_position.longitude = bottom_left.longitude;

  std::vector <std::pair <double, double>> points;
  std::vector <gcs::Waypoint> uav_route;

  double x_init = 0;
  double y_init = 0;

  // Calculate the cartesian points on the square
  for (int i = 1; i <= arr_size; i++)
  {
    for(int j = 1; j <= arr_size; j++)
    {
        double curr_x  = x_init + (2*i - 1) * d/2;
        double curr_y = y_init + (2*j - 1) * d/2;
        //ROS_INFO("X: %f" "Y: %f", curr_x, curr_y);
        points.push_back(std::make_pair(curr_x, curr_y));
    }
  }

  int length = points.size();

  gcs::Waypoint prev;
  gcs::Waypoint curr;
  double prev_x_pos;
  double prev_y_pos; 

 // generate the GPS positions from the cartesian lengths
  for(int i = 0; i < points.size(); i ++)
  {
    double x_pos = points[i].first;
    double y_pos = points[i].second;

    double bearing; 
    double length;

    // first case is always at a 45 degree angle
    if(i == 0)
    {
      length = sqrt( pow(x_pos, 2) + pow(y_pos, 2));
      bearing = 45;
      curr = gpsGenerator.GetDestinationCoordinate(initial_position, bearing, length);
    }

    else
    {
        double x_dir = x_pos - prev_x_pos;
        double y_dir = y_pos - prev_y_pos;
        length = sqrt(pow(x_dir, 2) + pow(y_dir, 2));
        bearing = atan2(x_dir, y_dir);
        double brn_drg = RadToDeg(bearing);
        brn_drg = (brn_drg >= 0) ? brn_drg : brn_drg + 2 * PI;
        curr = gpsGenerator.GetDestinationCoordinate(prev, brn_drg, length);
    }
   // Add GPS positions to the routes
    uav_route.push_back(curr);
    prev = curr;
    prev_x_pos = x_pos;
    prev_y_pos = y_pos;
  }

  for(int i = 0; i < uav_route.size(); i++)
    {
        QGeoCoordinate p_single ;
        p_single.setLatitude(uav_route[i].latitude) ;
        p_single.setLongitude(uav_route[i].longitude);
        qml_gps_points.append(p_single);

        uav_route[i].altitude = 10; // TODO define this later
        uav_route[i].sample = 1; // TODO Define this later
        uav_route[i].sampleTime = samplingTime; // TODO definet his later
        
        transect_list.push_back(uav_route[i]);
    }
  }
}

/// converts an input string to a gcs::Waypoint type
/// @param input_string a string parameter
/// @returns a gcs::Waypoint variable containing
/// GPS latitude and longitude
 gcs::Waypoint GCS::convertTextToWaypoint(std::string input_string)
 {
     std::vector<double> result;
     gcs::Waypoint gps_output;

     std::stringstream s_stream(input_string);

         while(s_stream.good())
    {
        std::string substr;
        std::getline(s_stream, substr, ',');
        double value = std::stod(substr);
        result.push_back(value);
    }

    gps_output.latitude = result[0];
    gps_output.longitude = result[1];

    return gps_output;
 }

/// converts a gcs::Waypoint type to a 
/// QGeocoordinate type
/// @param input a gcs::Waypoint parameter
/// @returns a QGeoCoordinate variable containing
/// GPS latitude and longitude
 QGeoCoordinate GCS::convertWaypointToQGeoCoordinate(gcs::Waypoint &input)
 {
   QGeoCoordinate output ;
   output.setLatitude(input.latitude);
   output.setLongitude(input.longitude);
   output.setAltitude(input.altitude);
   return output;
 }
