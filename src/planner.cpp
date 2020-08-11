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

void GCS::uploadWaypoints()
{
  armMav();
  std::vector<int> active_mav_copy;  // copy contents of active UAVs. 
  active_mav_copy = active_mavs;
  int active_key = 1;
  int index;
  
  int n  = std::count(active_mav_copy.begin(), active_mav_copy.end(), 1); // number of active mavs

  if(n > 0)
  {

    std::vector<std::vector<gcs::Waypoint>> truncated_waypoints = splitWaypoints(transect_list, n);      
    int wp_size = truncated_waypoints.size();

    ROS_INFO("Number of active Waypoints %d", wp_size);
    ROS_INFO("Number of active drones %d", n);

      for (auto& row: truncated_waypoints)
      {
        // find first active index
        std::vector<int>::iterator it = std::find(active_mav_copy.begin(), active_mav_copy.end(), active_key);
        if(it != active_mav_copy.end())
        {
          index = std::distance(active_mav_copy.begin(), it);
          ROS_INFO("\nUAV %d activated", index + 1 );
          active_mav_copy[index] = 0;
        }
        else
        {
          ROS_WARN("No active UAVs found");
        }
        for(auto& waypoint: row)
        {          
          waypoint.id = index;
          waypoint_publisher.publish(waypoint);
          ROS_INFO("ID: %d, Lat: %f, Lon: %f", waypoint.id, waypoint.latitude, waypoint.longitude);
          ros::Duration(0.001).sleep();
        }
      }

      transect_list.clear();
      // for( auto &itr : active_mavs)
      //   {
      //     itr = 0;
      //   }

  }

  else
  {
    ROS_ERROR("No Drones active, Check");
  }
}

std::vector<std::vector<gcs::Waypoint>> GCS::splitWaypoints(std::vector<gcs::Waypoint>& vec , size_t n)
{

  std::vector<std::vector<gcs::Waypoint>> rtn;

  size_t length = vec.size() / n;
  size_t remain = vec.size() % n;

  size_t begin = 0;
  size_t end = 0;

  for(size_t i = 0; i < std::min(n, vec.size()); i++)
  {
    end += (remain > 0) ? (length + !!(remain--)) : length;

    rtn.push_back(std::vector<gcs::Waypoint>(vec.begin() + begin, vec.begin() + end));

    begin = end;
  }

  std::vector<QGeoCoordindate> uav_home = model.getUavPositions();
  double distance;
  double min_distance;


  // Sort Waypoints according to distance. 
  for(auto i = rtn.begin(); i != rtn.end(); ++i)
  {
    for(auto j = i.begin(); j !=i.end(); ++j)
    {
      min_distance = gpsGenerator.GetPathLength()
      

    }
  }

  return rtn;
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
  publishMessage(msg, 4);
  ROS_INFO("Starting Mission");
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

  publishMessage(msg);
  ROS_INFO("Waypoint Added:, Lat: %f, Lon: %f, Alt: %f , Sample: %d, SampleTime: %f",  lat, lon, alt,
                                                                                 sample, sampleTime);
   
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

void GCS::addGeneratedWaypoints(QString start, QString end, int num_locations)
{
  qml_gps_points.clear();
  //ROS_INFO("Size before: %d", qml_gps_points.length());
    // Process incoming strings
    sensor_msgs::NavSatFix start_pos;
    sensor_msgs::NavSatFix end_pos;
    std::vector<sensor_msgs::NavSatFix> uav_route;
    gcs::Waypoint iterated_position;

    if(start.isEmpty() || end.isEmpty() || num_locations == 0)
    {
      ROS_ERROR("check input arguments");
    }

    else
    {

      start_pos = convertTextToNavSatFix(start.toStdString());
      end_pos = convertTextToNavSatFix(end.toStdString());

      double bearing = gpsGenerator.ComputeBearing(start_pos, end_pos);
      uav_route = gpsGenerator.returnPositionsBasedOnLocations(num_locations, bearing, start_pos, end_pos);
      //ROS_INFO("UAV ROUTE Size %d", uav_route.size());
      QGeoCoordinate start_coord;
      QGeoCoordinate end_coord;

      start_coord.setLatitude(start_pos.latitude);
      start_coord.setLongitude(start_pos.longitude);
      end_coord.setLatitude(end_pos.latitude);
      end_coord.setLongitude(end_pos.longitude);

      qml_gps_points.append(start_coord);

      for(int i = 0; i < uav_route.size(); i++)
      {
          QGeoCoordinate p_single ;
          p_single.setLatitude(uav_route[i].latitude) ;
          p_single.setLongitude(uav_route[i].longitude);
          qml_gps_points.append(p_single);

          iterated_position.latitude = uav_route[i].latitude;
          iterated_position.longitude = uav_route[i].longitude;
          iterated_position.altitude = 10; // TODO define this later
          iterated_position.sample = 1; // TODO Define this later
          iterated_position.sampleTime = 20; // TODO definet his later
          
          transect_list.push_back(iterated_position);

        }

        qml_gps_points.append(end_coord);
        uav_route.clear();
        //ROS_INFO("Size after: %d", qml_gps_points.length());

    }  

}

void GCS:: generateDisks(QString center, double distance, double samplingTime)
{
  qml_gps_points.clear();
  QVector<QGeoCoordinate>bounding_box;
  sensor_msgs::NavSatFix center_point;
  gcs::Waypoint iterated_position;

  if(center.isEmpty() || distance == 0)
  {
    ROS_ERROR("Check input arguments again");
  }

  else
  {

    ROS_INFO ("Distance is %f", distance);
  center_point = convertTextToNavSatFix(center.toStdString());
  
  // bounding boxes are in a clockwise dfirction
   sensor_msgs::NavSatFix top_right = gpsGenerator.GetDestinationCoordinate(center_point, 45, distance);
   sensor_msgs::NavSatFix bottom_right = gpsGenerator.GetDestinationCoordinate(center_point, 135, distance);
   sensor_msgs::NavSatFix top_left = gpsGenerator.GetDestinationCoordinate(center_point, 315, distance);
   sensor_msgs::NavSatFix bottom_left = gpsGenerator.GetDestinationCoordinate(center_point, 225, distance);

  // first 4 elements in the vecvtor represents the bouding box.
  QGeoCoordinate q_top_right = convertNavSatFixToQGeoCoordinate(top_right);
   QGeoCoordinate q_top_left = convertNavSatFixToQGeoCoordinate(top_left);
    QGeoCoordinate q_bottom_right = convertNavSatFixToQGeoCoordinate(bottom_right);
     QGeoCoordinate q_bottom_left = convertNavSatFixToQGeoCoordinate(bottom_left);

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
  sensor_msgs::NavSatFix initial_position;
  initial_position.latitude = bottom_left.latitude;
  initial_position.longitude = bottom_left.longitude;

  std::vector <std::pair <double, double>> points;
  std::vector <sensor_msgs::NavSatFix> uav_route;

  double x_init = 0;
  double y_init = 0;

  for (int i = 1; i <= arr_size; i++)
  {
    for(int j = 1; j <= arr_size; j++)
    {
        double curr_x  = x_init + (2*i - 1) * d/2;
        double curr_y = y_init + (2*j - 1) * d/2;
        ROS_INFO("X: %f" "Y: %f", curr_x, curr_y);

        points.push_back(std::make_pair(curr_x, curr_y));
    }
  }


   int length = points.size();

  sensor_msgs::NavSatFix prev;
  sensor_msgs::NavSatFix curr;
  double prev_x_pos;
  double prev_y_pos; 

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

        iterated_position.latitude = uav_route[i].latitude;
        iterated_position.longitude = uav_route[i].longitude;
        iterated_position.altitude = 10; // TODO define this later
        iterated_position.sample = 1; // TODO Define this later
        iterated_position.sampleTime = samplingTime; // TODO definet his later
        
        transect_list.push_back(iterated_position);

    }

  }



}


 sensor_msgs::NavSatFix GCS::convertTextToNavSatFix(std::string input_string)
 {
     std::vector<double> result;
     sensor_msgs::NavSatFix gps_output;

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

 QGeoCoordinate GCS::convertNavSatFixToQGeoCoordinate(sensor_msgs::NavSatFix &input)
 {
   QGeoCoordinate output ;

   output.setLatitude(input.latitude);
   output.setLongitude(input.longitude);
   output.setAltitude(input.altitude);

   return output;
 }
