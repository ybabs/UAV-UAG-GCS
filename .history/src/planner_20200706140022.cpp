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
  gcs::Action msg;
  msg.header.stamp = ros::Time::now();
  msg.droneaction = 2;
  drone_action_publisher.publish(msg);
  ROS_INFO("Land Button Pressed");
}

void GCS::goHome()
{
  gcs::Action msg;
  msg.header.stamp = ros::Time::now();
  msg.droneaction = 3;
  drone_action_publisher.publish(msg);
  ROS_INFO("Go Home pressed");
}

void GCS::takeoff()
{
  gcs::Action msg;
  
  msg.header.stamp = ros::Time::now();
  msg.droneaction = 1;
  drone_action_publisher.publish(msg);
  ROS_INFO("Takeoff Pressed");
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
    if(id == 1)
    {
      active_mavs[0] = 1;  
      for(auto &mav: active_mavs )
      {
        ROS_INFO( "Mav %d", mav);
      }
    }

    else if(id == 2)
    {
      active_mavs[1] = 1;  
      for(auto &mav: active_mavs )
      {
        ROS_INFO( "Mav %d", mav);
      }
    }

    else if(id == 3)
    {
      active_mavs[2] = 1;
      for(auto &mav: active_mavs )
      {
        ROS_INFO( "Mav %d", mav);
      }  
    }

    else if(id == 4)
    {
      active_mavs[3] = 1;  
      
      for(auto &mav: active_mavs )
      {
        ROS_INFO( "Mav %d", mav);
      }
    } 

    if(id == 255)
    {
       for (auto &mav : active_mavs)
        {
          mav = 1;
          ROS_INFO( "Mav %d", mav);
        }
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
  for( auto &itr : active_mavs)
   {
     itr = 0;
   }


}

void GCS::uploadWaypoints()
{
  armMav();
  ROS_INFO("Size of messages %d", transect_list.size());

  for(auto & waypoint: transect_list)
  {
     waypoint_publisher.publish(waypoint);
  }

  transect_list.clear();

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
    speedSliderChanged();
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
  msg.uavSpeed = uav_speed;
  
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
  gcs::Action msg;
  msg.header.stamp = ros::Time::now();
  msg.droneaction = 4;

  // Set and publish mission parameters
  setMissionParams();
  drone_action_publisher.publish(msg);
  ROS_INFO("Starting Mission");
}

void GCS::abortMission()
{
  gcs::Action msg;
  msg.header.stamp = ros::Time::now();
  msg.droneaction = 5;
  drone_action_publisher.publish(msg);
  ROS_WARN("Aborting Mission");
  
}

void GCS::addWaypoint(double lat, double lon,  float alt, int sample, float sampleTime)
{

  gcs::Waypoint msg;
  msg.header.stamp = ros::Time::now();
  msg.latitude = lat;
  msg.longitude = lon;
  msg.altitude = alt;
  msg.sample = sample;
  msg.sampleTime = sampleTime;

  waypoint_publisher.publish(msg);   
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

    start_pos = convertTextToNavSatFix(start.toStdString());
    end_pos = convertTextToNavSatFix(end.toStdString());

    double bearing = gpsGenerator.ComputeBearing(start_pos, end_pos);

  

    gcs::Waypoint msg;
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