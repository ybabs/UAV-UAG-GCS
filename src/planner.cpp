
#include "qml_gui/planner/planner.h"

GCS::GCS()
{

  initPublishers();
    
}

GCS::~GCS()
{
  
}

void GCS::initPublishers()
{

  mission_param_publisher = nh.advertise<qml_gui::missionparameters>("gcs/params", 10);
  waypoint_publisher = nh.advertise<qml_gui::waypoint>("gcs/waypoint", 10);
  mission_pause_publisher = nh.advertise<std_msgs::UInt8>("gcs/mission_pause", 1);
  drone_action_publisher = nh.advertise<qml_gui::action>("gcs/mission_action", 1);

}

void GCS::land()
{
  qml_gui::action msg;
  msg.header.stamp = ros::Time::now();
  msg.droneaction = 2;
  drone_action_publisher.publish(msg);
  ROS_INFO("Land Button Pressed");
}

void GCS::goHome()
{
  qml_gui::action msg;
  msg.header.stamp = ros::Time::now();
  msg.droneaction = 3;
  drone_action_publisher.publish(msg);
  ROS_INFO("Go Home pressed");
}

void GCS::takeoff()
{
  qml_gui::action msg;
  msg.header.stamp = ros::Time::now();
  msg.droneaction = 1;
  drone_action_publisher.publish(msg);
  ROS_INFO("Takeoff Pressed");
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
  qml_gui::missionparameters msg;
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
  qml_gui::action msg;
  msg.header.stamp = ros::Time::now();
  msg.droneaction = 4;

  // Set and publish mission parameters
  setMissionParams();
  drone_action_publisher.publish(msg);
  ROS_INFO("Starting Mission");
}

void GCS::abortMission()
{
  qml_gui::action msg;
  msg.header.stamp = ros::Time::now();
  msg.droneaction = 5;
  drone_action_publisher.publish(msg);
  ROS_WARN("Aborting Mission");
  
}

void GCS::addWaypoint(double lat, double lon,  float alt, int sample, float sampleTime)
{

  qml_gui::waypoint msg;
  msg.header.stamp = ros::Time::now();
  msg.latitude = lat;
  msg.longitude = lon;
  msg.altitude = alt;
  msg.sample = sample;
  msg.sampleTime = sampleTime;

  waypoint_publisher.publish(msg);   
   
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



