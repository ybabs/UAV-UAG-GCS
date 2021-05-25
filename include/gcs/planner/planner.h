#ifndef PLANNER_H
#define PLANNER_H

#include <QObject>
#include <QString>
#include <QTimer>
#include <serial/serial.h>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include "gcs/Waypoint.h"
#include "gcs/Missionparameters.h"
#include "gcs/Action.h"
#include <queue>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <tuple>
#include <QDebug>
#include "gcs/planner/waypointgenerator.h"
#include "gcs/model/uavmodel.h"
#include "gcs/model/tspmodel.h"
#include "gcs/planner/tsp.h"
#include "gcs/planner/mtsp.h"
#include <sstream>
#include <iostream>
#include <algorithm>

#define CA_FLAG_PRIORITY_RELEASED 1
#define CA_FLAG_PRIORITY_FREE 0
#define CA_IN_PROGRESS 2




class GCS: public QObject
{
    Q_OBJECT

    Q_PROPERTY(int droneSpeed READ getDroneSpeed WRITE setDroneSpeed NOTIFY speedTextChanged )
    Q_PROPERTY(int hydrophoneRange READ getHydrophoneRange WRITE setHydrophoneRange NOTIFY hydrophoneRangeChanged)
    Q_PROPERTY(float altitude READ getAltitude WRITE setAltitude NOTIFY altitudeValueChanged )
    Q_PROPERTY(float samplingTime READ getSamplingTime WRITE setSamplingTime NOTIFY samplingTimeSet)
    Q_PROPERTY(int sampleFlag READ getSamplingFlag WRITE setSamplingFlag NOTIFY samplingFlagSet )
    Q_PROPERTY(int missionStatusFlag READ getPlayPause WRITE setPlayPause NOTIFY pauseSet )
    Q_PROPERTY(int missionType READ getMissionType WRITE setMissionType NOTIFY missionTypeChanged)
    Q_PROPERTY(int mavId READ getMavId WRITE setMavId NOTIFY mavIdSet)
    Q_PROPERTY(QVariantList trackpoints READ getPointVector)
    Q_PROPERTY(QVariantList mtspPath READ getMtspVector)
    Q_PROPERTY(QVariantList tspPath READ getTspVector)
    
       

   
    public:
     GCS();
    ~GCS();

    enum MissionState{

        IDLE ,
        NEW_WAYPOINT,
        ARRIVED ,
        FINISHED,
        GO_HOME

    };

    enum MissionType{
        SINGLE,
        SWARM,
        TRANSECT
    };

    enum WaypointTask
    {
        LAND
    };

    int getMavId();
    void setMavId(int id);
    void setDroneSpeed(int speed); //    
    int getDroneSpeed(); //
    void setHydrophoneRange(int range);
    int getMissionType();
    void setMissionType(int mission_type);
    int getHydrophoneRange();
    void setAltitude(float alt); //
    float getAltitude(); //
    void setSamplingFlag(int flag);
    int getSamplingFlag();
    void setSamplingTime(float sample_time);
    float getSamplingTime();
    void setPlayPause(bool pause);
    bool getPlayPause();
    void initPublishers();
    void setMissionParams();
    QVariantList getPointVector();
    QVariantList getMtspVector();
    QVariantList getTspVector();
    gcs::Waypoint convertTextToWaypoint(std::string input_string);
    std::vector<std::vector<gcs::Waypoint>> splitTransectWaypoints(std::vector<gcs::Waypoint> &vec, size_t n);
    QGeoCoordinate convertWaypointToQGeoCoordinate(gcs::Waypoint &input_coord);
    void PrioSubscriberCallback(const std_msgs::UInt8::ConstPtr& msg);
    void m100flightStatusCallback(const std_msgs::UInt8::ConstPtr& msg);
    void n3flightStatusCallback(const std_msgs::UInt8::ConstPtr& msg);
    void a3flightStatusCallback(const std_msgs::UInt8::ConstPtr& msg);


    signals:
        void speedTextChanged();
        void hydrophoneRangeChanged();
        void altitudeValueChanged();
        void samplingFlagSet();
        void samplingTimeSet();
        void pauseSet();
        void mavIdSet();
        void missionTypeChanged();

    public slots:   
     void land();
     void goHome();
     void takeoff(); 
     void startMission();
     void abortMission();
     void armMav();
     void uploadWaypoints();
     void addWaypoint(double lat, double lon, float alt,  int sample, float sampleTime);
     void addGeneratedWaypoints(QString start, QString end, int num_locations);
     void generateDisks(QString center, double distance, double samplingTime);
     void publishMessage(gcs::Action &msg, int action);
     void publishMessage(gcs::Waypoint &msg);
     void tspTour(std::vector<gcs::Waypoint> &tsp_wp);
     std::vector<std::vector<gcs::Waypoint>> mtspTour(std::vector<gcs::Waypoint> &mtsp_wp, std::size_t nd);
     void processMissionWaypoints();
     void collisionAvoidance();
     void uavcaControl(gcs::Action &msg);
     void reset();
     void SpinLoop();


     



    private slots:

    private:
    ros::NodeHandle nh;
    GCS::MissionState uav_state_;
    GCS::WaypointTask waypoint_task;
    int hydrophone_range;
    int uav_speed;
    int hover_flag;
    int rth_flag;
    int land_flag;
    float target_altitude;
    int sampling_flag;
    float sampling_time;
    bool mission_pause;
    int mav_id;
    int ca_flag;
    int ca_prio_flag;
    int mission_type;

    uint8_t m100_flight_status;
    uint8_t a3_flight_status;
    uint8_t n3_flight_status;

    ros::Publisher mission_param_publisher;
    ros::Publisher waypoint_publisher;
    ros::Publisher mission_pause_publisher;
    ros::Publisher drone_action_publisher;
    ros::Publisher active_mav_publisher;
    ros::Subscriber ca_prio_subscriber;
    ros::Subscriber n3_flight_status_subscriber;
    ros::Subscriber m100_flight_status_subscriber;
    ros::Subscriber a3_flight_status_subscriber;


    std::vector<gcs::Waypoint> transect_list;
    std::vector<gcs::Waypoint> single_mission_list; // used to waypoints for single mission mode;
    QVector<QGeoCoordinate> qml_gps_points; // used to store GPS waypoints which are displayed on the UI
    QVariantList mtsp_points;
    QVariantList tsp_points;
    std::vector<int> active_mavs;
    
    GpsUtils gpsGenerator; 
    TSP tsp;
    MTSP mtsp;
    UavModel model;
    TspModel tspModel;
      

};



#endif //PLANNER_H