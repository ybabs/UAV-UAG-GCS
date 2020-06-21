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
#include <sstream>
#include <iostream>




class GCS: public QObject
{
    Q_OBJECT

    Q_PROPERTY(int getDroneSpeed READ getDroneSpeed WRITE setDroneSpeed NOTIFY speedSliderChanged )
    Q_PROPERTY(int getHoverFlag READ getHoverFlag WRITE setHoverFlag NOTIFY hoverFlagSet)
    Q_PROPERTY(int getRthFlag READ getRthFlag WRITE setRthFlag NOTIFY rthFlagSet)
    Q_PROPERTY(int getLandFlag READ getLandFlag WRITE setLandFlag NOTIFY landFlagSet)
    Q_PROPERTY(float getAltitude READ getAltitude WRITE setAltitude NOTIFY altitudeValueChanged )
    Q_PROPERTY(float getSamplingTime READ getSamplingTime WRITE setSamplingTime NOTIFY samplingTimeSet)
    Q_PROPERTY(int getSamplingFlag READ getSamplingFlag WRITE setSamplingFlag NOTIFY samplingFlagSet )
    Q_PROPERTY(int getPlayPause READ getPlayPause WRITE setPlayPause NOTIFY pauseSet )
    Q_PROPERTY(int getMavId READ getMavId WRITE setMavId NOTIFY mavIdSet)
    Q_PROPERTY(QVariantList trackpoints READ getPointVector)
    
       

   
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

    enum {
        NameRole,
        PositionRole = Qt::UserRole + 1,
        BatteryRole
        
    };

    enum WaypointTask
    {
        LAND
    };

    int getMavId();
    void setMavId(int id);
    void setDroneSpeed(int speed); //
    int getDroneSpeed(); //
    void setHoverFlag(int flag); //
    int getHoverFlag(); //
    void setRthFlag(int flag); //
    int getRthFlag(); //
    void setLandFlag(int flag); //
    int getLandFlag(); //
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
    void startTimer();
    QVariantList getPointVector();
    sensor_msgs::NavSatFix convertTextToNavSatFix(std::string input_string);

    signals:
        void speedSliderChanged();
        void hoverFlagSet();
        void rthFlagSet();
        void landFlagSet();
        void altitudeValueChanged();
        void samplingFlagSet();
        void samplingTimeSet();
        void pauseSet();
        void mavIdSet();

    public slots:   
     void land();
     void goHome();
     void takeoff(); 
     void startMission();
     void abortMission();
     void armMav();
     void addWaypoint(double lat, double lon, float alt,  int sample, float sampleTime);
     void addGeneratedWaypoints(QString start, QString end, int num_locations);
    



    private slots:

    private:
    ros::NodeHandle nh;
    GCS::MissionState uav_state_;
    GCS::WaypointTask waypoint_task;
    int uav_speed;
    int hover_flag;
    int rth_flag;
    int land_flag;
    float target_altitude;
    int sampling_flag;
    float sampling_time;
    bool mission_pause;
    int mav_id;

    ros::Publisher mission_param_publisher;
    ros::Publisher waypoint_publisher;
    ros::Publisher mission_pause_publisher;
    ros::Publisher drone_action_publisher;
    ros::Publisher active_mav_publisher;

    std::vector<gcs::Waypoint> transect_list;
    QVector<QGeoCoordinate> qml_gps_points;
    std::vector<int> active_mavs;
    
    GpsUtils gpsGenerator;


   

    
      

};



#endif //PLANNER_H