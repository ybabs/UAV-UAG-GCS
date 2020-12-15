#ifndef UAVMODEL_H
#define UAVMODEL_H

#include <QObject>
#include <QAbstractListModel>
#include <QStandardItemModel>
#include "QString"
#include <QTimer>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>
#include <QGeoCoordinate>
#include <QVariantList>
// #include "gcs/planner/planner.h"





class UavModel : public QObject
{

    Q_OBJECT

    Q_PROPERTY(QObject* uavModel READ uavModel CONSTANT)

public:
    enum {
        NameRole,
        PositionRole = Qt::UserRole + 1,
        BatteryRole
        
    };
    UavModel(QObject *parent = nullptr);
    QObject *uavModel() const;

    // Callbacks
    void A3gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void A3batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
    void N3gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void N3batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
    
    //void uav4gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    //void uav4batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
   // void uav3gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
   // void uav3batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
    QHash<int, QByteArray> roleNames() const ;
    
    
public Q_SLOTS: // slots
    void updateModelData();
    void SpinLoop();
    std::vector<QGeoCoordinate> getUavPositions();
private:
    ros::NodeHandle nh;
    QStandardItemModel* m_uavModel;
    int uav_id;
    int connected_clients;
    double battery_capacity;
    std::vector<QGeoCoordinate> uav_positions;
    std::vector<double> batteries;
    ros::Subscriber m100_battery_subscriber;
    ros::Subscriber m100_gps_subscriber;
    ros::Subscriber n3_battery_subscriber;
    ros::Subscriber n3_gps_subscriber;
    //ros::Subscriber uav4_battery_subscriber;
   // ros::Subscriber uav4_gps_subscriber;
    // ros::Subscriber uav3_battery_subscriber;
    //ros::Subscriber uav3_gps_subscriber;

};




#endif