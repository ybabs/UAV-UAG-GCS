#ifndef UAVMODEL_H
#define UAVMODEL_H

#include <QObject>
#include <QAbstractListModel>
#include <QStandardItemModel>
#include "QString"
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>
#include <QGeoCoordinate>
#include <QVariantList>




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
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
    QHash<int, QByteArray> roleNames() const ;
    
public Q_SLOTS: // slots
    void updateModelData();
private:
    QStandardItemModel* m_uavModel;
    int uav_id;
    int connected_clients;
    double battery_capacity;
    std::vector<QGeoCoordinate> uav_positions;
    std::vector<double> batteries;
    ros::Subscriber uav_battery_subscriber;
    ros::Subscriber uav_gps_subscriber;

};




#endif