#include "qml_gui/model/uavmodel.h"
#include <random>


UavModel::UavModel(QObject *parent):
          QObject(parent),
          m_uavModel(new QStandardItemModel(this))
{
    connected_clients = 4;
     QHash<int, QByteArray> roles;
     roles = roleNames();
     m_uavModel->setItemRoleNames(roles);
    for(int i = 0; i < connected_clients; i++)
    {
        QStandardItem *item = new QStandardItem;
        item->setData(QVariant::fromValue(QGeoCoordinate()), PositionRole);
        item->setData(QVariant::fromValue(battery_capacity), BatteryRole);
        m_uavModel->appendRow(item);
    }

}


QObject *UavModel::uavModel() const
{
    return m_uavModel;
}


void UavModel::batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
  double battery = msg->voltage;
}


void UavModel::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
 
  uav_position.setLatitude(msg->latitude) ;
  uav_position.setLongitude( msg->longitude);
  uav_position.setAltitude(msg->altitude);
}

void UavModel::updateModelData()
{
    std::mt19937 rng;
    rng.seed(std::random_device()());
    std::normal_distribution<> dist(-0.001, +0.001);
     std::normal_distribution<> val(-4.5, +10.5);

    connected_clients = 3; // get this data from the number of Uavs connected to the network

    if(QStandardItem *item = m_uavModel->item(0))
    {
        //QGeoCoordinate uav_pos;
        // uav_position.setLatitude(52.75591 + dist(rng));
        // uav_position.setLongitude(-1.246310 + dist(rng));
        uav_position.setLatitude(52.75591 );
        uav_position.setLongitude(-1.246310 );
        uav_position.setAltitude(5);
        battery_capacity = 45.85 ;
        item->setData(QVariant::fromValue(uav_position), PositionRole);
        item->setData(QVariant::fromValue(battery_capacity), BatteryRole);
    }
}

QHash<int, QByteArray> UavModel::roleNames() const
{
    QHash<int, QByteArray> roles;
    roles[PositionRole] = "position";
    roles[BatteryRole] = "battery";

    return roles;

}