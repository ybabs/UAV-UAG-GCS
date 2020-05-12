#include "qml_gui/model/uavmodel.h"
#include <random>


UavModel::UavModel(QObject *parent):
          QObject(parent),
          m_uavModel(new QStandardItemModel(this)),
          uav_positions(4, uav_position)
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

    
    batteries.reserve(4);

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

    connected_clients = 4; // get this data from the number of Uavs connected to the network
    batteries[0]= 50.2;
    batteries[1] = 54.6;
    batteries[2] = 30;
    batteries[3] = 50;

    QGeoCoordinate uav_pos1;
    QGeoCoordinate uav_pos2;
    QGeoCoordinate uav_pos3;
    QGeoCoordinate uav_pos4;

    uav_pos1.setLatitude(52.75591 );
    uav_pos1.setLongitude(-1.246310 );
    uav_pos1.setAltitude(5);
    
    uav_pos2.setLatitude(52.755964 );
    uav_pos2.setLongitude( -1.246648 );
    uav_pos2.setAltitude(5);

    uav_pos3.setLatitude(52.756151 );
    uav_pos3.setLongitude(-1.248142 );
    uav_pos3.setAltitude(5);
        
    uav_pos4.setLatitude(52.755872 );
    uav_pos4.setLongitude(-1.246683 );
    uav_pos4.setAltitude(5);

    uav_positions[0] = uav_pos1;
    uav_positions[1] = uav_pos2;
    uav_positions[2] = uav_pos3;
    uav_positions[3] = uav_pos4;

     
    for(int i = 0; i < connected_clients; i++)
    {
        if(QStandardItem *item = m_uavModel->item(i))
        {
            item->setData(QVariant::fromValue(uav_positions[i]), PositionRole);
            item->setData(QVariant::fromValue(batteries[i]), BatteryRole);
        }

       

     }
}

QHash<int, QByteArray> UavModel::roleNames() const
{
    QHash<int, QByteArray> roles;
    roles[PositionRole] = "position";
    roles[BatteryRole] = "battery";

    return roles;

}