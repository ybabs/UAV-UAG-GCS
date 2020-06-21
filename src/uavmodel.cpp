#include "gcs/model/uavmodel.h"
#include <random>



UavModel::UavModel(QObject *parent):
          QObject(parent),
          m_uavModel(new QStandardItemModel(this)),
          uav_positions(4, QGeoCoordinate()),
          batteries(4,0),
          uav_id(0)
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
        item->setData(QVariant::fromValue(5), NameRole);
        m_uavModel->appendRow(item);
    }

    ROS_INFO("Constructor UAV Model");

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
 

}

void UavModel::updateModelData()
{
   // connected_clients = 4; // get this data from the number of Uavs connected to the network
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
    
    uav_pos2.setLatitude(52.755934);
    uav_pos2.setLongitude(-1.247107);
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

    //ROS_INFO("This is running");
     
    for(int i = 0; i < connected_clients; i++)
    {
        QString uav_Text = "MAV " + QString::number(i + 1);
        
        if(QStandardItem *item = m_uavModel->item(i))
        {
            
            item->setData(QVariant::fromValue(uav_positions[i]), PositionRole);
            item->setData(QVariant::fromValue(batteries[i]), BatteryRole);
            item->setData(QVariant::fromValue(uav_Text), NameRole);
            
        }      

     }
}

QHash<int, QByteArray> UavModel::roleNames() const
{
    QHash<int, QByteArray> roles;
    roles[PositionRole] = "position";
    roles[BatteryRole] = "battery";
    roles[NameRole] = "identifier";

    return roles;

}