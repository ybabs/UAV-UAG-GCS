#include "gcs/model/uavmodel.h"
#include <random>



UavModel::UavModel(QObject *parent):
          QObject(parent),
          m_uavModel(new QStandardItemModel(this)),
          uav_positions(4, QGeoCoordinate()),
          batteries(4,0),
          uav_id(0)
{

     // Spin up ros node handle
     QTimer * ros_timer = new QTimer(this);
     connect(ros_timer, SIGNAL(timeout()), this, SLOT(SpinLoop()));
     ros_timer->start(0);

    m100_battery_subscriber = nh.subscribe("/a3/battery_state", 10, &UavModel::A3batteryStateCallback, this);
    m100_gps_subscriber = nh.subscribe("/a3/gps_position", 10, &UavModel::A3gpsCallback, this);
    n3_battery_subscriber = nh.subscribe("n3/battery_state", 10, &UavModel::N3batteryStateCallback, this);
    n3_gps_subscriber = nh.subscribe("n3/gps_position", 10, &UavModel::N3gpsCallback, this);


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

   


}


void UavModel::SpinLoop()
{
    ros::spinOnce();
}


QObject *UavModel::uavModel() const
{
    return m_uavModel;
}

void UavModel::A3batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
  double battery = msg->voltage;
  double battery_percent = battery * 100;
  //ROS_INFO("Battery: %f", battery_percent );
  // based on a 4S LiHV battery. 
  // Low votage per cell = 3.6V  3.6  * 4 * 100 = 1440
  //Fully charged  = 4.35V. 4.35 * 4 * 100 = 2520; 
  double value = ((battery_percent - 1440) / (1740 - 1440)) * 100; 
  value = (int) (100 * value) / 100.0;
  batteries[0] = value;
}

void UavModel::N3batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
  double battery = msg->voltage;
  double battery_percent = battery * 100;
  //ROS_INFO("Battery: %f", battery_percent );
  // based on a 4S LiHV battery. 
  // Low votage per cell = 3.6V  3.6  * 4 * 100 = 1440
  //Fully charged  = 4.35V. 4.35 * 4 * 100 = 2520; 
  double value = ((battery_percent - 1440) / (1740 - 1440)) * 100; 
  value = (int) (100 * value) / 100.0;
  batteries[1] = value;
  
}


void UavModel::N3gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    QGeoCoordinate uav_pos;
    uav_pos.setLatitude(msg->latitude);
    uav_pos.setLongitude(msg->longitude);
    uav_pos.setAltitude(msg->altitude);

    uav_positions[1] = uav_pos;
    
}

void UavModel::A3gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    QGeoCoordinate uav_pos;
    uav_pos.setLatitude(msg->latitude);
    uav_pos.setLongitude(msg->longitude);
    uav_pos.setAltitude(msg->altitude);

    uav_positions[0] = uav_pos;
 

}

std::vector<QGeoCoordinate> UavModel::getUavPositions()
{
    QGeoCoordinate uav_pos;
    uav_pos.setLatitude(52.770104);
    uav_pos.setLongitude(-1.231820);

    QGeoCoordinate uav_pos2;
    uav_pos2.setLatitude(52.770104);
    uav_pos2.setLongitude(-1.231820);

    QGeoCoordinate uav_pos3;
    uav_pos3.setLatitude(52.770104);
    uav_pos3.setLongitude(-1.231820);

    QGeoCoordinate uav_pos4;
    uav_pos4.setLatitude(52.770104);
    uav_pos4.setLongitude(-1.231820);

    uav_positions[0] = uav_pos;
    uav_positions[1] = uav_pos2;
    uav_positions[2] = uav_pos3;
    uav_positions[3] = uav_pos4;
    
    return uav_positions;
}

void UavModel::updateModelData()
{
     
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