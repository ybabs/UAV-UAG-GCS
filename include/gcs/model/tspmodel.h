#ifndef TSPMODEL_H
#define TSPMODEL_H

#include <QAbstractListModel>
#include <QGeoCoordinate>
#include <QVariantList>
#include <ros/ros.h>

class TspModel :  public QAbstractListModel
{
    Q_OBJECT
  
public:
     
  using QAbstractListModel::QAbstractListModel;
  enum MarkerRoles { positionRole = Qt::UserRole + 1 };

Q_INVOKABLE void addMarker(const QGeoCoordinate &coordinate) {
    beginInsertRows(QModelIndex(), rowCount(), rowCount());
    //ROS_INFO("Added Runway to list LAT: %f, ", coordinate.latitude());
    m_coordinates.append(coordinate);
    endInsertRows();
  }

  int rowCount(const QModelIndex &parent = QModelIndex()) const override {
    Q_UNUSED(parent)
    return m_coordinates.count();
  }

  QVariant data(const QModelIndex &index,
                int role = Qt::DisplayRole) const override {
    if (index.row() < 0 || index.row() >= m_coordinates.count())
      return QVariant();
      const QGeoCoordinate &coord = m_coordinates[index.row()];
    if (role == TspModel::positionRole)
    {
      QVariantList path;

      const auto &pathList = m_coordinates;
      for(const QGeoCoordinate &point: pathList )
      {
        path << QVariant::fromValue(point);
      }
      return path;
      
    }
     return QVariant(); 
  }

public Q_SLOTS:
  //void updateModelData();
  void spinLoop()
  {
    ros::spinOnce();
  }

protected:
  QHash<int, QByteArray> roleNames() const {
    QHash<int, QByteArray> roles;
    roles[positionRole] = "position";
    return roles;
  }

private:
  QList<QGeoCoordinate> m_coordinates;
};


#endif