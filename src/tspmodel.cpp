#include "gcs/model/tspmodel.h"

TspModel::TspModel(QAbstractItemModel *parent):
    QAbstractListModel(parent)
{

   // connect(this, &QAbstractListModel::rowsInserted, this, &TspModel::pathChanged);
   // connect(this, &QAbstractListModel::rowsRemoved, this, &TspModel::pathChanged);
    connect(this, &QAbstractListModel::dataChanged, this, &TspModel::pathChanged);
   // connect(this, &QAbstractListModel::modelReset, this, &TspModel::pathChanged);
   // connect(this, &QAbstractListModel::rowsMoved, this, &TspModel::pathChanged);

}

Q_INVOKABLE void TspModel::addPosition(const QGeoCoordinate &coordinate) {
    beginInsertRows(QModelIndex(), rowCount(), rowCount());
    //ROS_INFO("Added Runway to list LAT: %f, ", coordinate.latitude());
    m_coordinates.append(QVariant::fromValue(coordinate));
     emit pathChanged();
    endInsertRows();
  
    
  }

 int TspModel::rowCount(const QModelIndex &parent) const  
 {
   Q_UNUSED(parent)
    return m_coordinates.count();
}

QVariant TspModel::data(const QModelIndex &index, int role) const 
{
    if (index.row() < 0 || index.row() >= m_coordinates.count())
    {
        return QVariant();
    }
    if (role == positionRole)
    {
        return QVariant::fromValue(m_coordinates[index.row()]);   
    }
     return QVariant(); 
  }

  QHash<int, QByteArray> TspModel::roleNames() const
  {
    QHash<int, QByteArray> roles;
    roles[positionRole] = "position";
    return roles;
  }


  QVariantList TspModel::path() const
  {
    return m_coordinates;
  }

QGeoPath TspModel::geoPath() const
{
  return m_geopath;

}

void TspModel::setGeoPath(const QGeoPath &geoPath)
{
  if(geoPath == m_geopath)
  { 
      return;
  }

  m_geopath = geoPath;
  emit geopathChanged();
}

Q_INVOKABLE void TspModel::test()
{
      m_geopath.addCoordinate(QGeoCoordinate(56.006355, 92.860984));
    m_geopath.addCoordinate(QGeoCoordinate(56.1, 93));
    m_geopath.addCoordinate(QGeoCoordinate(56.1, 92.777));

         QTimer *timer = new QTimer(this);

       QObject::connect(timer, &QTimer::timeout, [this]() {
      m_geopath.translate(0.001, -0.01);
      emit geopathChanged();
    });
    timer->start(1000);
    

}