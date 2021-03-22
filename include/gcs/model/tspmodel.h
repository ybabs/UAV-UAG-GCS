#ifndef TSPMODEL_H
#define TSPMODEL_H

#include <QAbstractListModel>
#include <QTimer>
#include <QGeoCoordinate>
#include <QGeoPath>
#include <QVariantList>
#include <ros/ros.h>

class TspModel :  public QAbstractListModel
{
    Q_OBJECT
    Q_PROPERTY(QVariantList path READ path NOTIFY pathChanged)
    Q_PROPERTY(QGeoPath geopath READ geoPath WRITE setGeoPath NOTIFY geopathChanged)
  
public: 
  enum MarkerRoles { 
    positionRole = Qt::UserRole + 1 
    };

  
  TspModel(QAbstractItemModel *parent = 0);
  Q_INVOKABLE void addPosition(const QGeoCoordinate &coordinate);
  int rowCount(const QModelIndex &parent = QModelIndex() ) const ;
 // bool removeRows(int row, int count, const QModelIndex &parent = QModelIndex()) override;
 // bool removeRow(int row, const QModelIndex &parent = QModelIndex());
  QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const  ;
  QVariantList path() const;
  QGeoPath geoPath() const;
  void setGeoPath(const QGeoPath &geoPath);
  Q_INVOKABLE void test();

  protected:
   QHash<int, QByteArray> roleNames() const ;

  private:
    QVariantList m_coordinates;
    QGeoPath m_geopath;
    QTimer m_timer;

  signals:
    void pathChanged();
    void geopathChanged();

};


#endif