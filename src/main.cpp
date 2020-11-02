#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QtQuick/QQuickItem>
#include <QTimer>
#include "gcs/planner/planner.h"
#include "gcs/model/uavmodel.h"
#include "gcs/model/tspmodel.h"



int main(int argc, char* argv[])
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    ros::init(argc, argv, "planner");
    QGuiApplication app(argc, argv);
     
     QQmlApplicationEngine engine;
     QQmlContext* context = engine.rootContext();
     GCS gcs;
     UavModel model;
     TspModel tspModel;

     QGeoCoordinate coord1;
     QGeoCoordinate coord2;
     QGeoCoordinate coord3;

  

     context->setContextProperty("planner", &gcs);
     context->setContextProperty("mav", &model);
     context->setContextProperty("tspPath", &tspModel);

    
     const QUrl url(QStringLiteral("qrc:/main.qml"));

         coord1.setLatitude(53.186166);
     coord1.setLongitude(-1.926956);
          coord2.setLatitude(52.545485);
     coord2.setLongitude(-1.926956);
          coord3.setLatitude(53.684997);
     coord3.setLongitude(-1.974328);
     tspModel.addMarker(coord1);
     tspModel.addMarker(coord2);
     tspModel.addMarker(coord3);
     QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) 
    {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);
    engine.load(url);

    QObject *item = engine.rootObjects().first();
    Q_ASSERT(item);
    QMetaObject::invokeMethod(item, "initializeProviders",
                                Qt::QueuedConnection);

    QTimer timer;
    timer.setInterval(60);
    QObject::connect(&timer, &QTimer::timeout, &model, &UavModel::updateModelData);
    timer.start();


    return app.exec();
}