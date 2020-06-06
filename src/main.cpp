#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QtQuick/QQuickItem>
#include <QTimer>
#include "gcs/planner/planner.h"
#include "gcs/model/uavmodel.h"



int main(int argc, char* argv[])
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    ros::init(argc, argv, "planner");
    QGuiApplication app(argc, argv);
     
     QQmlApplicationEngine engine;
     QQmlContext* context = engine.rootContext();
     GCS gcs;
     UavModel model;
     context->setContextProperty("planner", &gcs);
     context->setContextProperty("mav", &model);
     const QUrl url(QStringLiteral("qrc:/main.qml"));
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