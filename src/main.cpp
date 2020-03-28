#include <ros/ros.h>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "qml_gui/planner/planner.h"

int main(int argc, char *argv[])
{
     ros::init(argc, argv, "planner");
    QGuiApplication app(argc, argv);
     
     QQmlApplicationEngine engine;
     QQmlContext* context = engine.rootContext();
     GCS gcs;
     context->setContextProperty("planner", &gcs);    
     engine.load(QUrl(QStringLiteral("qrc:/planner.qml")));
    
    
    return app.exec();
}