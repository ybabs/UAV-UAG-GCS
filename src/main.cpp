#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QTimer>
#include "qml_gui/model/uavmodel.h"

int main(int argc, char* argv[])
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    ros::init(argc, argv, "planner");
    QGuiApplication app(argc, argv);
     
     QQmlApplicationEngine engine;
     QQmlContext* context = engine.rootContext();
     UavModel model;

     context->setContextProperty("planner", &model);
     const QUrl url(QStringLiteral("qrc:/planner.qml"));
     QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) 
    {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);
    engine.load(url);

    QTimer timer;
    timer.setInterval(20);
    QObject::connect(&timer, &QTimer::timeout, &model, &UavModel::updateModelData);
    timer.start();

    return app.exec();
}