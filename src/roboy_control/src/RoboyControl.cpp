//
// Created by bruh on 17.11.15.
//

#include "ros/ros.h"

#include <QApplication>
#include <QFile>
#include <QDebug>
#include <QDateTime>

#include "view/MainWindow.h"
#include "model/RoboyBehaviorXmlParser.h"
#include "model/XmlModelService.h"

int main(int argc, char ** argv) {

    QApplication app(argc, argv);
    MainWindow window;
    window.show();

    XmlModelService xmlModelService;
    IModelService & modelService = xmlModelService;

    RoboyBehavior newBehavior;
    newBehavior.m_metadata.m_sBehaviorName = "GreetBehavior";
    newBehavior.m_metadata.m_ulBehaviorId = 2;
    QList<RoboyWaypoint> wps;
    RoboyWaypoint wp;
    for (int i = 0; i < 5; i++) {
        wps.clear();
        for (int j = 0; j < 20; j++) {
            wp.m_ulId = j;
            wp.m_ulTimestamp = QDateTime::currentMSecsSinceEpoch();
            wp.m_ulPosition =  j * i + (i + j);
            wps.append(wp);
        }
        newBehavior.m_mapMotorWaypoints.insert(i, wps);
    }

    modelService.persistNewRoboyBehavior(newBehavior);

    RoboyBehaviorMetadata metadata;
    metadata.m_sBehaviorName = "GreetBehavior";
    metadata.m_ulBehaviorId = 2;

    RoboyBehavior behavior = modelService.getBehavior(metadata);

    LOG << behavior.toString();

    return app.exec();
}
