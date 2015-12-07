//
// Created by bruh on 17.11.15.
//

#include <QApplication>
#include "controller/RoboyController.h"
#include "transceiver/ROSMessageTransceiverService.h"
#include "ros/ros.h"

int main(int argc, char ** argv) {
    ros::init(argc, argv, "roboycontrol");

    QApplication app(argc, argv);

    RoboyController controller;
    controller.start();

    RoboyBehavior newBehavior;
    newBehavior.m_metadata.m_sBehaviorName = "GreetBehavior";
    newBehavior.m_metadata.m_ulBehaviorId = 2;

    ROSMessageTransceiverService t;
    t.sendRoboyBehavior(newBehavior);

    return app.exec();
}

int testCode() {
/*    RoboyControlConfiguration& config = RoboyControlConfiguration::instance();

    DBG << RoboyControlConfiguration::instance().getModelConfig("databasePath");
    DBG << RoboyControlConfiguration::instance().getModelConfig("databaseType");
    if (config.getModelConfig("asdf") == QString::null) {
        DBG << "Invalid Value";
    }

    XmlModelService xmlModelService;
    IModelService & modelService = xmlModelService;

    MainWindow window(&modelService);
    window.show();

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

    QList <RoboyBehaviorMetadata> behaviorList = modelService.getBehaviorList();
    for (RoboyBehaviorMetadata behavior : behaviorList) {
        LOG << behavior.m_sBehaviorName << " ID: " << behavior.m_ulBehaviorId;
    }
*/
}
