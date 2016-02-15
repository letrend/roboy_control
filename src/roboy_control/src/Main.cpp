//
// Created by bruh on 17.11.15.
//

#include <QApplication>
#include "ros/ros.h"

#include "IModelService.h"
#include "RoboyController.h"
#include "XmlModelService.h"

void initializeDatabase();

void myMessageOutput(QtMsgType type,  const QMessageLogContext &context, const QString &msg) {
    QByteArray localMsg = msg.toLocal8Bit();
    switch(type) {
        case QtDebugMsg:
            fprintf(stderr, "\033[0m%s\n", localMsg.constData());
            break;
        case QtWarningMsg:
            fprintf(stderr, "\033[33m%s\n", localMsg.constData());
            break;
        case QtCriticalMsg:
            fprintf(stderr, "\033[32m%s\n", localMsg.constData());
            break;
        case QtFatalMsg:
            fprintf(stderr, "\033[31m%s\n", localMsg.constData());
            break;
    }
}

int main(int argc, char ** argv) {
    qInstallMessageHandler(myMessageOutput);

    ros::init(argc, argv, "roboy_control");
    ros::NodeHandle n;

    QApplication app(argc, argv);

    RoboyController controller;
    controller.start();

    return app.exec();
}

void initializeDatabase() {
    RoboyBehaviorMetadata metadata;
    metadata.m_sBehaviorName = "TestBehavior1";
    metadata.m_ulBehaviorId = 1;
    RoboyBehavior behavior;
    behavior.m_metadata = metadata;
    Trajectory trajectory;
    trajectory.m_controlMode = ControlMode::POSITION_CONTROL;
    trajectory.m_sampleRate = 100;
    RoboyWaypoint wp;
    for(int i = 0; i < 20; i++) {
        wp.m_ulId = i;
        wp.m_ulValue = i * 2;
        trajectory.m_listWaypoints.append(wp);
    }
    behavior.m_mapMotorTrajectory.insert(1, trajectory);

    IModelService * modelService = new XmlModelService();
    modelService->createRoboyBehavior(behavior);

    metadata.m_sBehaviorName = "TestBehavior2";
    metadata.m_ulBehaviorId = 2;

    behavior.m_metadata = metadata;
    trajectory.m_listWaypoints.clear();
    behavior.m_mapMotorTrajectory.clear();
    for (int i = 0; i < 20; i++) {
        wp.m_ulId = i;
        wp.m_ulValue = pow(i, 2) + i;
        trajectory.m_listWaypoints.append(wp);
    }
    behavior.m_mapMotorTrajectory.insert(1, trajectory);

    modelService->createRoboyBehavior(behavior);
}
