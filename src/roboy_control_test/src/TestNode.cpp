//
// Created by bruh on 12/14/15.
//

#include "TestNode.h"

TestNode::TestNode() {
    qDebug() << "Advertise Service 'roboy/initialize'";
    ros::ServiceServer initializeServer = m_nodeHandle.advertiseService("roboy/initialize", &TestNode::callbackInitialize, this);

    ros::spin();
}

TestNode::~TestNode() {
    for(ROSController & controller : m_listControllers) {
        delete controller.process;
    }
}



bool TestNode::callbackInitialize(common_utilities::Initialize::Request & req, common_utilities::Initialize::Response & res){
    qDebug() << "Process 'roboy/initialize' request";

    ROSController controller;
    common_utilities::ControllerState responseMessage;
    int i = 0;
    for(int i = 0; i < req.idList.size(); i++) {
        QString nodeName;
        nodeName.sprintf("controller_stub%i", req.idList[i]);

        QString serviceName;
        serviceName.sprintf("roboy/trajectory_motor%i", req.idList[i]);

        controller.id = req.idList[i];
        controller.controlMode = (ControlMode) req.controlmode[i];

        if(startNode(controller.id, nodeName, serviceName, controller)){
            controller.state = STATUS::INITIALIZED;
        } else {
            controller.state = STATUS::INITIALIZE_ERROR;
        }

        m_listControllers.append(controller);

        responseMessage.id = req.idList[i];
        responseMessage.state = controller.state;
        res.states.push_back(responseMessage);

        qDebug() << "\t- Update Controller: " << controller.toString();
    }

    return true;
}

void TestNode::callbackSteering(const common_utilities::Steer & msg){
    qDebug() << "Heard steering message: " << msg.steeringCommand;
}

bool TestNode::startNode(qint32 id, QString nodeName, QString serviceName, ROSController & controller) {
    QString roboyControlHome = QProcessEnvironment::systemEnvironment().value("ROBOY_TEST_HOME");
    if(roboyControlHome.isEmpty()) {
        qDebug() << "Set environment variable 'ROBOY_TEST_HOME' to etc folder.";
        return false;
    }

    QString startCommand;
    startCommand.sprintf("%s/controller_stub %i %s %s", roboyControlHome.toStdString().data(), (int)id, nodeName.toStdString().data(), serviceName.toStdString().data());
    qDebug() << "Run Node start script: " << startCommand;

    QObject * parent;
    QString program;
    program.sprintf("%s/controller_stub", roboyControlHome.toStdString().data());
    QStringList arguments;
    arguments << QString::number(id).toStdString().data() << nodeName.toStdString().data() << serviceName.toStdString().data();

    qDebug() << "- Program: " << program;
    qDebug() << "- Arguments: " << arguments;

    QProcess * proc = new QProcess();
    proc->start(program, arguments);
    controller.process = proc;
    if(proc->waitForStarted()) {
        qDebug() << "Started Pro cess: [name:" << nodeName << "]";
        return true;
    }

    qDebug() << "Failed to start Process: [name:" << nodeName << "]";
    return false;
}