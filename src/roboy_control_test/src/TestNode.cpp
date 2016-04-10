//
// Created by bruh on 12/14/15.
//

#include "TestNode.h"

TestNode::TestNode() {
    qDebug() << "Advertise Service 'roboy/initialize'";

    ros::AsyncSpinner spinner(4);
    spinner.start();

    m_initializeServer = m_nodeHandle.advertiseService("/roboy/initialize", &TestNode::callbackInitialize, this);
    m_recordServer = m_nodeHandle.advertiseService("/roboy/record", &TestNode::callbackRecord, this);
    m_recordSteeringSubscriber = m_nodeHandle.subscribe("/roboy/steer_record", 1000, &TestNode::callbackSteerRecord, this);
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
            controller.state = ControllerState::INITIALIZED;
        } else {
            controller.state = ControllerState::INITIALIZE_ERROR;
        }

        m_listControllers.append(controller);

        responseMessage.id = req.idList[i];
        responseMessage.state = controller.state;
        res.states.push_back(responseMessage);

        qDebug() << "\t- Update Controller: " << controller.toString();
    }

    return true;
}

bool TestNode::callbackRecord(common_utilities::Record::Request & req, common_utilities::Record::Response & res) {
    m_bInterrupted = false;

    qCritical() << "Start Recording";

    ros::Duration duration(0, 500000000);

//    common_utilities::Trajectory trajectory1;
//    common_utilities::Trajectory trajectory2;
//
//    trajectory1.samplerate = 100;
//    trajectory2.samplerate = 100;
//
//    res.trajectories.push_back(trajectory1);
//    res.trajectories.push_back(trajectory2);
//
//    double v1 = 0.0;
//    double v2 = 100.0;

    while(!m_bInterrupted) {
//        qDebug() << "Wait.";
        duration.sleep();
//        res.trajectories[0].waypoints.push_back(v1);
//        res.trajectories[1].waypoints.push_back(v2);
//        v1 += 1;
//        v2 += 2;
    }

//    qDebug() << "Return " << res.trajectories.at(0).waypoints.size() << " Waypoints";

    return true;
}

void TestNode::callbackSteerRecord(const common_utilities::Steer & msg) {
    qWarning() << "Received Record Steering Message";
    m_mutexCV.lock();
    m_bInterrupted = true;
    m_mutexCV.unlock();
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