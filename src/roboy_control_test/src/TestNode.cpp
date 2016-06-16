//
// Created by bruh on 12/14/15.
//

#include "TestNode.h"

TestNode::TestNode() {
    qDebug() << "Advertise Service 'roboy/initialize'";

    ros::AsyncSpinner spinner(4);
    spinner.start();

    m_initializeSubscriber = m_nodeHandle.subscribe("/roboy/initialize", 1000, &TestNode::callbackInitialize, this);
    m_recordSteeringSubscriber = m_nodeHandle.subscribe("/roboy/steer_record", 1000, &TestNode::callbackSteerRecord, this);

    // Advertise Record Service
//    boost::function<bool (common_utilities::Record::Request &, common_utilities::Record::Response &)> function;
//    function = &TestNode::callbackRecord;
//    struct callback_record {
//        bool operator()(common_utilities::Record::Request & req, common_utilities::Record::Response & res) {
//                return true;
//            }
//    };
//    function = callback_record();

//    ros::CallbackQueue * queue = new ros::CallbackQueue();
//    m_nodeHandle.setCallbackQueue(queue);

//    ros::AdvertiseServiceOptions options = ros::AdvertiseServiceOptions::create("/roboy/steer_record", &TestNode::callbackRecord, ros::VoidConstPtr(), queue);
//    m_recordServer = m_nodeHandle.advertiseService(options);

//    m_recordServer = m_nodeHandle.advertiseService("/roboy/record", &TestNode::callbackRecord, this);
}

TestNode::~TestNode() {
    for(MotorController & controller : m_listControllers) {
        delete controller.process;
    }
}

void TestNode::callbackInitialize(const common_utilities::Initialize & msg){
    qDebug() << "Process 'roboy/initialize' request";

    MotorController controller;

    int i = 0;
    for(int i = 0; i < msg.controllers.size(); i++) {
        QString nodeName;
        nodeName.sprintf("controller_stub%i", msg.controllers[i].id);

        QString serviceName;
        serviceName.sprintf("/roboy/trajectory_motor%i", msg.controllers[i].id);

        controller.id = msg.controllers[i].id;
        controller.controlMode = (ControlMode) msg.controllers[i].controlmode;

        if(startNode(controller.id, nodeName, serviceName, controller)){
            controller.state = ControllerState::INITIALIZED;
        } else {
            controller.state = ControllerState::INITIALIZE_ERROR;
        }

        m_listControllers.append(controller);

        qDebug() << "\t- Update Controller: " << controller.toString();
    }
}

//bool TestNode::callbackRecord(common_utilities::Record::Request & req, common_utilities::Record::Response & res) {
//    m_bInterrupted = false;
//
//    qCritical() << "Start Recording";
//
//    ros::Duration duration(20);
//
//    common_utilities::Trajectory trajectory1;
//    common_utilities::Trajectory trajectory2;
//
//    trajectory1.id = 1;
//    trajectory2.id = 2;
//    trajectory1.samplerate = 110;
//    trajectory2.samplerate = 110;
//
//    res.trajectories.push_back(trajectory1);
//    res.trajectories.push_back(trajectory2);
//
//    double v1 = 0.0;
//    double v2 = 100.0;
//
//    for(int i = 0; i < 20; i++) {
//        res.trajectories[0].waypoints.push_back(v1);
//        res.trajectories[1].waypoints.push_back(v2);
//        v1 += 1;
//        v2 += 2;
//    }
//
//    duration.sleep();
//
////    while(!m_bInterrupted) {
////        qDebug() << "Wait.";
////        duration.sleep();
////        res.trajectories[0].waypoints.push_back(v1);
////        res.trajectories[1].waypoints.push_back(v2);
////        v1 += 1;
////        v2 += 2;
////    }
//
////    qDebug() << "Return " << res.trajectories.at(0).waypoints.size() << " Waypoints";
//
//    return true;
//}

void TestNode::callbackSteerRecord(const common_utilities::Steer & msg) {
    qWarning() << "Received Record Steering Message";
    m_mutexCV.lock();
    m_bInterrupted = true;
    m_mutexCV.unlock();
}

bool TestNode::startNode(qint32 id, QString nodeName, QString serviceName, MotorController & controller) {
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