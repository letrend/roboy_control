#include "ROSMasterCommunication.h"

ROSMasterCommunication::ROSMasterCommunication() {
    m_initializeClient = m_nodeHandle.serviceClient<common_utilities::Initialize>("/roboy/initialize");
    m_steerPublisher = m_nodeHandle.advertise<common_utilities::Steer>("/roboy/steer", 1000);

//    m_switchController = m_nodeHandle.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
//    m_unloadController = m_nodeHandle.serviceClient<controller_manager_msgs::UnloadController>("/controller_manager/unload_controller");

    m_recordClient = m_nodeHandle.serviceClient<common_utilities::Record>("/roboy/record");
    m_recordSteerPublisher = m_nodeHandle.advertise<common_utilities::Steer>("/roboy/steer_record", 1000);
}

// MyoMaster Interface
void ROSMasterCommunication::eventHandle_sendInitializeRequest() {
    TRANSCEIVER_LOG << "Consume Service 'initialize'";

    common_utilities::Initialize initialize;
    for(auto controller : m_initializationList) {
        initialize.request.idList.push_back(controller->m_id);
        initialize.request.controlmode.push_back(controller->m_controlMode);
    }

    if(m_initializeClient.call(initialize)) {
        TRANSCEIVER_SUC << "Service call successfull";
    } else {
        TRANSCEIVER_WAR << "Failed to call Service";
    }
}

void ROSMasterCommunication::eventHandle_sendSteeringMessage() {
    common_utilities::Steer steer;
    steer.steeringCommand = m_steeringCommand;

    ros::Rate rollRate(10);
    while(m_steerPublisher.getNumSubscribers() == 0) {
        TRANSCEIVER_LOG << "Send Steering Message - Wait for subscribers";
        rollRate.sleep();
    }

    TRANSCEIVER_LOG << "Sending steering message: " << m_steeringCommand << " to " << m_steerPublisher.getNumSubscribers() << " subscribers.";
    m_steerPublisher.publish(steer);
}

void ROSMasterCommunication::eventHandle_recordBehavior() {
    TRANSCEIVER_LOG << "Start Recording: SampleRate: " << m_sampleRate;
    bool res = false;

    common_utilities::Record message;
    message.request.sampleRate = m_sampleRate;

    common_utilities::ControllerRequest controllerRequest;
    for(auto controller : m_recordRequest.values()) {
        TRANSCEIVER_LOG << "Request Controller: " << controller->m_id;
        controllerRequest.id = controller->m_id;
        controllerRequest.controlmode = controller->m_controlMode;
        message.request.controllers.push_back(controllerRequest);
    }

    res = m_recordClient.call(message);
    if(res) {
        TRANSCEIVER_LOG << "Record successful";
        RoboyBehavior * behavior = new RoboyBehavior();
        Trajectory trajectory;
        RoboyWaypoint waypoint;

        qint32 i = 0;
        for(auto t : message.response.trajectories) {
            trajectory.m_listWaypoints.clear();
            trajectory.m_sampleRate = (qint32) t.samplerate;
            trajectory.m_controlMode = m_recordRequest[t.id]->m_controlMode;
            for(auto wp : t.waypoints) {
                waypoint.m_ulValue = wp;
                trajectory.m_listWaypoints.append(waypoint);
            }
            behavior->m_mapMotorTrajectory.insert(t.id, trajectory);
            TRANSCEIVER_LOG << "Trajectory : " << i++ << " motor-id: " << t.id << " waypoints: " << t.waypoints.size() << " sampleRate: " << trajectory.m_sampleRate << " ControlMode: " << trajectory.m_controlMode;
        }

        m_mutexData.lock();
        m_pRecordedBehavior = behavior;
        m_mutexData.unlock();

        res = true;
    } else {
        TRANSCEIVER_WAR << "Record failed";
        m_mutexData.lock();
        m_pRecordedBehavior = nullptr;
        m_mutexData.unlock();
        res = false;
    }
    emit signalRecordFinished(res);
}

void ROSMasterCommunication::eventHandle_sendRecordSteeringMessage() {
    TRANSCEIVER_LOG << "Send Steering Command " << m_recordSteeringCommand;

    common_utilities::Steer message;
    message.steeringCommand = m_recordSteeringCommand;
    m_recordSteerPublisher.publish(message);
}

void ROSMasterCommunication::startControllers(const QList<qint32> & controllers) {
//    controller_manager_msgs::SwitchController message;
//
//    std::vector<std::string> resources;
//    for(qint8 id : controllers) {
//        QString name;
//        name.sprintf("motor%u", id);
//        resources.push_back(name.toStdString());
//    }
//
//    message.request.start_controllers = resources;
//    message.request.strictness = 1;
//
//    ros::Duration duration(1);
//    duration.sleep();
//    m_switchController.call(message);
//    TRANSCEIVER_LOG << "Start Call returned";
}

void ROSMasterCommunication::stopControllers(const QList<qint32> & controllers) {
//    controller_manager_msgs::SwitchController message;
//
//    std::vector<std::string> resources;
//    for(qint8 id : controllers) {
//        QString name;
//        name.sprintf("motor%u", id);
//        resources.push_back(name.toStdString());
//    }
//
//    message.request.stop_controllers = resources;
//    message.request.strictness = 1;
//
//    ros::Duration duration(1);
//    duration.sleep();
//    m_switchController.call(message);
//    TRANSCEIVER_LOG << "Start Call returned";
}


void ROSMasterCommunication::unloadControllers(const QList<qint32> & controllers) {
//    controller_manager_msgs::UnloadController message;
//
//    for(qint8 id : controllers) {
//        QString name;
//        name.sprintf("motor%u", id);
//        message.request.name = name.toStdString();
//        m_unloadController.call(message);
//    }
//    TRANSCEIVER_LOG << "Unload Calls returned";
}
