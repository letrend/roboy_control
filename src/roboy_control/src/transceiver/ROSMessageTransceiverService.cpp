#include "ROSMessageTransceiverService.h"

ROSMessageTransceiverService::ROSMessageTransceiverService() {
}

// MyoMaster Interface
void ROSMessageTransceiverService::sendInitializeRequest(const std::list<qint8> initializationList) {
    m_bReceivedInitializeResponse = false;

    TRANSCEIVER_LOG << "Publish on topic: 'initialize_request'";
    ros::Publisher publisher = m_nodeHandle.advertise<roboy_control::InitializeRequest>("initialize_request", 1000);

    TRANSCEIVER_LOG << "Subscribing to topic 'initialize_response'";
    ros::Subscriber subscriber = m_nodeHandle.subscribe("initialize_response", 1000, &ROSMessageTransceiverService::receiveInitializeResponse,this);

    TRANSCEIVER_LOG << "Wait for MYO-Master to connect";
    ros::Rate rollRate2(10);
    while(subscriber.getNumPublishers() == 0 || publisher.getNumSubscribers() == 0) {
        rollRate2.sleep();
    }

    TRANSCEIVER_LOG << "MYO-Master connected";

    TRANSCEIVER_LOG << "Preparing Message: 'initialize_request'";
    roboy_control::InitializeRequest request;

    for(qint8 id : initializationList)
        request.idList.push_back(id);

    TRANSCEIVER_LOG << "Send Message: 'initialize_request', Controller-Count: " << initializationList.size();
    publisher.publish(request);

    TRANSCEIVER_LOG << "Wait for InitializeResponse";
    ros::Rate rate(50);
    while(!m_bReceivedInitializeResponse) {
        ros::spinOnce();
        rate.sleep();
    }
}

void ROSMessageTransceiverService::receiveInitializeResponse(const roboy_control::InitializeResponse &msg) {
    TRANSCEIVER_LOG << "Processing Message: 'InitializeResponse'";
    QList<ROSController> controllers;
    ROSController controller;
    for (roboy_control::ControllerState state : msg.controllers) {
        controller.id = state.id;
        controller.state = (ControllerState) state.state;
        controllers.append(controller);
        TRANSCEIVER_LOG << " - Initialized Controller Id: " << controller.id << " status: " << controller.state;
    }
    if (delegate != nullptr)
        delegate->receivedControllerStatusUpdate(controllers);

    m_bReceivedInitializeResponse = true;
}

// MotorController Interface
void ROSMessageTransceiverService::sendTrajectory(u_int32_t motorId, const Trajectory trajectory) {
    status_received = false;
    ros::Subscriber subscriber = m_nodeHandle.subscribe("motor_status"+ boost::lexical_cast<std::string>(motorId), 1000, &ROSMessageTransceiverService::receiveControllerStatus,this);
    TRANSCEIVER_LOG << "Subscribing to motor_status...";
    ros::Rate rollRate2(10);
    while(subscriber.getNumPublishers() == 0) {
        rollRate2.sleep();
    }
    TRANSCEIVER_LOG << "Subscribing to motor_status, publishers: " << subscriber.getNumPublishers();

    QString topic = "motor" + QString::number(motorId);

    TRANSCEIVER_LOG << "Publish on Topic: " << topic;

    ros::Publisher pub = m_nodeHandle.advertise<roboy_control::Trajectory>("motor" + boost::lexical_cast<std::string>(motorId), 1000);
    roboy_control::Trajectory trajectoryMsg;
    for (RoboyWaypoint wp : trajectory.m_listWaypoints) {
        trajectoryMsg.waypoints.push_back(wp.m_ulValue);
    }

    trajectoryMsg.samplerate = trajectory.m_sampleRate;
    trajectoryMsg.controlmode = trajectory.m_controlMode;

    ros::Rate rollRate(10);
    while(pub.getNumSubscribers() == 0) {
        rollRate.sleep();
    }

    TRANSCEIVER_LOG << "Sending trajectory message on topic "<< pub.getTopic().c_str() <<" to " << pub.getNumSubscribers() << " subscribers.";
    pub.publish(trajectoryMsg);

    while(!status_received){
        ros::spinOnce();
        rollRate.sleep();
        TRANSCEIVER_LOG << "Waiting for motor response.";
    }
    TRANSCEIVER_LOG << "Done sending trajectory.";
}

void ROSMessageTransceiverService::receiveControllerStatus(const roboy_control::Status &msg) {
    TRANSCEIVER_LOG << "Processing motor status message.";
    status_received = true;
}

void ROSMessageTransceiverService::sendSteeringMessage(uint8_t steeringaction) {
    ros::Publisher pub = m_nodeHandle.advertise<roboy_control::Steer>("steeringaction", 1000);
    roboy_control::Steer steer;
    steer.steeringaction = steeringaction;
    ros::Rate rollRate(10);
    while(pub.getNumSubscribers() == 0) {
        rollRate.sleep();
    }
    TRANSCEIVER_LOG << "Sending steering message to " << pub.getNumSubscribers() << " subscribers.";
    pub.publish(steer);
    rollRate.sleep();
    ros::spinOnce();
}
