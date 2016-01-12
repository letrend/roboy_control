#include "ROSMessageTransceiverService.h"

ROSMessageTransceiverService::ROSMessageTransceiverService() {
    TRANSCEIVER_LOG << "Subscribe to topic 'motor_status'";
    QString topic = "motor_status";
    ros::Subscriber subscriber = m_nodeHandle.subscribe(topic.toStdString(), 1000, &ROSMessageTransceiverService::receiveControllerStatus, this);
}

// MyoMaster Interface
void ROSMessageTransceiverService::sendInitializeRequest(const std::list<qint8> initializationList) {
    m_bReceivedInitializeResponse = false;

    TRANSCEIVER_LOG << "Publish on topic: 'initialize_request'";
    ros::Publisher publisher = m_nodeHandle.advertise<common_utilities::InitializeRequest>("initialize_request", 1000);

    TRANSCEIVER_LOG << "Subscribing to topic 'initialize_response'";
    ros::Subscriber subscriber = m_nodeHandle.subscribe("initialize_response", 1000, &ROSMessageTransceiverService::receiveInitializeResponse,this);

    TRANSCEIVER_LOG << "Wait for MYO-Master to connect";
    ros::Rate rollRate2(10);
    while(subscriber.getNumPublishers() == 0 || publisher.getNumSubscribers() == 0) {
        rollRate2.sleep();
    }

    TRANSCEIVER_LOG << "MYO-Master connected";

    TRANSCEIVER_LOG << "Preparing Message: 'initialize_request'";
    common_utilities::InitializeRequest request;

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

void ROSMessageTransceiverService::receiveInitializeResponse(const common_utilities::InitializeResponse &msg) {
    TRANSCEIVER_LOG << "Processing Message: 'InitializeResponse'";
    QList<ROSController> controllers;
    ROSController controller;
    for (common_utilities::ControllerState state : msg.controllers) {
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
void ROSMessageTransceiverService::sendTrajectory(quint32 motorId, const Trajectory trajectory) {
    QString topic;
    topic.sprintf("motor%u", motorId);
    TRANSCEIVER_LOG << "Publish on Topic: " << topic;

    ros::Publisher pub = m_nodeHandle.advertise<common_utilities::Trajectory>("motor1", 1000);

    common_utilities::Trajectory trajectoryMsg;
    trajectoryMsg.samplerate = trajectory.m_sampleRate;
    trajectoryMsg.controlmode = trajectory.m_controlMode;
    for (RoboyWaypoint wp : trajectory.m_listWaypoints) {
        trajectoryMsg.waypoints.push_back(wp.m_ulValue);
    }

    ros::Rate rollRate(10);
    while(pub.getNumSubscribers() == 0) {
        rollRate.sleep();
    }

    TRANSCEIVER_LOG << "Send Message TRAJECTORY on topic: " << topic;
    pub.publish(trajectoryMsg);

    while(!m_bStatusReceived){
        ros::spinOnce();
        rollRate.sleep();
        TRANSCEIVER_LOG << "Waiting for motor response.";
    }
    TRANSCEIVER_LOG << "Done sending trajectory.";
}

void ROSMessageTransceiverService::receiveControllerStatus(const common_utilities::ControllerState &msg) {
    TRANSCEIVER_LOG << "Receive Message: 'ControllerState'";
    m_bStatusReceived = true;
    ROSController controller;
    controller.id = msg.id;
    controller.state = (ControllerState) msg.state;
    delegate->receivedControllerStatusUpdate(controller);
}

void ROSMessageTransceiverService::sendSteeringMessage(uint8_t steeringaction) {
    ros::Publisher pub = m_nodeHandle.advertise<common_utilities::Steer>("steeringaction", 1000);
    common_utilities::Steer steer;
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
