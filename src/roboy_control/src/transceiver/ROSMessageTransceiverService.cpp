#include "ROSMessageTransceiverService.h"

ROSMessageTransceiverService::ROSMessageTransceiverService(qint32 motorId, QString name) : ITransceiverService(motorId, name) {

}

// MyoMaster Interface
void ROSMessageTransceiverService::sendInitializeRequest() {
    TRANSCEIVER_LOG << "Consume Service 'initialize'";
    ros::ServiceClient client = m_nodeHandle.serviceClient<common_utilities::Initialize>("/roboy/initialize");

    common_utilities::Initialize initialize;

    for(ROSController controller : m_initializationList) {
        initialize.request.idList.push_back(controller.id);
        initialize.request.controlmode.push_back(controller.controlMode);
    }

    if(client.call(initialize)) {
        TRANSCEIVER_LOG << "Service call successfull";

        TRANSCEIVER_LOG << "Processing Message: 'initialize' response";
        QList<ROSController> controllers;
        ROSController controller;
        for (common_utilities::ControllerState state : initialize.response.states) {
            controller.id = state.id;
            controller.state = (STATUS) state.state;
            controllers.append(controller);
            TRANSCEIVER_LOG << " - Initialized Controller Id: " << controller.id << " status: " << controller.state;
        }
        if (delegate != nullptr)
            delegate->receivedControllerStatusUpdate(controllers);

        m_steerPublisher = m_nodeHandle.advertise<common_utilities::Steer>("/roboy/steer", 1000);

    } else {
        TRANSCEIVER_LOG << "Failed to call Service";
    }
}

// MotorController Interface
void ROSMessageTransceiverService::sendTrajectory() {
    QString topic;
    topic.sprintf("/roboy/trajectory_motor%u", m_motorId);
    TRANSCEIVER_LOG << "Send Trajectory to motor: " << m_motorId << "on topic: " << topic;
    ros::ServiceClient client = m_nodeHandle.serviceClient<common_utilities::Trajectory>(topic.toStdString());

    common_utilities::Trajectory srv;
    srv.request.samplerate = m_trajectory.m_sampleRate;
    for(RoboyWaypoint wp : m_trajectory.m_listWaypoints) {
        srv.request.waypoints.push_back(wp.m_ulValue);
    }

    if(client.call(srv)) {
        TRANSCEIVER_LOG << "Call " << topic << "successfull";
        TRANSCEIVER_LOG << "Update Controller State: [id:" << m_motorId << "][state:" << srv.response.state << "]";

        ROSController controller;
        controller.id = m_motorId;
        controller.state = (STATUS) srv.response.state;
        delegate->receivedControllerStatusUpdate(controller);
    } else {
        TRANSCEIVER_LOG << "Call " << topic << "failed";
        ROSController controller;
        controller.id = m_motorId;
        controller.state = STATUS::TRAJECTORY_FAILED;
        delegate->receivedControllerStatusUpdate(controller);
    }
}

void ROSMessageTransceiverService::sendSteeringMessage() {
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

void ROSMessageTransceiverService::listenOnControllerStatus() {
    QString topic;
    topic.sprintf("/roboy/status_motor%u", m_motorId);
    //ros::Subscriber subscriber = m_nodeHandle.subscribe(topic.toStdString(), 1000, &ROSMessageTransceiverService::callbackStatus);
    //ros::spin();
}

void ROSMessageTransceiverService::callbackStatus(common_utilities::ControllerState::ConstPtr & status) {
    TRANSCEIVER_LOG << "Received Status Update";
    ROSController controller;
    controller.id = status->id;
    controller.state = (STATUS) status->state;
    delegate->receivedControllerStatusUpdate(controller);
}