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
//    srv.request.controlmode = m_trajectory.m_controlMode;
    for(RoboyWaypoint wp : m_trajectory.m_listWaypoints) {
        srv.request.waypoints.push_back(wp.m_ulValue);
    }

    if(client.call(srv)) {
        TRANSCEIVER_LOG << "Call " << topic << "successfull";
        TRANSCEIVER_LOG << "Update Controller State: [id:" << srv.response.state.id << "][state:" << srv.response.state.state << "]";

        ROSController controller;
        controller.id = srv.response.state.id;
        controller.state = (STATUS) srv.response.state.state;
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
    ros::Publisher pub = m_nodeHandle.advertise<common_utilities::Steer>("/roboy/steering", 1000);
    common_utilities::Steer steer;
    steer.steeringCommand = m_steeringCommand;
    ros::Rate rollRate(10);
    while(pub.getNumSubscribers() == 0) {
        rollRate.sleep();
    }
    TRANSCEIVER_LOG << "Sending steering message to " << pub.getNumSubscribers() << " subscribers.";
    pub.publish(steer);
}
