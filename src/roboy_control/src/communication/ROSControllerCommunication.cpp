//
// Created by bruh on 3/16/16.
//

#include "ROSControllerCommunication.h"

ROSControllerCommunication::ROSControllerCommunication(ROSController * controller)
 : IControllerCommunication(controller)
{
    listenOnControllerStatus();
}

void ROSControllerCommunication::eventHandle_sendTrajectory() {
    QString topic;
    topic.sprintf("/roboy/trajectory_motor%u", m_id);
    TRANSCEIVER_LOG << "Send Trajectory: topic: " << topic;

    ros::ServiceClient client = m_nodeHandle.serviceClient<common_utilities::SetTrajectory>(topic.toStdString());

    common_utilities::SetTrajectory service;
    service.request.trajectory.samplerate = m_trajectory.m_sampleRate;
    for(RoboyWaypoint wp : m_trajectory.m_listWaypoints) {
        service.request.trajectory.waypoints.push_back(wp.m_ulValue);
    }

    if(client.call(service)) {
        TRANSCEIVER_LOG << "Call " << topic << "successfull";
    } else {
        TRANSCEIVER_LOG << "Call " << topic << "failed";
    }
}

// Private Interface
void ROSControllerCommunication::listenOnControllerStatus() {
    // Open topic to receive status updates for specific motor controller
    QString topic;
    topic.sprintf("/roboy/status_motor%u", m_id);
    TRANSCEIVER_LOG << "Subscribe to Status Topic: " << topic;
    m_statusSubscriber = m_nodeHandle.subscribe(topic.toStdString(), 1000, &ROSControllerCommunication::callbackStatus, this);
}

void ROSControllerCommunication::callbackStatus(const common_utilities::ControllerStateConstPtr & status) {
    TRANSCEIVER_LOG << "Received Status Update: " << status->state;
    m_pController->m_state = (ControllerState) status->state;
    emit signalControllerStatusUpdated(m_id);
}