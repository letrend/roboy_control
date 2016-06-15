//
// Created by bruh on 3/16/16.
//

#include "ROSMotorController.h"

#include "DataPool.h"

#include "common_utilities/Trajectory.h"
#include "common_utilities/ControllerState.h"

#include "ros/ros.h"

ROSMotorController::ROSMotorController(qint32 id, const ControlMode controlMode)
 : IMotorController(id, controlMode) {
    QString topic;
    topic.sprintf("/roboy/trajectory_motor%u", m_id);
    m_trajectoryPublisher = m_nodeHandle.advertise<common_utilities::Trajectory>(topic.toStdString(), 1000);

    topic.sprintf("/roboy/status_motor%u", m_id);
    TRANSCEIVER_LOG << "Subscribe to Status Topic: " << topic;
    m_statusSubscriber = m_nodeHandle.subscribe(topic.toStdString(), 1000, &ROSMotorController::callbackStatus, this);
}

void ROSMotorController::sendTrajectory(const Trajectory & trajectory) const {
    TRANSCEIVER_LOG << "Send Trajectory - motor id: " << m_id;

    common_utilities::Trajectory trajectoryMessage;
    trajectoryMessage.id = m_id;
    trajectoryMessage.samplerate = trajectory.m_sampleRate;
    for(auto wp : trajectory.m_listWaypoints) {
        trajectoryMessage.waypoints.push_back(wp.m_ulValue);
    }

    m_trajectoryPublisher.publish(trajectoryMessage);
}

// Private Interface
void ROSMotorController::callbackStatus(const common_utilities::ControllerStateConstPtr & status) const {
    TRANSCEIVER_LOG << "Received Status Update: " << status->state;
    DataPool::instance().setMotorControllerState(m_id, (ControllerState) status->state);
}