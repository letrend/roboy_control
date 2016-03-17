//
// Created by bruh on 3/16/16.
//

#include "ROSControllerHandle.h"
ROSControllerHandle::ROSControllerHandle(qint8 controllerId, ControlMode controlMode)
 : IControllerHandle(controllerId, controlMode)
{
    listenOnControllerStatus();
}

bool ROSControllerHandle::sendTrajectory(const Trajectory & trajectory) {

}

// Private Interface
void ROSControllerHandle::listenOnControllerStatus() {
    // Open topic to receive status updates for specific motor controller
    //TRANSCEIVER_LOG << "Subscribe to Status Topic";
    //QString topic;
    //topic.sprintf("/roboy/status_motor%u", m_motorId);
    //m_statusSubscriber = m_nodeHandle.subscribe(topic.toStdString(), 1000, &ROSMessageTransceiverService::callbackStatus, this);
}

void ROSControllerHandle::callbackStatus(const common_utilities::ControllerStateConstPtr & status) {
    //TRANSCEIVER_LOG << "Received Status Update";
    //ROSController controller;
    //controller.id = status->id;
    //controller.state = (STATUS) status->state;
    //delegate->receivedControllerStatusUpdate(controller);
}

