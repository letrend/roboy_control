#include "ROSMessageTransceiverService.h"

ROSMessageTransceiverService::ROSMessageTransceiverService(qint32 motorId, QString name) : ITransceiverService(motorId, name) {
    m_switchController = m_nodeHandle.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    m_unloadController = m_nodeHandle.serviceClient<controller_manager_msgs::UnloadController>("/controller_manager/unload_controller");

    m_recordClient     = m_nodeHandle.serviceClient<common_utilities::Record>("/roboy/record");

    m_steerRecording= m_nodeHandle.advertise<common_utilities::Steer>("/roboy/steer_recording", 1000);
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

        // Advertise topic for broadcasting steering messages
        m_steerPublisher = m_nodeHandle.advertise<common_utilities::Steer>("/roboy/steer", 1000);

    } else {
        TRANSCEIVER_LOG << "Failed to call Service";
    }
}

void ROSMessageTransceiverService::startControllers(const QList<qint32> & controllers) {
    controller_manager_msgs::SwitchController message;

    std::vector<std::string> resources;
    for(qint8 id : controllers) {
        QString name;
        name.sprintf("motor%u", id);
        resources.push_back(name.toStdString());
    }

    message.request.start_controllers = resources;
    message.request.strictness = 1;

    ros::Duration duration(1);
    duration.sleep();
    m_switchController.call(message);
    TRANSCEIVER_LOG << "Start Call returned";
}

void ROSMessageTransceiverService::unloadControllers(const QList<qint32> & controllers) {
    controller_manager_msgs::UnloadController message;

    for(qint8 id : controllers) {
        QString name;
        name.sprintf("motor%u", id);
        message.request.name = name.toStdString();
        m_unloadController.call(message);
    }
    TRANSCEIVER_LOG << "Unload Calls returned";
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
    // Open topic to receive status updates for specific motor controller
    TRANSCEIVER_LOG << "Subscribe to Status Topic";
    QString topic;
    topic.sprintf("/roboy/status_motor%u", m_motorId);
    m_statusSubscriber = m_nodeHandle.subscribe(topic.toStdString(), 1000, &ROSMessageTransceiverService::callbackStatus, this);
}

void ROSMessageTransceiverService::startRecording() {
    bool res = false;
    TRANSCEIVER_LOG << "Start Recording";

    common_utilities::Record message;
    QList<qint32> ids;
    QList<qint32> modes;
    for(ROSController controller : m_recordRequest) {
        ids.append(controller.id);
        modes.append(controller.controlMode);
    }

    message.request.idList = ids.toVector().toStdVector();
    message.request.controlmode = modes.toVector().toStdVector();
    message.request.samplingTime = 10.0;

    res = m_recordClient.call(message);

    if(res) {
        TRANSCEIVER_LOG << "Record successful";
        std::vector<float> waypoints;

        for(auto v : message.response.trajectories) {
            TRANSCEIVER_LOG << "Received Waypoints for controller: " << v.waypoints.size();
        }
    } else {
        TRANSCEIVER_WAR << "Record failed";
    }

}

void ROSMessageTransceiverService::sendRecordingSteeringMessage(SteeringCommand command) {
    TRANSCEIVER_LOG << "Send Steering Command " << command;

    common_utilities::Steer message;
    message.steeringCommand = command;
    m_steerRecording.publish(message);


}

// Private Interface
void ROSMessageTransceiverService::callbackStatus(const common_utilities::StatusConstPtr & status) {
    TRANSCEIVER_LOG << "Received Status Update";
    ROSController controller;
    controller.id = m_motorId;
    controller.state = (STATUS) status->statusCode;
    delegate->receivedControllerStatusUpdate(controller);
}

