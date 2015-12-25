#include "ROSMessageTransceiverService.h"

ROSMessageTransceiverService::ROSMessageTransceiverService() {
    initialized = false;
}

// MyoMaster Interface
void ROSMessageTransceiverService::sendInitializeRequest(std::vector<bool> enable) {
    ros::Subscriber subscriber = m_nodeHandle.subscribe("initialize_response", 1000, &ROSMessageTransceiverService::receiveInitializeResponse,this);
    TRANSCEIVER_LOG << "Subscribing to initialize_response...";
    ros::Rate rollRate2(10);
    while(subscriber.getNumPublishers() == 0) {
        rollRate2.sleep();
    }
    TRANSCEIVER_LOG << "Subscribing to initialize_response, publishers: " << subscriber.getNumPublishers();

    TRANSCEIVER_LOG << "Preparing initialize request for " << enable.size() << " motors.";
    ros::Publisher publisher = m_nodeHandle.advertise<roboy_control::InitializeRequest>("initialize_request", 1000);

    ros::Rate rollRate(10);
    while(publisher.getNumSubscribers() == 0) {
        rollRate.sleep();
    }
    roboy_control::InitializeRequest request;

    for(bool b : enable) {
        request.enable.push_back(b);
    }

    TRANSCEIVER_LOG << "Sending initialize request for " << enable.size() << " motors to " << publisher.getNumSubscribers() << " subscribers.";
    publisher.publish(request);
    ros::Rate r(10);
    while(!initialized){
        ros::spinOnce();
        r.sleep();
        TRANSCEIVER_LOG << "Waiting for initialize response.";
    }
    TRANSCEIVER_LOG << "Done initializing.";
}

void ROSMessageTransceiverService::receiveInitializeResponse(const roboy_control::InitializeResponse &msg) {
    TRANSCEIVER_LOG << "Processing InitializeResponse.";
    initialized = true;
}

// MotorController Interface
void ROSMessageTransceiverService::sendTrajectory(u_int32_t motorId, QList<RoboyWaypoint> & waypoints) {
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
    roboy_control::Trajectory trajectory;
    for(int k=0; k<waypoints.length(); k++){
        // for every waypoint
        trajectory.waypoints.push_back(waypoints.at(k).m_ulPosition);
    }
    if(waypoints.length() > 1){
        trajectory.samplerate = waypoints.at(1).m_ulTimestamp - waypoints.at(0).m_ulTimestamp;
        TRANSCEIVER_LOG << "Estimated sample rate: " << trajectory.samplerate;
    } else {
        trajectory.samplerate = 100;
        TRANSCEIVER_LOG << "Not enough waypoints, using default samplerate.";
    }
    trajectory.controlmode = 1;
    ros::Rate rollRate(10);
    while(pub.getNumSubscribers() == 0) {
        rollRate.sleep();
    }
    TRANSCEIVER_LOG << "Sending trajectory message on topic "<< pub.getTopic().c_str() <<" to " << pub.getNumSubscribers() << " subscribers.";
    pub.publish(trajectory);
    ros::Rate r(10);
    while(!status_received){
        ros::spinOnce();
        r.sleep();
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
