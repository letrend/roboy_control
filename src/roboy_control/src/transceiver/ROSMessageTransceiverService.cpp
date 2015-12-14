#include "ROSMessageTransceiverService.h"

ROSMessageTransceiverService::ROSMessageTransceiverService() {

}

// MyoMaster Interface
void ROSMessageTransceiverService::sendInitializeRequest(std::vector<bool> enable) {
    ros::Publisher publisher = m_nodeHandle.advertise<roboy_control::InitializeRequest>("initialize_request", 1000);

    ros::Rate rollRate(10);
    while(publisher.getNumSubscribers() == 0) {
        rollRate.sleep();
    }
    roboy_control::InitializeRequest request;

    for(bool b : enable) {
        request.enable.push_back(b);
    }

    publisher.publish(request);
}

void ROSMessageTransceiverService::receiveInitializeResponse() {

}

// MotorController Interface
void ROSMessageTransceiverService::sendTrajectory(u_int32_t motorId, QList<RoboyWaypoint> & waypoints) {
    QString topic = "motor";

    TRANSCEIVER_LOG << "Publish on Topic: " << topic;

    roboy_control::Trajectory trajectoryMessage;
    trajectoryMessage.waypoints.push_back(2);

    ros::Publisher pub = m_nodeHandle.advertise<std_msgs::String>("motor",1000);

    ros::Rate loop_rate(10);

    while(ros::ok()) {

        std_msgs::String msg;
        std::stringstream ss;
        ss << "Behave";
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());


        pub.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();
    }
}

void ROSMessageTransceiverService::receiveControllerStatus(u_int32_t motorId) {

}

void ROSMessageTransceiverService::sendSteeringMessage() {

}