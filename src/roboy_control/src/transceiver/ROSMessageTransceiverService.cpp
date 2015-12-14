#include "ROSMessageTransceiverService.h"

ROSMessageTransceiverService::ROSMessageTransceiverService() {

}

// MyoMaster Interface
void ROSMessageTransceiverService::sendInitializeRequest(std::vector<bool> enable) {

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