#include "ROSMessageTransceiverService.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

ROSMessageTransceiverService::ROSMessageTransceiverService()
{

}

// Exemplary implementation to evaluate other compontents
void ROSMessageTransceiverService::sendRoboyBehavior(const RoboyBehavior behavior) {
    //qDebug() << "[ TRANSCEIVER ] send behavior: " << behavior.m_metadata.m_sBehaviorName;
    //qDebug() << "[ TRANSCEIVER ] send behavior: " << behavior;
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::String>("sendBehavior",1000);
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Behave";
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
    ros::spinOnce();
}
