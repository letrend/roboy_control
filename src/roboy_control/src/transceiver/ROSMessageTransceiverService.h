#ifndef ROSMESSAGETRANSCEIVERSERVICE_H
#define ROSMESSAGETRANSCEIVERSERVICE_H

#include "DataTypes.h"
#include "ITransceiverService.h"
#include "LogDefines.h"

#include "roboy_control/InitializeRequest.h"
#include "roboy_control/Status.h"
#include "roboy_control/Steer.h"
#include "roboy_control/Trajectory.h"

#include <ros/node_handle.h>
#include "std_msgs/String.h"

#include <vector>
#include <sstream>


class ROSMessageTransceiverService : public ITransceiverService
{
private:
    ros::NodeHandle m_nodeHandle;
    bool initialized;
    bool status_received;


public:
    ROSMessageTransceiverService();

    // MyoMaster Interface
    void sendInitializeRequest(std::vector<bool> enable);
    void receiveInitializeResponse(const roboy_control::InitializeResponse& msg);

    // MotorController Interface
    void sendTrajectory(u_int32_t motorId, QList<RoboyWaypoint> & waypoints);
    void receiveControllerStatus(const roboy_control::Status &msg);

    void sendSteeringMessage(uint8_t steeringaction);
};

#endif // ROSMESSAGETRANSCEIVERSERVICE_H
