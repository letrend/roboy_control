#ifndef ROSMESSAGETRANSCEIVERSERVICE_H
#define ROSMESSAGETRANSCEIVERSERVICE_H

#include "DataTypes.h"
#include "ITransceiverService.h"
#include "LogDefines.h"

#include "roboy_control/InitializeRequest.h"
#include "roboy_control/InitializeResponse.h"
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


public:
    ROSMessageTransceiverService();

    // MyoMaster Interface
    void sendInitializeRequest(std::vector<bool> enable);
    void receiveInitializeResponse();

    // MotorController Interface
    void sendTrajectory(u_int32_t motorId, QList<RoboyWaypoint> & waypoints);
    void receiveControllerStatus(u_int32_t motorId);

    void sendSteeringMessage();
};

#endif // ROSMESSAGETRANSCEIVERSERVICE_H
