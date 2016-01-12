#ifndef ROSMESSAGETRANSCEIVERSERVICE_H
#define ROSMESSAGETRANSCEIVERSERVICE_H

#include "DataTypes.h"
#include "ITransceiverService.h"
#include "LogDefines.h"

#include "common_utilities/ControllerState.h"
#include "common_utilities/InitializeRequest.h"
#include "common_utilities/Status.h"
#include "common_utilities/Steer.h"
#include "common_utilities/Trajectory.h"

#include <ros/node_handle.h>
#include "std_msgs/String.h"

#include <vector>
#include <sstream>

class ROSMessageTransceiverService : public ITransceiverService
{

private:
    ros::NodeHandle m_nodeHandle;

    bool m_bReceivedInitializeResponse = false;
    bool m_bStatusReceived = false;

public:
    ROSMessageTransceiverService();

    // MyoMaster Interface
    void sendInitializeRequest(const std::list<qint8> initializationList);
    void receiveInitializeResponse(const common_utilities::InitializeResponse& msg);

    // MotorController Interface
    void sendTrajectory(quint32 motorId, const Trajectory trajectory);
    void receiveControllerStatus(const common_utilities::ControllerState &msg);

    void sendSteeringMessage(uint8_t steeringaction);
};

#endif // ROSMESSAGETRANSCEIVERSERVICE_H
