#ifndef ROSMESSAGETRANSCEIVERSERVICE_H
#define ROSMESSAGETRANSCEIVERSERVICE_H

#include "ITransceiverService.h"

#include "common_utilities/Initialize.h"

#include "common_utilities/ControllerState.h"
//#include "common_utilities/InitializeRequest.h"
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
    ros::Publisher  m_steerPublisher;

    void callbackStatus(common_utilities::ControllerState::ConstPtr & status);

public:
    ROSMessageTransceiverService(qint32 motorId, QString name = QString());

    void sendInitializeRequest();
    void sendTrajectory();
    void sendSteeringMessage();

    void listenOnControllerStatus();
};

#endif // ROSMESSAGETRANSCEIVERSERVICE_H
