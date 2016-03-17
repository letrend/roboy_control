#ifndef ROSMESSAGETRANSCEIVERSERVICE_H
#define ROSMESSAGETRANSCEIVERSERVICE_H

#include "ITransceiverService.h"

#include "common_utilities/Initialize.h"

#include "common_utilities/ControllerState.h"
//#include "common_utilities/InitializeRequest.h"

#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/UnloadController.h"

#include "common_utilities/Status.h"
#include "common_utilities/Steer.h"
#include "common_utilities/Trajectory.h"
#include "common_utilities/Record.h"

#include <ros/node_handle.h>
#include "std_msgs/String.h"

#include <vector>
#include <string>
#include <sstream>

class ROSMessageTransceiverService : public ITransceiverService
{
private:
    ros::NodeHandle m_nodeHandle;
    ros::Subscriber m_statusSubscriber;
    ros::Publisher  m_steerPublisher;
    ros::Publisher  m_steerRecording;
    ros::ServiceClient m_switchController;
    ros::ServiceClient m_unloadController;
    ros::ServiceClient m_recordClient;

    void callbackStatus(const common_utilities::StatusConstPtr & status);

public:
    ROSMessageTransceiverService(qint32 motorId, QString name = QString());

    void sendInitializeRequest();
    void sendTrajectory();

    void sendSteeringMessage();

    void startRecording();

    void startControllers(const QList<qint32> & controllers);

    void unloadControllers(const QList<qint32> & controllers);

    void listenOnControllerStatus();
    void sendRecordingSteeringMessage(SteeringCommand command);
};

#endif // ROSMESSAGETRANSCEIVERSERVICE_H
