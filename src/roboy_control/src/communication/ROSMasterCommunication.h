#ifndef ROSMESSAGETRANSCEIVERSERVICE_H
#define ROSMESSAGETRANSCEIVERSERVICE_H

#include "IMasterCommunication.h"

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

class ROSMasterCommunication : public IMasterCommunication
{
private:
    ros::NodeHandle m_nodeHandle;

    ros::ServiceClient m_initializeClient;
    ros::Publisher     m_steerPublisher;

    ros::ServiceClient m_switchController;
    ros::ServiceClient m_unloadController;

    ros::Publisher     m_recordSteerPublisher;
    ros::ServiceClient m_recordClient;

public:
    ROSMasterCommunication();

    void eventHandle_sendInitializeRequest();
    void eventHandle_sendSteeringMessage();

    void eventHandle_recordBehavior();
    void eventHandle_sendRecordSteeringMessage();

    void startControllers(const QList<qint32> & controllers);
    void unloadControllers(const QList<qint32> & controllers);

private:
    void callbackStatus(const common_utilities::StatusConstPtr & status);

};

#endif // ROSMESSAGETRANSCEIVERSERVICE_H
