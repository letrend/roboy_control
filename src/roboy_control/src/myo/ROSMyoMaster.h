#ifndef ROSMESSAGETRANSCEIVERSERVICE_H
#define ROSMESSAGETRANSCEIVERSERVICE_H

#include "IMyoMaster.h"

#include "ros/ros.h"

#include "IMotorController.h"

#include "common_utilities/ControllerRequest.h"
#include "common_utilities/Initialize.h"
#include "common_utilities/Steer.h"
#include "common_utilities/Record.h"

class ROSMyoMaster : public IMyoMaster {

private:
    ros::NodeHandle    m_nodeHandle;

    ros::Publisher     m_initializePublisher;
    ros::Publisher     m_steerPublisher;
    ros::Publisher     m_recordClient;
    ros::Publisher     m_recordSteerPublisher;

    ros::ServiceClient m_switchController;
    ros::ServiceClient m_unloadController;

public:
    ROSMyoMaster();
    ~ROSMyoMaster() {};

    void sendInitializeRequest(const QList<IMotorController *> initializationList) const;
    void sendSteeringMessage(const SteeringCommand command) const;
    void startRecording(const QMap<qint32, IMotorController *> controllers, qint32 sampleRate) const;
    void sendRecordSteeringMessage(const SteeringCommand command) const;
};

#endif // ROSMESSAGETRANSCEIVERSERVICE_H
