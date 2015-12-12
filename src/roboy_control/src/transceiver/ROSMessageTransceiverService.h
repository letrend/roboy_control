#ifndef ROSMESSAGETRANSCEIVERSERVICE_H
#define ROSMESSAGETRANSCEIVERSERVICE_H

#include "ITransceiverService.h"
#include "DataTypes.h"
#include "LogDefines.h"
#include "std_msgs/String.h"

class ROSMessageTransceiverService : public ITransceiverService
{

public:
    ROSMessageTransceiverService();

    void sendRoboyBehaviorPlan(const RoboyBehaviorPlan plan);
    void sendRoboyBehavior(const RoboyBehavior behavior);
    void sendRecordRequest(const int id);
    void sendStopRecordRequest(const int id);
    void sendCancelBehaviorRequest(const int id);
    void callback(const std_msgs::String::ConstPtr& msg);

};

#endif // ROSMESSAGETRANSCEIVERSERVICE_H
