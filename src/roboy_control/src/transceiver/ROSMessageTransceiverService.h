#ifndef ROSMESSAGETRANSCEIVERSERVICE_H
#define ROSMESSAGETRANSCEIVERSERVICE_H

#include "ITransceiverService.h"
#include "DataTypes.h"
#include "LogDefines.h"

class ROSMessageTransceiverService : public ITransceiverService
{

public:
    ROSMessageTransceiverService();

    void sendRoboyBehaviorPlan(const RoboyBehaviorPlan plan);
    void sendRoboyBehavior(const RoboyBehavior behavior);
    void sendRecordRequest(const int id);
    void sendStopRecordRequest();
    void sendCancelBehaviorRequest(const int id);

};

#endif // ROSMESSAGETRANSCEIVERSERVICE_H
