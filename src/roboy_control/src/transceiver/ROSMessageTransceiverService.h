#ifndef ROSMESSAGETRANSCEIVERSERVICE_H
#define ROSMESSAGETRANSCEIVERSERVICE_H

#include "ITransceiverService.h"
#include "../DataTypes.h"

class ROSMessageTransceiverService : public ITransceiverService
{

public:
    ROSMessageTransceiverService();

    void sendRoboyBehavior(const RoboyBehavior behavior);

};

#endif // ROSMESSAGETRANSCEIVERSERVICE_H
