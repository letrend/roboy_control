//
// Created by bruh on 30.11.15.
//

#ifndef ROBOYCONTROL_ITRANSCEIVERSERVICE_H
#define ROBOYCONTROL_ITRANSCEIVERSERVICE_H

#include "../DataTypes.h"

class ITransceiverService {

public:
    virtual void sendRoboyBehaviorPlan(const RoboyBehaviorPlan plan) = 0;
    virtual void sendRoboyBehavior(const RoboyBehavior behavior) = 0;
};


#endif //ROBOYCONTROL_ITRANSCEIVERSERVICE_H
