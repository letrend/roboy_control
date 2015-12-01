//
// Created by bruh on 30.11.15.
//

#ifndef ROBOYCONTROL_ITRANSCEIVERSERVICE_H
#define ROBOYCONTROL_ITRANSCEIVERSERVICE_H

#include "../DataTypes.h"

class ITransceiverService {

public:
    // Exemplary implementation to evaluate other compontents
    void sendRoboyBehavior(RoboyBehavior behavior) {
        qDebug() << "[ TRANSCEIVER ] send behavior: " << behavior.m_metadata.m_sBehaviorName;
    }
};


#endif //ROBOYCONTROL_ITRANSCEIVERSERVICE_H
