//
// Created by bruh on 30.11.15.
//

#ifndef ROBOYCONTROL_ITRANSCEIVERSERVICE_H
#define ROBOYCONTROL_ITRANSCEIVERSERVICE_H

#include "DataTypes.h"
#include "LogDefines.h"

class IMotorController;

class IMyoMaster {

protected:
    QString m_name;

public:
    IMyoMaster() {
        m_name = "MASTER";
    }

    // Virtual as it might be used in a polymorphic context.
    // No need to delete Controller-Pointers as they point to instances handled MyoController
    virtual ~IMyoMaster() {
        TRANSCEIVER_LOG << "IMasterCommunictation Destructor";
    }

    virtual void sendInitializeRequest(const QList<IMotorController *> initializationList) const = 0;
    virtual void sendSteeringMessage(const SteeringCommand command) const = 0;
    virtual void startRecording(const QMap<qint32, IMotorController *> controllers, qint32 sampleRate) const = 0;
    virtual void sendRecordSteeringMessage(const SteeringCommand command) const = 0;

    uint idCounter = 0;

//    virtual void startControllers(const QList<qint32> & controllers) = 0;
//    virtual void stopControllers(const QList<qint32> & contollers) = 0;
};

#endif //ROBOYCONTROL_ITRANSCEIVERSERVICE_H