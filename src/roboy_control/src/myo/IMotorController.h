//
// Created by bruh on 3/16/16.
//

#ifndef ROBOY_CONTROL_ICONTROLLERHANDLE_H
#define ROBOY_CONTROL_ICONTROLLERHANDLE_H

#include "DataTypes.h"
#include "LogDefines.h"

class IMotorController {

protected:
    qint32  m_id;
    QString m_name;

    ControlMode     m_controlMode = ControlMode::UNDEFINED_CONTROL;
    ControllerState m_state = ControllerState::UNDEFINED;

public:
    IMotorController(qint32 id, const ControlMode controlMode) {
        m_id = id;
        QString name;
        m_name = name.sprintf("motor%u", m_id);
        m_controlMode = controlMode;
    }

    virtual ~IMotorController() {
        TRANSCEIVER_LOG << "IMotorController Destructor";
    }

    qint32 getId() const {
        return m_id;
    }

    ControlMode getControlMode() const {
        return m_controlMode;
    }

    ControllerState getState() const {
        return m_state;
    }

    void setState(ControllerState m_state) {
        IMotorController::m_state = m_state;
    }

    QString toString() const {
        QString string;
        string.sprintf("MotorController[id=%i][controlMode=%i][state=%i]", m_id, m_controlMode, m_state);
        return string;
    }

    virtual void sendTrajectory(const Trajectory & trajectory) const = 0;
};


#endif //ROBOY_CONTROL_ICONTROLLERHANDLE_H
