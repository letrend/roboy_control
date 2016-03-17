//
// Created by bruh on 3/16/16.
//

#ifndef ROBOY_CONTROL_ICONTROLLERHANDLE_H
#define ROBOY_CONTROL_ICONTROLLERHANDLE_H


class IControllerHandle {

protected:
    qint8                 controllerId;
    ControlMode           controlMode;
    STATUS                state;

public:
    IControllerHandle(qint8 controllerId, ControlMode controlMode) {
        this->controllerId = controllerId;
        this->controlMode = controlMode;
        this->state = STATUS::UNDEFINED;
    }

    virtual bool sendTrajectory(const Trajectory & trajectory) = 0;

    // Getter & Setter
    qint8 getControllerId() {
        return controllerId;
    }

    ControlMode getControlMode() {
        return controlMode;
    }

    STATUS getControllerState() {
        return state;
    }
};


#endif //ROBOY_CONTROL_ICONTROLLERHANDLE_H
