//
// Created by bruh on 1/8/16.
//

#ifndef ROBOY_CONTROL_ITRANSCEIVERSERVICEDELEGATE_H
#define ROBOY_CONTROL_ITRANSCEIVERSERVICEDELEGATE_H


class ITransceiverServiceDelegate {

public:
    virtual void receivedControllerStatusUpdate(const QList<ROSController> & controllers) = 0;

};


#endif //ROBOY_CONTROL_ITRANSCEIVERSERVICEDELEGATE_H
