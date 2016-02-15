//
// Created by bruh on 1/29/16.
//

#ifndef ROBOY_CONTROL_DATATYPES_H_H
#define ROBOY_CONTROL_DATATYPES_H_H

#include <QString>

#include "CommonDefinitions.h"

struct ROSController {
    qint8                 id;
    ControlMode           controlMode;
    STATUS                state;
    QProcess *            process;

    QString toString() const {
        QString string;
        string.sprintf("ROSController: [id:%i][controlmode:%i][state:%i]", id, controlMode, state);
        return string;
    }
};


#endif //ROBOY_CONTROL_DATATYPES_H_H
