//
// Created by bruh on 1/29/16.
//

#ifndef ROBOY_CONTROL_DATATYPES_H_H
#define ROBOY_CONTROL_DATATYPES_H_H

#include <QString>

#include "CommonDefinitions.h"

struct ROSController {
    qint8                 id;
    STATUS                state;

    QString toString() const {
        QString string;
        string.sprintf("ROSController: [id:%i][state:%i]", id, state);
        return string;
    }
};


#endif //ROBOY_CONTROL_DATATYPES_H_H
