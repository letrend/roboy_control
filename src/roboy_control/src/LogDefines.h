//
// Created by bruh on 12/4/15.
//

#ifndef ROBOYCONTROL_LOGDEFINES_H
#define ROBOYCONTROL_LOGDEFINES_H

#include <QDebug>

#define LOG         qDebug()
#define DBG         qDebug()

#define OBSERVABLE_DBG  DBG << "[ Observable ]"

#define MODEL_DBG       DBG << "[ MODEL ]"

#define CONFIG_DBG      DBG << "[ CONFIG ]"

#define VIEW_DBG        DBG << "[ VIEW ]"

#define CONTROLLER_DBG  DBG << "[ CONTROLLER ]"

#define TRANSCEIVER_LOG DBG << "[ TRANSCEIVER ]"

#endif //ROBOYCONTROL_LOGDEFINES_H
