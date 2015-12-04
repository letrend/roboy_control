//
// Created by bruh on 12/4/15.
//

#ifndef ROBOYCONTROL_LOGDEFINES_H
#define ROBOYCONTROL_LOGDEFINES_H

#include <QDebug>

#define LOG         qDebug()
#define DBG         qDebug()

#define OBSERVABLE_DBG  DBG << "[ Observable ]"

#define MODEL_DBG       DBG << "[ XML Parser ]"

#define CONFIG_DBG      DBG << "[ CONFIG ]"

#endif //ROBOYCONTROL_LOGDEFINES_H
