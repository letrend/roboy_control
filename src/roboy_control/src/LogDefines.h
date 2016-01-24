//
// Created by bruh on 12/4/15.
//

#ifndef ROBOYCONTROL_LOGDEFINES_H
#define ROBOYCONTROL_LOGDEFINES_H

#include <QDebug>

#include "stdio.h"

#define LOG         qDebug()
#define DBG         qDebug()
#define WAR         qWarning()
#define SUC         qCritical()

#define OBSERVABLE_DBG  DBG << "[ Observable ]"

#define MODEL_DBG       DBG << "[ MODEL ]"

#define CONFIG_DBG      DBG << "[ CONFIG ]"

#define VIEW_DBG        DBG << "[ VIEW ]"

#define CONTROLLER_DBG  DBG << "[ CONTROLLER ]"
#define CONTROLLER_WAR  WAR << "[ CONTROLLER ]"
#define CONTROLLER_SUC  SUC << "[ CONTROLLER ]"

#define TRANSCEIVER_LOG DBG << "[ TRANSCEIVER | " << m_name << " ] "
#define TRANSCEIVER_WAR WAR << "[ TRANSCEIVER | " << m_name << " ] "
#define TRANSCEIVER_SUC SUC << "[ TRANSCEIVER | " << m_name << " ] "

#define MYOCONTROLLER_DBG DBG << "[ MYO ]"
#define MYOCONTROLLER_WAR WAR << "[ MYO ]"
#define MYOCONTROLLER_SUC SUC << "[ MYO ]"

#endif //ROBOYCONTROL_LOGDEFINES_H
