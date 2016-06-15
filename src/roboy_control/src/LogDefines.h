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

#define TRANSCEIVER_LOG DBG << "[ TRANSCEIVER | " << this->m_name << " ] "
#define TRANSCEIVER_WAR WAR << "[ TRANSCEIVER | " << this->m_name << " ] "
#define TRANSCEIVER_SUC SUC << "[ TRANSCEIVER | " << this->m_name << " ] "

#define MYOCONTROLLER_DBG DBG << "[ MYO ]"
#define MYOCONTROLLER_WAR WAR << "[ MYO ]"
#define MYOCONTROLLER_SUC SUC << "[ MYO ]"

#define PLAN_DBG        DBG << "[ PLAN ]"
#define PLAN_WAR        WAR << "[ PLAN ]"
#define PLAN_SUC        SUC << "[ PLAN ]"

#define DATAPOOL_DBG        DBG << "[ DATAPOOL ]"
#define DATAPOOL_WAR        WAR << "[ DATAPOOL ]"
#define DATAPOOL_SUC        SUC << "[ DATAPOOL ]"

#endif //ROBOYCONTROL_LOGDEFINES_H
