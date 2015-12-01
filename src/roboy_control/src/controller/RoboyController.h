//
// Created by bruh on 30.11.15.
//

#ifndef ROBOYCONTROL_ROBOYCONTROLLER_H
#define ROBOYCONTROL_ROBOYCONTROLLER_H

#include "../model/XmlModelService.h"
#include "../view/MainWindow/MainWindow.h"
#include "../transceiver/ITransceiverService.h"

class RoboyController {

private:
    IModelService       * m_pModelService;
    MainWindow          *    m_pMainWindow;
    ITransceiverService * m_pTransceiverService;

public:
    RoboyController();
    ~RoboyController();

    void startExecution();
    void pauseExecution();
    void stopExecution();

    void updateBehaviorQueue();
};


#endif //ROBOYCONTROL_ROBOYCONTROLLER_H
