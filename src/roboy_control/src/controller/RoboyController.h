//
// Created by bruh on 30.11.15.
//

#ifndef ROBOYCONTROL_ROBOYCONTROLLER_H
#define ROBOYCONTROL_ROBOYCONTROLLER_H

#include "../model/XmlModelService.h"
#include "../view/MainWindow/MainWindow.h"

class RoboyController {

private:
    IModelService * m_pModelService;
    MainWindow *    m_pMainWindow;


    void initializeComponents();

public:
    RoboyController();
    ~RoboyController();
};


#endif //ROBOYCONTROL_ROBOYCONTROLLER_H
