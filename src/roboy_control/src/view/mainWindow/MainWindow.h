#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "DataTypes.h"

#include "../../interfaces/IObserver.h"
#include "../../model/IModelService.h"

class ViewController;

class MainWindow : public IObserver {

public:
    explicit MainWindow(IModelService *modelService, ViewController * pViewController, QWidget *parent = 0);
    ~MainWindow();

    void notify();
    RoboyBehaviorMetaplan fromMainWindow_getCurrentRoboyPlan();

private:
    ViewController * m_pViewController;
};

#endif // MAINWINDOW_H
