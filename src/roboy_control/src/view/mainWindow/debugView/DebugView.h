#pragma once

#include <QObject>
#include <QQmlApplicationEngine>

#include "../BehaviorListModel.h"
#include "MotorStateListModel.h"
#include "DataTypes.h"
#include "IModelService.h"
#include "IObserver.h"
#include "ViewController.h"

class ViewController;

class DebugView : public QObject, public IObserver {
    
    Q_OBJECT

public:
    explicit DebugView(IModelService * pModelService, ViewController * pViewController, QQmlApplicationEngine * pAppEngine, QObject * pParent = 0);
    ~DebugView();

    void notify();
    void controllerStatusUpdated(qint32 motorId, ControllerState state);
    void dataPoolReset();

    Q_INVOKABLE void initalizeButtonClicked();

private:
    MotorStateListModel      * m_pStateModel;
	ViewController           * m_pViewController;
    IModelService            * m_pModelService;
    QQmlApplicationEngine    * m_pAppEngine;

};
