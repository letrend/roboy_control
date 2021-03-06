#ifndef PLAYERVIEW_H
#define PLAYERVIEW_H

#include <QObject>
#include <QQmlApplicationEngine>

#include "../BehaviorListModel.h"
#include "DataTypes.h"
#include "IModelService.h"
#include "IObserver.h"
#include "multiLaneView/IMultiLaneViewModel.h"
#include "ViewController.h"

class ViewController;

class PlayerView : public QObject, public IObserver {
    
    Q_OBJECT

public:
    explicit PlayerView(IModelService *pModelService, ViewController * pViewController, QQmlApplicationEngine * pAppEngine, QObject * pParent = 0);
    ~PlayerView();

    void notify();
    void playerStatusUpdated();
    void controllerStatusUpdated(qint32 motorId, ControllerState state);

    // the following methods are defined as Q_INVOKABLE and 
    // not as slots so their return value can be used
    Q_INVOKABLE void preprocessButtonClicked();
    Q_INVOKABLE void playButtonClicked();
    Q_INVOKABLE void pauseButtonClicked();
    Q_INVOKABLE void stopButtonClicked();

    Q_INVOKABLE void addLaneButtonClicked();

    Q_INVOKABLE int getCurrentPlayerState();

    /* MultiLaneView related methods */
    Q_INVOKABLE void removeLaneHandler    (qint32 laneIndex);
    Q_INVOKABLE void removeItemHandler    (qint32 laneIndex, qint32 itemIndex);
    Q_INVOKABLE int  insertBehaviorHandler(qint32 behaviorIndex, qint32 laneIndex, qint64 lTimestamp);

    RoboyBehaviorMetaplan fromPlayerView_getCurrentRoboyPlan();

signals: 
    void signalPlayerStatusUpdated();
    void signalControllerStatusUpdated(qint32 motorId, int controllerState);

private:    
    ViewController        * m_pViewController;
    IModelService         * m_pModelService;
    IMultiLaneViewModel   * m_pMultiLaneViewModel;
    QQmlApplicationEngine * m_pAppEngine;
    BehaviorListModel     * m_pBehaviorListModel;
};

#endif // PLAYERVIEW_H