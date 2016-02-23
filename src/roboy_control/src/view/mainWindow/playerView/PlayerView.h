#ifndef PLAYERVIEW_H
#define PLAYERVIEW_H

#include <QObject>
#include <QQmlApplicationEngine>

#include "BehaviorListModel.h"
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

public slots:
    void playButtonClicked   ();
    void pauseButtonClicked  ();
    void stopButtonClicked   ();
    void skipButtonClicked   ();
    void addLaneButtonClicked();

    /* MultiLaneView related slots */
    void  removeLaneHandler    (qint32 laneIndex);
    void  removeItemHandler    (qint32 laneIndex, qint32 itemIndex);
    qint8 insertBehaviorHandler(qint32 behaviorIndex, qint32 laneIndex, qint64 lTimestamp);

    RoboyBehaviorMetaplan fromPlayerView_getCurrentRoboyPlan();

private:
    ViewController          * m_pViewController;
    IModelService           * m_pModelService;
    IMultiLaneViewModel     * m_pMultiLaneViewModel;
    QQmlApplicationEngine   * m_pAppEngine;
    BehaviorListModel       * m_pBehaviorListModel;
};

#endif // PLAYERVIEW_H