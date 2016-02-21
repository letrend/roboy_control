#ifndef PLAYERVIEW_H
#define PLAYERVIEW_H

#include <QObject>
#include <QQmlApplicationEngine>

#include "BehaviorListModel.h"
#include "DataTypes.h"
#include "IModelService.h"
#include "IObserver.h"
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

    RoboyBehaviorMetaplan fromPlayerView_getCurrentRoboyPlan();

private:
    ViewController          * m_pViewController;
    IModelService           * m_pModelService;
    QQmlApplicationEngine   * m_pAppEngine;
    BehaviorListModel       * m_pBehaviorListModel;
};

#endif // PLAYERVIEW_H