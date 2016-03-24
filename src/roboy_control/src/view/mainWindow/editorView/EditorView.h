#ifndef EDITORVIEW_H
#define EDITORVIEW_H

#include <QObject>
#include <QQmlApplicationEngine>

#include "../BehaviorListModel.h"
#include "DataTypes.h"
#include "IModelService.h"
#include "IObserver.h"
#include "ViewController.h"

class ViewController;

class EditorView : public QObject, public IObserver {
    
    Q_OBJECT

public:
    explicit EditorView(IModelService * pModelService, ViewController * pViewController, QQmlApplicationEngine * pAppEngine, QObject * pParent = 0);
    ~EditorView();

    void notify();
    void signalPlayerStatusUpdated(PlayerState state);
    void signalControllerStatusUpdated(qint32 motorId, ControllerState state);

    Q_INVOKABLE bool setSelectedBehaviorIndex(int index);

    /* functions to update data */
    Q_INVOKABLE bool updateBehaviorName     (QString name);
    Q_INVOKABLE bool updateBehaviorId       (int id      );

    /* button handler functions */
    Q_INVOKABLE bool saveButtonClicked      ();

private:
	ViewController          * m_pViewController;
    IModelService           * m_pModelService;
    QQmlApplicationEngine   * m_pAppEngine;
    BehaviorListModel       * m_pBehaviorListModel;
    RoboyBehavior           * m_pSelectedBehavior;

};

#endif // EDITORVIEW_H