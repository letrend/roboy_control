#ifndef RECORDERVIEW_H
#define RECORDERVIEW_H

#include <QObject>
#include <QQmlApplicationEngine>

#include "DataTypes.h"
#include "IModelService.h"
#include "IObserver.h"
#include "ViewController.h"

class ViewController;

class RecorderView : public QObject, public IObserver {
    
    Q_OBJECT

public:
    explicit RecorderView(IModelService * pModelService, ViewController * pViewController, QQmlApplicationEngine *pAppEngine, QObject * pParent = 0);
    ~RecorderView();

    void notify();
    void playerStatusUpdated(PlayerState state);
    void controllerStatusUpdated(qint32 motorId, ControllerState state);

	Q_INVOKABLE void recordButtonClicked();
    Q_INVOKABLE void pauseButtonClicked();
	Q_INVOKABLE void stopRecordButtonClicked();

private:
	ViewController          * m_pViewController;
    IModelService           * m_pModelService;
    QQmlApplicationEngine   * m_pAppEngine;

};

#endif // RECORDERVIEW_H