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
    void recorderStatusUpdated(RecorderState recorderState);
    void controllerStatusUpdated(qint32 motorId, ControllerState state);
    void recorderResultReceived();

	Q_INVOKABLE void recordButtonClicked();
    Q_INVOKABLE void pauseButtonClicked();
	Q_INVOKABLE void stopRecordButtonClicked();

    Q_INVOKABLE int getCurrentRecorderState();
    
    Q_INVOKABLE void saveRecorderBehavior(QString behaviorName);

signals:
    void signalRecorderStatusUpdated(RecorderState recorderState);
    void signalRecorderResultReceived();

private:
    RecorderState mRecorderState = RecorderState::RECORDER_READY;

	ViewController          * m_pViewController;
    IModelService           * m_pModelService;
    QQmlApplicationEngine   * m_pAppEngine;
};

#endif // RECORDERVIEW_H