#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QObject>
#include <QQmlApplicationEngine>

#include "DataTypes.h"
#include "IObserver.h"
#include "IModelService.h"
#include "editorView/EditorView.h"
#include "playerView/PlayerView.h"
#include "recorderView/RecorderView.h"
#include "roboyView/RoboyView.h"
#include "debugView/DebugView.h"

class ViewController;
class EditorView;
class PlayerView;
class RecorderView;
class RoboyView;
class DebugView;

class MainWindow : public QObject, public IObserver {
    
	Q_OBJECT

public:
    explicit MainWindow(IModelService *pModelService, ViewController * pViewController, QQmlApplicationEngine *pAppEngine, QObject *pParent = 0);
    ~MainWindow();

    void notify();
    
    void playerStatusUpdated();
    void recorderStatusUpdated();
    
    void controllerStatusUpdated(qint32 motorId, ControllerState state);
    
    void recorderResultReceived();

    void dataPoolReset();

    RoboyBehaviorMetaplan fromMainWindow_getCurrentRoboyPlan();

private:
    ViewController * m_pViewController;
    EditorView	   * m_pEditorView;
    PlayerView     * m_pPlayerView;
    RecorderView   * m_pRecorderView;
    RoboyView      * m_pRoboyView;
    DebugView      * m_pDebugView;
};

#endif // MAINWINDOW_H
