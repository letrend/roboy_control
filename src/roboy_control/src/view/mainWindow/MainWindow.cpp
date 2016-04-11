#include "MainWindow.h"

/**
 * @brief MainWindow::MainWindow cosntructor
 * @param modelService modelService from which the RoboyBehaviors are retrieved
 * @param pViewController controller of the ui component
 * @param parent non mandatory parent for the MainWindow
 */
MainWindow::MainWindow(IModelService *pModelService, ViewController * pViewController, QQmlApplicationEngine * pAppEngine, QObject * pParent) : QObject(pParent) {
    pModelService->subscribe(this);
    m_pEditorView   = new EditorView   (pModelService, pViewController, pAppEngine);
    m_pPlayerView   = new PlayerView   (pModelService, pViewController, pAppEngine);
    m_pRecorderView = new RecorderView (pModelService, pViewController, pAppEngine);
    m_pRoboyView    = new RoboyView    (pModelService, pViewController, pAppEngine);

    QQmlContext * pQmlContext = pAppEngine->rootContext();
    pQmlContext->setContextProperty("cpp_EditorView",   m_pEditorView  );
    pQmlContext->setContextProperty("cpp_PlayerView",   m_pPlayerView  );
    pQmlContext->setContextProperty("cpp_RecorderView", m_pRecorderView);
    pQmlContext->setContextProperty("cpp_RoboyView",    m_pRoboyView   );
}

/**
 * @brief MainWindow::~MainWindow destructor
 */
MainWindow::~MainWindow() {
    delete m_pEditorView;
    delete m_pPlayerView;
    delete m_pRecorderView;
    delete m_pRoboyView;
}

/**
 * @brief MainWindow::notify method to notify about data changes implemented from IObserver interface
 */
void MainWindow::notify() {
    m_pEditorView->notify();
    m_pPlayerView->notify();
    m_pRecorderView->notify();
    m_pRoboyView->notify();
}

/**
 * @brief MainWindow::signalPlayerStatusUpdated method to notfiy the gui when the players state changes
 * @param state state of the playerview
 */
void MainWindow::playerStatusUpdated(PlayerState state) {
    m_pEditorView->playerStatusUpdated(state);
    m_pPlayerView->playerStatusUpdated(state);
    m_pRecorderView->playerStatusUpdated(state);
    m_pRoboyView->playerStatusUpdated(state);
}

/**
 * @brief MainWindow::controllerStateChanged method to notify the gui about a when the state of a motor changed
 * @param motorId id of the motor of which the state changed
 * @param state state of the motor
 */
void MainWindow::controllerStatusUpdated(qint32 motorId, ControllerState state) {
    m_pEditorView->controllerStatusUpdated(motorId, state);
    m_pPlayerView->controllerStatusUpdated(motorId, state);
    m_pRecorderView->controllerStatusUpdated(motorId, state);
    m_pRoboyView->controllerStatusUpdated(motorId, state);
}

/**
 * @brief fromMainWindow_getCurrentRoboyPlan method to retrieve the current behavior plan from the MainWindow
 * @return the current behavior plan
 */
RoboyBehaviorMetaplan MainWindow::fromMainWindow_getCurrentRoboyPlan() {
    return m_pPlayerView->fromPlayerView_getCurrentRoboyPlan();
}