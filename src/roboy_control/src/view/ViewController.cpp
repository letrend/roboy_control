//
// Created by bruh on 12/4/15.
//

#include "ViewController.h"
#include "RoboyController.h"

/**
 * @brief ViewController::ViewController constructor
 * @param pRoboyController roboy controller that is in charge of coordinating all components of the program
 * @param pModelService modelService from which the RoboyBehaviors are retrieved
 */
ViewController::ViewController(RoboyController * pRoboyController, IModelService * pModelService) {
    VIEW_DBG << "Initialize View";
    VIEW_DBG << "View Thread ID is: " << QThread::currentThreadId();

    m_pRoboyController = pRoboyController;
    m_pModelSerivce    = pModelService;

    m_pApplicationEngine = new QQmlApplicationEngine();
    m_pApplicationEngine->addImportPath("qrc:/");

    m_pMainWindow = new MainWindow(m_pModelSerivce, this, m_pApplicationEngine);

    m_pApplicationEngine->load(QUrl(QStringLiteral("qrc:/mainWindow/MainWindow.qml")));

    ViewController::connect(pRoboyController, SIGNAL(signalPlayerStatusUpdated(PlayerState)), this, SLOT(signalPlayerStatusUpdated(PlayerState)));
    ViewController::connect(pRoboyController, SIGNAL(signalControllerStatusUpdated(qint32, ControllerState)), this, SLOT(signalControllerStatusUpdated(qint32, ControllerState)));
}

// PlayerView - Interface
void ViewController::triggerInit() {
    VIEW_DBG << "Init Roboy triggered.";
    emit signalInitialize();
}

void ViewController::preprocessBehaviorPlan() {
    VIEW_DBG << "Preprocess Behavior Plan triggered.";
    emit signalPreprocess();
}

void ViewController::playBehaviorPlan() {
    VIEW_DBG << "Play Behavior Plan triggered.";
    emit signalPlay();
}

void ViewController::stopBehaviorPlan() {
    VIEW_DBG << "Stop Behavior Plan triggered.";
    emit signalStop();
}
void ViewController::pauseBehaviorPlan() {
    VIEW_DBG << "Pause Behavior Plan triggered.";
    emit signalPause();
}

// RecorderView - Interface
void ViewController::recordBehavior() {
    emit signalRecord();
}

void ViewController::stopRecording() {
    emit signalStopRecording();
}

// RoboyController - Interface

/**
 * @brief ViewController::fromController_getCurrentRoboyPlan method fore retrieving the current behavior plan
 * @return the current behavior plan
 */
RoboyBehaviorMetaplan ViewController::fromController_getCurrentRoboyPlan() {
    return this->m_pMainWindow->fromMainWindow_getCurrentRoboyPlan();
}

/**
 * @brief ViewController::signalPlayerStatusUpdated slot to notfiy the gui when the players state changes
 * @param state state of the playerview
 */
void ViewController::signalPlayerStatusUpdated(PlayerState state) {
    m_pMainWindow->signalPlayerStatusUpdated(state);
}

/**
 * @brief ViewController::controllerStateChanged slot to notify the gui about a when the state of a motor changed
 * @param motorId id of the motor of which the state changed
 * @param state state of the motor
 */
void ViewController::signalControllerStatusUpdated(qint32 motorId, ControllerState state) {
    m_pMainWindow->signalControllerStatusUpdated(motorId, state);
}
