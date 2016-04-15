//
// Created by bruh on 12/4/15.
//

#include "DataPool.h"
#include "ViewController.h"

/**
 * @brief ViewController::ViewController constructor
 * @param pModelService modelService from which the RoboyBehaviors are retrieved
 */
ViewController::ViewController(IModelService * pModelService) {
    VIEW_DBG << "Initialize View";
    VIEW_DBG << "View Thread ID is: " << QThread::currentThreadId();

    m_pModelSerivce    = pModelService;

    m_pApplicationEngine = new QQmlApplicationEngine();
    m_pApplicationEngine->addImportPath("qrc:/");

    m_pMainWindow = new MainWindow(m_pModelSerivce, this, m_pApplicationEngine);

    m_pApplicationEngine->load(QUrl(QStringLiteral("qrc:/mainWindow/MainWindow.qml")));

    connect(DataPool::getInstance(), SIGNAL(signalNotifyOnPlayerStateUpdated()), this, SLOT(
            slotPlayerStateUpdated()));
    connect(DataPool::getInstance(), SIGNAL(signalNotifyOnControllerStateUpdated(qint32)), this, SLOT(
            slotControllerStateUpdated(qint32)));
    connect(DataPool::getInstance(), SIGNAL(signalNotifyOnRecorderStateUpdated()), this, SLOT(slotRecorderStateUpdated()));
    connect(DataPool::getInstance(), SIGNAL(signalNotifyOnRecordResult()), this, SLOT(slotRecorderResultReceived()));
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
void ViewController::slotPlayerStateUpdated() const {
    PlayerState newState = DataPool::getInstance()->getPlayerState();
    VIEW_DBG << "Received Player Status Update: " << newState;
    m_pMainWindow->playerStatusUpdated(newState);
}

void ViewController::slotRecorderStateUpdated() const {
    // TODO
}

/**
 * @brief ViewController::controllerStateChanged slot to notify the gui about a when the state of a motor changed
 * @param motorId id of the motor of which the state changed
 * @param state state of the motor
 */
void ViewController::slotControllerStateUpdated(qint32 motorId) const {
    ControllerState newState = DataPool::getInstance()->getControllerState(motorId);
    VIEW_DBG << "Received Controller Status Update: " << motorId << "\t" << newState;
    m_pMainWindow->controllerStatusUpdated(motorId, newState);
}

void ViewController::slotRecorderResultReceived() const {
    VIEW_DBG << "Received Record Result";

    // TODO:
    if(DataPool::getInstance()->getRecordResult()) {
        RoboyBehavior * behavior = DataPool::getInstance()->getRecordedBehavior();
        behavior->m_metadata.m_sBehaviorName = "Record1";
        behavior->m_metadata.m_ulBehaviorId = 100;
        for(auto id : behavior->m_mapMotorTrajectory.keys()) {
            if(behavior->m_mapMotorTrajectory[id].m_listWaypoints.isEmpty())
                behavior->m_mapMotorTrajectory.remove(id);
        }
        m_pModelSerivce->createRoboyBehavior(*behavior);
    } else {
        VIEW_DBG << "Record ERROR";
    }
}