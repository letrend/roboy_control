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
    m_pModelSerivce = pModelService;

    m_pApplicationEngine = new QQmlApplicationEngine();
    m_pApplicationEngine->addImportPath("qrc:/");

    m_pMainWindow = new MainWindow(m_pModelSerivce, this, m_pApplicationEngine);

    m_pApplicationEngine->load(QUrl(QStringLiteral("qrc:/mainWindow/MainWindow.qml")));
}

/**
 * @brief ViewController::playBehaviorPlan method for triggering playing the current behavior plan
 */
void ViewController::playBehaviorPlan() {
    VIEW_DBG << "Play Behavior Plan triggered.";
    emit signalPlay();
//    m_pRoboyController->fromViewController_triggerPlayPlan();
}

/**
 * @brief ViewController::pauseBehaviorPlan method for triggering pausing the current behavior plan
 */
void ViewController::pauseBehaviorPlan() {
    emit signalPause();
}

/**
 * @brief ViewController:stopBehaviorPlan method for triggering stopping the current behavior plan
 */
void ViewController::stopBehaviorPlan() {
    emit signalStop();
}

/**
 * @brief ViewController::skipBehavior method for triggering skipping the current behavior
 */
void ViewController::skipBehavior() {
    emit signalSkip();
}

/**
 * @brief ViewController::rewindBehavior method for rewinding the current behavior plan
 */
 void ViewController::rewindBehaviorPlan() {
    emit signalRewind();
 }

/**
 * @rbrief ViewController::triggerInit method for triggering the initialization
 */
void ViewController::triggerInit() {
    emit signalInitialize();
}

/**
 * @brief ViewController::fromController_getCurrentRoboyPlan method fore retrieving the current behavior plan
 * @return the current behavior plan
 */
RoboyBehaviorMetaplan ViewController::fromController_getCurrentRoboyPlan() {
    return this->m_pMainWindow->fromMainWindow_getCurrentRoboyPlan();
}
