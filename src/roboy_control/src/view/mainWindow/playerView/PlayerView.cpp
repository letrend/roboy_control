#include "LogDefines.h"
#include "PlayerView.h"

/**
 * @brief PlayerView::PlayerView constructor
 * @param modelService modelService from which the RoboyBehaviors are retrieved
 * @param pViewController controller of the ui component
 * @param pAppEngine qml application engine of the gui
 * @param parent non mandatory parent for the MainWindow
 */
PlayerView::PlayerView(IModelService *pModelService, ViewController * pViewController, QQmlApplicationEngine * pAppEngine, QObject * pParent) :
    QObject(pParent) {
    m_pViewController       = pViewController;
    m_pModelService         = pModelService;
    m_pAppEngine            = pAppEngine;
    m_pBehaviorListModel    = new BehaviorListModel(pModelService);

    QQmlContext * pQmlContext = pAppEngine->rootContext();
    pQmlContext->setContextProperty("cpp_BehaviorListModel", m_pBehaviorListModel);
}

/**
 * @brief PlayerView::~PlayerView destructor
 */
PlayerView::~PlayerView() {
    delete m_pBehaviorListModel;
}

/**
 * @brief PlayerView::notify method to notify about data changes implemented from IObserver interface
 */
void PlayerView::notify() {
    m_pBehaviorListModel->notify();
}

/**
 * @brief PlayerView::playButtonClicked click handler for the play button
 */
void PlayerView::playButtonClicked() {
    m_pViewController->playBehaviorPlan();
}

/**
 * @brief PlayerView::pauseButtonClicked click handler for the pause button
 */
void PlayerView::pauseButtonClicked() {

}

/**
 * @brief PlayerView::stopButtonClicked click handler for the stop button
 */
void PlayerView::stopButtonClicked() {

}

/**
 * @brief PlayerView::skipButtonClicked click handler for the skip button
 */
void PlayerView::skipButtonClicked() {

}

/**
 * @brief PlayerView::addLaneButtonClicked click handler for the add lane button
 */
void PlayerView::addLaneButtonClicked() {

}

/**
 * @brief PlayerView::fromPlayerView_getCurrentRoboyPlan method to retrieve the current behavior plan from the MainWindow
 * @return the current behavior plan
 */
RoboyBehaviorMetaplan PlayerView::fromPlayerView_getCurrentRoboyPlan() {
    RoboyBehaviorMetaplan p;
    return p;
}