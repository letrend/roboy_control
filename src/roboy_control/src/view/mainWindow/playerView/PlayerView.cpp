#include "LogDefines.h"
#include "PlayerView.h"
#include "RoboyMultiLaneModel.h"

/**
 * @brief PlayerView::PlayerView constructor
 * @param modelService modelService from which the RoboyBehaviors are retrieved
 * @param pViewController controller of the ui component
 * @param pAppEngine qml application engine of the gui
 * @param parent non mandatory parent for the MainWindow
 */
PlayerView::PlayerView(IModelService *pModelService, ViewController * pViewController, QQmlApplicationEngine * pAppEngine, QObject * pParent) : QObject(pParent) {
    m_pViewController       = pViewController;
    m_pModelService         = pModelService;
    m_pAppEngine            = pAppEngine;
    m_pBehaviorListModel    = new BehaviorListModel(pModelService);
    m_pMultiLaneViewModel   = new RoboyMultiLaneModel();

    QQmlContext * pQmlContext = pAppEngine->rootContext();
    pQmlContext->setContextProperty("cpp_PVBehaviorListModel", m_pBehaviorListModel);
    pQmlContext->setContextProperty("cpp_MultiLaneViewModel", m_pMultiLaneViewModel);
}

/**
 * @brief PlayerView::~PlayerView destructor
 */
PlayerView::~PlayerView() {
    delete m_pBehaviorListModel;
    delete m_pMultiLaneViewModel;
}

/**
 * @brief PlayerView::notify method to notify about data changes implemented from IObserver interface
 */
void PlayerView::notify() {
    m_pBehaviorListModel->notify();
}

/**
 * @brief PlayerView::initButtonClicked click handler for the init button
 */
void PlayerView::initButtonClicked() {
    VIEW_DBG << "init button clicked";
    m_pViewController->triggerInit();
}

void PlayerView::preprocessButtonClicked() {
    VIEW_DBG << "preprocess button clicked";
    m_pViewController->preprocessBehaviorPlan();
}

/**
 * @brief PlayerView::playButtonClicked click handler for the play button
 */
void PlayerView::playButtonClicked() {
    VIEW_DBG << "play button clicked";
    m_pViewController->playBehaviorPlan();
}

/**
 * @brief PlayerView::pauseButtonClicked click handler for the pause button
 */
void PlayerView::pauseButtonClicked() {
    VIEW_DBG << "pause button clicked";
    m_pViewController->pauseBehaviorPlan();
}

/**
 * @brief PlayerView::stopButtonClicked click handler for the stop button
 */
void PlayerView::stopButtonClicked() {
    VIEW_DBG << "stop button clicked";
    m_pViewController->stopBehaviorPlan();
}

/**
 * @brief PlayerView::addLaneButtonClicked click handler for the add lane button
 */
void PlayerView::addLaneButtonClicked() {
    m_pMultiLaneViewModel->addLane();
}

/* MultiLaneView related slots */

/**
 * @brief PlayerView::removeLaneHandler handler to remove a lane from the MultiLaneView
 * @param laneIndex index of the lane that should be removed
 */
void PlayerView::removeLaneHandler(qint32 laneIndex) {
    m_pMultiLaneViewModel->removeLane(laneIndex);
}

/**
 * @brief PlayerView::removeItemHandler handler to remove a item from the MultiLaneView
 * @param laneIndex index of the lane of the item that should be removed
 * @param itemIndex index of the item that should be removed
 */
void PlayerView::removeItemHandler(qint32 laneIndex, qint32 itemIndex) {
    m_pMultiLaneViewModel->removeBehaviorExecWithIndex(laneIndex, itemIndex);
}

/**
 * @brief PlayerView::insertBehaviorHandler handler to insert a behavior into the MultiLaneView
 * @param behaviorIndex index of the behavior in the behaviorListView that should be inserted
 * @param laneIndex index of the lane into which the behavior should be inserted
 * @param lTimestamp timestamp at which the behavior should be inserted
 * @return 0 for sucess, < 0 for failure
 */
int PlayerView::insertBehaviorHandler(qint32 behaviorIndex, qint32 laneIndex, qint64 lTimestamp) {
    RoboyBehavior selectedBehavior = m_pBehaviorListModel->behaviorAt(behaviorIndex);
    return m_pMultiLaneViewModel->insertBehaviorExec(laneIndex, lTimestamp, selectedBehavior);
}

/**
 * @brief PlayerView::fromPlayerView_getCurrentRoboyPlan method to retrieve the current behavior plan from the MainWindow
 * @return the current behavior plan
 */
RoboyBehaviorMetaplan PlayerView::fromPlayerView_getCurrentRoboyPlan() {
    return m_pMultiLaneViewModel->getBehaviorMetaPlan();
}