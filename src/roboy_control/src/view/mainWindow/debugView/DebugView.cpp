#include "DebugView.h"

/**
 * @brief DebugView::DebugView constructor
 * @param modelService modelService from which the RoboyBehaviors are retrieved
 * @param parent non mandatory parent for the DebugView
 */
DebugView::DebugView(IModelService * pModelService, ViewController * pViewController, QQmlApplicationEngine * pAppEngine, QObject * pParent) : QObject(pParent) {
	m_pStateModel             = new MotorStateListModel();
    m_pViewController         = pViewController;
    m_pModelService           = pModelService;
    m_pAppEngine              = pAppEngine;

    QQmlContext * pQmlContext = pAppEngine->rootContext();

    pQmlContext->setContextProperty("cpp_MotorStateListModel", m_pStateModel);
}

/**
 * @brief DebugView::~DebugView destructor
 */
DebugView::~DebugView() {

}

/**
 * @brief DebugView::notify method to notify about data changes implemented from IObserver interface
 */
void DebugView::notify() {
	
}

/**
 * @brief DebugView::controllerStateChanged method to notify the gui about a when the state of a motor changed
 * @param motorId id of the motor of which the state changed
 * @param state state of the motor
 */
void DebugView::controllerStatusUpdated(qint32 motorId, ControllerState state) {
	m_pStateModel->controllerStatusUpdated(motorId, state);
}

/**
 * @brief DebugView::dataPoolReset method for handling a reset of the data pool
 */
void DebugView::dataPoolReset() {
	m_pStateModel->dataPoolReset();
}

/**
 * @brief DebugView::initializeButtonClicked click handler for the initialize button
 */
void DebugView::initalizeButtonClicked() {
	m_pViewController->triggerInit();
}