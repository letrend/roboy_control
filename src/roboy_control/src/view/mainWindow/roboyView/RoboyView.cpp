#include "RoboyView.h"

/**
 * @brief RoboyView::RoboyView constructor
 * @param modelService modelService from which the RoboyBehaviors are retrieved
 * @param parent non mandatory parent for the RoboyView
 */
RoboyView::RoboyView(IModelService * pModelService, ViewController * pViewController, QQmlApplicationEngine * pAppEngine, QObject * pParent) : QObject(pParent) {
	m_pStateModel             = new ControllerStateListModel();
    m_pViewController         = pViewController;
    m_pModelService           = pModelService;
    m_pAppEngine              = pAppEngine;

    QQmlContext * pQmlContext = pAppEngine->rootContext();

    pQmlContext->setContextProperty("cpp_ControllerStateListModel", m_pStateModel);
}

/**
 * @brief RoboyView::~RoboyView destructor
 */
RoboyView::~RoboyView() {

}

/**
 * @brief RoboyView::notify method to notify about data changes implemented from IObserver interface
 */
void RoboyView::notify() {
	
}

/**
 * @brief RoboyView::controllerStateChanged method to notify the gui about a when the state of a motor changed
 * @param motorId id of the motor of which the state changed
 * @param state state of the motor
 */
void RoboyView::controllerStatusUpdated(qint32 motorId, ControllerState state) {
	m_pStateModel->controllerStatusUpdated(motorId, state);
}

/**
 * @brief RoboyView::dataPoolReset method for handling a reset of the data pool
 */
void RoboyView::dataPoolReset() {
	m_pStateModel->dataPoolReset();
}

/**
 * @brief RoboyView::initializeButtonClicked click handler for the initialize button
 */
void RoboyView::initalizeButtonClicked() {
	m_pViewController->triggerInit();
}