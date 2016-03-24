#include "RoboyView.h"

/**
 * @brief RoboyView::RoboyView constructor
 * @param modelService modelService from which the RoboyBehaviors are retrieved
 * @param parent non mandatory parent for the RoboyView
 */
RoboyView::RoboyView(IModelService * pModelService, ViewController * pViewController, QQmlApplicationEngine * pAppEngine, QObject * pParent) : QObject(pParent) {
    m_pViewController         = pViewController;
    m_pModelService           = pModelService;
    m_pAppEngine              = pAppEngine;

    QQmlContext * pQmlContext = pAppEngine->rootContext();
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
 * @brief RoboyView::signalPlayerStatusUpdated method to notfiy the gui when the players state changes
 * @param state state of the playerview
 */
void RoboyView::signalPlayerStatusUpdated(PlayerState state) {

}

/**
 * @brief RoboyView::controllerStateChanged method to notify the gui about a when the state of a motor changed
 * @param motorId id of the motor of which the state changed
 * @param state state of the motor
 */
void RoboyView::signalControllerStatusUpdated(qint32 motorId, ControllerState state) {

}

/**
 * @brief RoboyView::initializeButtonClicked click handler for the initialize button
 */
void RoboyView::initalizeButtonClicked() {
	m_pViewController->triggerInit();
}