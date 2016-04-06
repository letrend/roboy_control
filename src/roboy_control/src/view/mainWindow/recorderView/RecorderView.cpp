#include "RecorderView.h"

/**
 * @brief RecorderView::RecorderView constructor
 * @param modelService modelService from which the RoboyBehaviors are retrieved
 * @param parent non mandatory parent for the EditorView
 */
RecorderView::RecorderView(IModelService * pModelService, ViewController * pViewController, QQmlApplicationEngine *pAppEngine, QObject * pParent) :
	QObject(pParent) {
	m_pViewController       = pViewController;
    m_pModelService         = pModelService;
    m_pAppEngine            = pAppEngine;
}

/**
 * @brief RecorderView::~RecorderView destructor
 */
RecorderView::~RecorderView() {

}

/**
 * @brief RecorderView::notify method to notify about data changes implemented from IObserver interface
 */
void RecorderView::notify() {

}

/**
 * @brief RecorderView::signalPlayerStatusUpdated method to notfiy the gui when the players state changes
 * @param state state of the playerview
 */
void RecorderView::signalPlayerStatusUpdated(PlayerState state) {

}

/**
 * @brief RecorderView::controllerStateChanged method to notify the gui about a when the state of a motor changed
 * @param motorId id of the motor of which the state changed
 * @param state state of the motor
 */
void RecorderView::signalControllerStatusUpdated(qint32 motorId, ControllerState state) {

}

/**
 * @brief RecorderView::recordButtonClicked click handler for the record button
 */
void RecorderView::recordButtonClicked() {
    VIEW_DBG << "Record Button Clicked";
    m_pViewController->recordBehavior();
}

/**
 * @brief RecorderView::stopRecordButtonClicked click handler for the stop record button
 */
void RecorderView::stopRecordButtonClicked() {
    VIEW_DBG << "Stop Record Button Clicked";
    m_pViewController->stopRecording();
}
