#include "RecorderView.h"
#include "DataPool.h"

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
 * @brief RecorderView::recorderStatusUpdated method to notfiy the gui when the players state changes
 * @param state state of the recorder
 */
void RecorderView::recorderStatusUpdated(RecorderState recorderState) {
    mRecorderState = recorderState;
    emit signalRecorderStatusUpdated(recorderState);
}

/**
 * @brief RecorderView::controllerStateChanged method to notify the gui about a when the state of a motor changed
 * @param motorId id of the motor of which the state changed
 * @param state state of the motor
 */
void RecorderView::controllerStatusUpdated(qint32 motorId, ControllerState state) {

}

/**
 * @brief RecorderView::recorderResultReceived handler for when a record result is received 
 */
void RecorderView::recorderResultReceived() {
	emit signalRecorderResultReceived();
}

/**
 * @brief RecorderView::recordButtonClicked click handler for the record button
 */
void RecorderView::recordButtonClicked() {
    VIEW_DBG << "Record Button Clicked";
    m_pViewController->recordBehavior();
}

/**
 * @brief RecorderView::pauseButtonClicked click handler for the pause button
 */
void RecorderView::pauseButtonClicked() {
	VIEW_DBG << "Pause Button Clicked";
    //m_pViewController->pauseBehavior();
}

/**
 * @brief RecorderView::stopRecordButtonClicked click handler for the stop record button
 */
void RecorderView::stopRecordButtonClicked() {
    VIEW_DBG << "Stop Record Button Clicked";
    m_pViewController->stopRecording();
}

/**
 * @brief RecorderView::getCurrentRecorderState getter for the current recorder state
 * @return the current recorder state
 */
int RecorderView::getCurrentRecorderState() {
    return mRecorderState;
}

/**
 * @brief RecorderView::saveRecorderBehavior method to save a recorder behavior to the database
 */
void RecorderView::saveRecorderBehavior(QString behaviorName) {
	if(DataPool::getInstance()->getRecordResult()) {
        RoboyBehavior * behavior = DataPool::getInstance()->getRecordedBehavior();
        behavior->m_metadata.m_sBehaviorName = behaviorName;
        behavior->m_metadata.m_ulBehaviorId = 200;
        for (auto id : behavior->m_mapMotorTrajectory.keys()) {
            if(behavior->m_mapMotorTrajectory[id].m_listWaypoints.isEmpty())
                behavior->m_mapMotorTrajectory.remove(id);
        }
        m_pModelService->createRoboyBehavior(*behavior);
    } else {
        VIEW_DBG << "Record ERROR";
    }
}
