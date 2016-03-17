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

void RecorderView::recordButtonClicked() {
    VIEW_DBG << "Record Button Clicked";
    m_pViewController->recordBehavior();
}

void RecorderView::stopRecordButtonClicked() {
    VIEW_DBG << "Stop Record Button Clicked";
    m_pViewController->stopRecording();
}
