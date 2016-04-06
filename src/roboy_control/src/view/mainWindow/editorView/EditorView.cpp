#include "EditorView.h"

/**
 * @brief EditorView::EditorView constructor
 * @param modelService modelService from which the RoboyBehaviors are retrieved
 * @param parent non mandatory parent for the EditorView
 */
EditorView::EditorView(IModelService * pModelService, ViewController * pViewController, QQmlApplicationEngine * pAppEngine, QObject * pParent) : QObject(pParent) {
    m_pViewController         = pViewController;
    m_pModelService           = pModelService;
    m_pAppEngine              = pAppEngine;
    m_pBehaviorListModel      = new BehaviorListModel(pModelService);
    m_pSelectedBehavior       = 0;

    QQmlContext * pQmlContext = pAppEngine->rootContext();
    pQmlContext->setContextProperty("cpp_EVBehaviorListModel", m_pBehaviorListModel);
}

/**
 * @brief EditorView::~EditorView destructor
 */
EditorView::~EditorView() {
	delete m_pBehaviorListModel;
}

/**
 * @brief EditorView::notify method to notify about data changes implemented from IObserver interface
 */
void EditorView::notify() {
	m_pBehaviorListModel->notify();
}

/**
 * @brief EditorView::signalPlayerStatusUpdated method to notfiy the gui when the players state changes
 * @param state state of the playerview
 */
void EditorView::signalPlayerStatusUpdated(PlayerState state) {

}

/**
 * @brief EditorView::controllerStateChanged method to notify the gui about a when the state of a motor changed
 * @param motorId id of the motor of which the state changed
 * @param state state of the motor
 */
void EditorView::signalControllerStatusUpdated(qint32 motorId, ControllerState state) {

}

/**
 * @brief EditorVire::setSelectedBehaviorIndex method to set the current behavior selection by index
 * @param index index of the behavior that should be selected
 * @return true for success, false for failure
 */
bool EditorView::setSelectedBehaviorIndex(int index) {
	if(index >= 0 || index < m_pBehaviorListModel->rowCount()) {
		static RoboyBehavior behavior = m_pBehaviorListModel->behaviorAt(index);
		m_pSelectedBehavior = &behavior;
		return true;
	}
	return false;
}

/**
 * @brief EditorView::setBehaviorName handler for when the behavior name has been edited
 * @param name the new behavior name
 * @return true for success, false for failure
 */
bool EditorView::updateBehaviorName(QString name) {
	if (m_pSelectedBehavior != 0 && name.length() > 0) {
		m_pSelectedBehavior->m_metadata.m_sBehaviorName = name;
		return true;
	}
	return false;
}

/**
 * @brief EditorView::setBehaviorId handler for when the behavior id has been edited
 * @param id the new behavior id
 * @return true for success, false for failure
 */
bool EditorView::updateBehaviorId(int id) {
	if (m_pSelectedBehavior != 0) {
		m_pSelectedBehavior->m_metadata.m_ulBehaviorId = (quint64) id;
		return true;
	}
	return false;
}

/**
 * @brief EditorView::saveButtonClicked click handler for the save button
 * @param index index of the behavior that is supposed to be safed
 * @return true for success, false for failure
 */
bool EditorView::saveButtonClicked() {
	if (m_pSelectedBehavior != 0) {
		m_pModelService->updateRoboyBehavior(*m_pSelectedBehavior);
		return true;
	}
	return false;
}