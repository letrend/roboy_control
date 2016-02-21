#include "EditorView.h"

/**
 * @brief EditorView::EditorView constructor
 * @param modelService modelService from which the RoboyBehaviors are retrieved
 * @param parent non mandatory parent for the EditorView
 */
EditorView::EditorView(IModelService * pModelService, ViewController * pViewController, QQmlApplicationEngine * pAppEngine, QObject * pParent) :
    QObject(pParent) {
    m_pViewController       = pViewController;
    m_pModelService         = pModelService;
    m_pAppEngine            = pAppEngine;
}

/**
 * @brief EditorView::~EditorView destructor
 */
EditorView::~EditorView() {

}

/**
 * @brief EditorView::notify method to notify about data changes implemented from IObserver interface
 */
void EditorView::notify() {

}