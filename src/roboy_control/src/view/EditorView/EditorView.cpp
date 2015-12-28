#include "EditorView.h"
#include "ui_EditorView.h"

/**
 * @brief EditorView::EditorView constructor
 * @param modelService modelService from which the RoboyBehaviors are retrieved
 * @param parent non mandatory parent for the EditorView
 */
EditorView::EditorView(IModelService *modelService, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::EditorView)
{
    this->ui->setupUi(this);
}

/**
 * @brief EditorView::~EditorView destructor
 */
EditorView::~EditorView()
{
    delete this->ui;
}

/**
 * @brief EditorView::notify method to notify about data changes implemented from IObserver interface
 */
void EditorView::notify()
{

}
