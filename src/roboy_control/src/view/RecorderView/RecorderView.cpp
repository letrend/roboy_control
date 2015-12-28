#include "RecorderView.h"
#include "ui_RecorderView.h"

/**
 * @brief RecorderView::RecorderView constructor
 * @param modelService modelService from which the RoboyBehaviors are retrieved
 * @param parent non mandatory parent for the EditorView
 */
RecorderView::RecorderView(IModelService *modelService, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RecorderView)
{
    this->ui->setupUi(this);
}

/**
 * @brief RecorderView::~RecorderView destructor
 */
RecorderView::~RecorderView()
{
    delete this->ui;
}

/**
 * @brief RecorderView::notify method to notify about data changes implemented from IObserver interface
 */
void RecorderView::notify()
{

}
