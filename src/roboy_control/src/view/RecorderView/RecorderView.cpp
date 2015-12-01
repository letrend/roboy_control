#include "RecorderView.h"
#include "ui_RecorderView.h"

RecorderView::RecorderView(IModelService *modelService, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RecorderView)
{
    this->ui->setupUi(this);
}

RecorderView::~RecorderView()
{
    delete this->ui;
}

void RecorderView::notify()
{

}
