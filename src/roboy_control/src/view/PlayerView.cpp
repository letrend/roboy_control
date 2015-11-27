#include "PlayerView.h"
#include "ui_PlayerView.h"

PlayerView::PlayerView(IModelService *modelService, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PlayerView)
{
    this->ui->setupUi(this);
}

PlayerView::~PlayerView()
{
    delete this->ui;
}

void PlayerView::notify() 
{

}