#include "AddRoboyBehaviorDialog.h"
#include "ui_AddRoboyBehaviorDialog.h"

AddRoboyBehaviorDialog::AddRoboyBehaviorDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AddRoboyBehaviorDialog)
{
    ui->setupUi(this);
}

AddRoboyBehaviorDialog::~AddRoboyBehaviorDialog()
{
    delete ui;
}
