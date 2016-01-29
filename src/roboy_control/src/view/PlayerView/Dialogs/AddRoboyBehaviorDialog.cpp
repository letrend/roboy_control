#include "AddRoboyBehaviorDialog.h"
#include "ui_AddRoboyBehaviorDialog.h"

#include <QLineEdit>

AddRoboyBehaviorDialog::AddRoboyBehaviorDialog(qint32 laneCount, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AddRoboyBehaviorDialog)
{
    ui->setupUi(this);

    for (int i = 0; i < laneCount; i++) {
        ui->laneComboBox->insertItem(i, QString("lane %1").arg(i+1));
    }
}

AddRoboyBehaviorDialog::~AddRoboyBehaviorDialog()
{
    delete ui;
}

qint64 AddRoboyBehaviorDialog::selectedTimestamp()
{
    return this->ui->timestampSpinBox->value();
}

qint32 AddRoboyBehaviorDialog::selectedLane()
{
    return this->ui->laneComboBox->currentIndex();
}
