#ifndef ADDROBOYBEHAVIORDIALOG_H
#define ADDROBOYBEHAVIORDIALOG_H

#include <QDialog>

namespace Ui {
class AddRoboyBehaviorDialog;
}

class AddRoboyBehaviorDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AddRoboyBehaviorDialog(QWidget *parent = 0);
    ~AddRoboyBehaviorDialog();

private:
    Ui::AddRoboyBehaviorDialog *ui;
};

#endif // ADDROBOYBEHAVIORDIALOG_H
