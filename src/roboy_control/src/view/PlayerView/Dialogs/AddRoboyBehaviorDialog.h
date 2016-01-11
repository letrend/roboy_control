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
    explicit AddRoboyBehaviorDialog(qint32 laneCount, QWidget *parent = 0);
    ~AddRoboyBehaviorDialog();

    qint64 selectedTimestamp();
    qint32 selectedLane();

private:
    Ui::AddRoboyBehaviorDialog *ui;
};

#endif // ADDROBOYBEHAVIORDIALOG_H
