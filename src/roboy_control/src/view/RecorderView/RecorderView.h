#ifndef RECORDERVIEW_H
#define RECORDERVIEW_H

#include <QWidget>

#include "../../DataTypes.h"
#include "../../interfaces/IObserver.h"
#include "../../model/IModelService.h"

namespace Ui {
class RecorderView;
}

class RecorderView : public QWidget, public IObserver {
    Q_OBJECT

public:
    explicit RecorderView(IModelService *modelService, QWidget *parent = 0);
    ~RecorderView();

    void notify();

private:
    Ui::RecorderView *ui;
};

#endif // RECORDERVIEW_H
