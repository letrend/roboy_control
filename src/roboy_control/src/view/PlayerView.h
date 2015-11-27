#ifndef PLAYERVIEW_H
#define PLAYERVIEW_H

#include <QWidget>

#include "../DataTypes.h"
#include "../interfaces/IObserver.h"
#include "../model/IModelService.h"

namespace Ui {
class PlayerView;
}

class PlayerView : public QWidget, public IObserver
{
    Q_OBJECT

public:
    explicit PlayerView(IModelService *modelService, QWidget *parent = 0);
    ~PlayerView();

    void notify();

private:
    Ui::PlayerView *ui;
};

#endif // PLAYERVIEW_H
