#ifndef PLAYERVIEW_H
#define PLAYERVIEW_H

#include <QModelIndex>
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

public slots:
    void playButtonClicked();
    void pauseButtonClicked();
    void stopButtonClicked();
    void behaviorListWidgetCurrentRowChanged(int row);

private:
    Ui::PlayerView *ui;
    IModelService *modelService;
    QList<RoboyBehaviorMetadata> behaviorList; //list of avalaible behaviors
    QList<RoboyBehaviorMetadata> behaviorQueue; //list of behaviors that are queued for execution

    void setupConnections();
    void updateBehaviorList();
};

#endif // PLAYERVIEW_H
