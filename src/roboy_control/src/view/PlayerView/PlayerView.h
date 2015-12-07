#ifndef PLAYERVIEW_H
#define PLAYERVIEW_H

#include <QModelIndex>
#include <QWidget>

#include "BehaviorListModel.h"
#include "BehaviorQueueModel.h"
#include "../../DataTypes.h"
#include "../../interfaces/IObserver.h"
#include "../../model/IModelService.h"

namespace Ui {
class PlayerView;
}

class ViewController;

class PlayerView : public QWidget, public IObserver
{
    Q_OBJECT

public:
    explicit PlayerView(IModelService *modelService, ViewController * pViewController, QWidget *parent = 0);
    ~PlayerView();

    void notify();

public slots:
    void playButtonClicked();
    void pauseButtonClicked();
    void stopButtonClicked();
    void skipButtonClicked();
    void addToQueueButtonClicked();
    void behaviorQueueListViewCurrentRowChanged(const QModelIndex & index);
    void showBehaviorListItemMenu(const QPoint& pos);
    void showBehaviorQueueItemMenu(const QPoint& pos);

private:
    ViewController * m_pViewController;

    Ui::PlayerView *ui;
    IModelService *modelService;
    BehaviorListModel *behaviorListModel;
    BehaviorQueueModel *behaviorQueueModel;
    RoboyBehaviorMetadata currentlyDisplayedBehaviorMetaData;
    RoboyBehavior currentlyDisplayedBehavior;

    void setupConnections();
};

#endif // PLAYERVIEW_H
