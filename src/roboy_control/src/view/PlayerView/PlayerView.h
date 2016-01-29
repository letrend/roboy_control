#ifndef PLAYERVIEW_H
#define PLAYERVIEW_H

#include <QModelIndex>
#include <QWidget>

#include "BehaviorListModel.h"
#include "MultiLaneView/RoboyMultiLaneModel.h"
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
    void addLaneButtonClicked();
    void behaviorListViewCurrentRowChanged(const QModelIndex & index);
    void showBehaviorListItemMenu(const QPoint& pos);
    void scaleFactorComboxBoxIndexChanged(int index);

    RoboyBehaviorMetaplan fromPlayerView_getCurrentRoboyPlan();

private:
    ViewController * m_pViewController;

    Ui::PlayerView *ui;
    IModelService *modelService;
    BehaviorListModel *behaviorListModel;
    IMultiLaneViewModel *multiLaneModel;
    RoboyBehavior currentlyDisplayedBehavior;

    void setupConnections();
    void setupScaleFactorComboBox();
};

#endif // PLAYERVIEW_H
