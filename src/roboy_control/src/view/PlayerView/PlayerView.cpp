#include <QMenu>
#include <QMessageBox>

#include "DataTypes.h"
#include "LogDefines.h"
#include "PlayerView.h"
#include "ui_PlayerView.h"
#include "ViewController.h"

#include "Dialogs/AddRoboyBehaviorDialog.h"

#include "MultiLaneView/MultiLaneView.h"
#include "MultiLaneView/RoboyMultiLaneModel.h"

/**
 * @brief PlayerView::PlayerView constructor
 * @param modelService modelService from which the RoboyBehaviors are retrieved
 * @param pViewController controller of the ui component
 * @param parent non mandatory parent for the MainWindow
 */
PlayerView::PlayerView(IModelService *modelService, ViewController * pViewController, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PlayerView) {
    m_pViewController = pViewController;

    this->ui->setupUi(this);
    this->modelService = modelService;
    this->behaviorListModel = new BehaviorListModel(this->modelService);
    this->ui->behaviorListView->setModel(this->behaviorListModel);
    this->setupConnections();

    this->multiLaneModel = new RoboyMultiLaneModel();
    this->ui->multiLaneView->setModel(this->multiLaneModel);
}

/**
 * @brief PlayerView::~PlayerView destructor
 */
PlayerView::~PlayerView() {
    delete this->ui;
    delete this->behaviorListModel;
}

/**
 * @brief PlayerView::notify method to notify about data changes implemented from IObserver interface
 */
void PlayerView::notify() {
    this->behaviorListModel->notify();
}

/* ui connections */

/**
 * @brief PlayerView::setupConnections method to the signals of all ui controls to their respective handler functions
 */
void PlayerView::setupConnections() {
    /* buttons */
    QObject::connect(ui->playButton, SIGNAL(clicked()), this, SLOT(playButtonClicked()));
    QObject::connect(ui->pauseButton, SIGNAL(clicked()), this, SLOT(pauseButtonClicked()));
    QObject::connect(ui->stopButton, SIGNAL(clicked()), this, SLOT(stopButtonClicked()));
    QObject::connect(ui->skipButton, SIGNAL(clicked()), this, SLOT(skipButtonClicked()));
    QObject::connect(ui->addLaneButton, SIGNAL(clicked()), this, SLOT(addLaneButtonClicked()));

    /* behavior listview */
    connect(ui->behaviorListView, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(showBehaviorListItemMenu(const QPoint&)));
    QObject::connect(ui->behaviorListView, SIGNAL(clicked(const QModelIndex &)), this, SLOT(behaviorListViewCurrentRowChanged(const QModelIndex &)));
    this->setupScaleFactorComboBox();
}

/**
 * @brief PlayerView::setupScaleFactorComboBox method to setup the ComboBox from which the scale factor for the MultiLaneView can be chosen
 */
void PlayerView::setupScaleFactorComboBox() {
    ui->scaleFactorComboBox->addItem("scalefactor milliseconds" );
    ui->scaleFactorComboBox->addItem("scalefactor centiseconds" );
    ui->scaleFactorComboBox->addItem("scalefactor deciseconds"  );
    ui->scaleFactorComboBox->addItem("scalefactor seconds"      );

    QObject::connect(ui->scaleFactorComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(scaleFactorComboxBoxIndexChanged(int)));
}

/**
 * @brief PlayerView::playButtonClicked click handler for the play button
 */
void PlayerView::playButtonClicked() {
    m_pViewController->playBehaviorPlan();
}

/**
 * @brief PlayerView::pauseButtonClicked click handler for the pause button
 */
void PlayerView::pauseButtonClicked() {

}

/**
 * @brief PlayerView::stopButtonClicked click handler for the stop button
 */
void PlayerView::stopButtonClicked() {

}

/**
 * @brief PlayerView::skipButtonClicked click handler for the skip button
 */
void PlayerView::skipButtonClicked() {

}

/**
 * @brief PlayerView::addLaneButtonClicked click handler for the add lane button
 */
void PlayerView::addLaneButtonClicked() {
    if (this->multiLaneModel->addLane() < 0) {
        QMessageBox msgBox;
        msgBox.setWindowTitle("Adding lane failed");
        msgBox.setText("Adding a new lane to the MultiLaneView failed.");
        msgBox.exec();
    }
}

/**
 * @brief PlayerView::behaviorListViewCurrentRowChanged method for filling the behaviorDetailView with information about the selected behavior
 * @param index index of the currently selected behavior in the behaviorListView
 */
void PlayerView::behaviorListViewCurrentRowChanged(const QModelIndex & index) {
    this->currentlyDisplayedBehavior = this->behaviorListModel->getBehavior(index.row());
    ui->behaviorNameValueLabel->setText(this->currentlyDisplayedBehavior.m_metadata.m_sBehaviorName);
    ui->idValueLabel->setText(QString::number(this->currentlyDisplayedBehavior.m_metadata.m_ulBehaviorId));
    ui->motorCountValueLabel->setText(QString::number(this->currentlyDisplayedBehavior.m_mapMotorTrajectory.count()));

    ui->motorListView->clear();
    for(u_int32_t iterator : this->currentlyDisplayedBehavior.m_mapMotorTrajectory.keys()) {
        ui->motorListView->addItem(QString("MOTOR ID %1 WAYPOINT COUNT %2").arg(iterator).arg(this->currentlyDisplayedBehavior.m_mapMotorTrajectory.value(iterator).m_listWaypoints.count()));
    }
}

/**
 * @brief PlayerView::showBehaviorListItemMenu method for invoking the behaviorListItem context menu
 * @param pos position where the context menu has to be shown
 */
void PlayerView::showBehaviorListItemMenu(const QPoint& pos) {
    QPoint globalPos = ui->behaviorListView->mapToGlobal(pos);
    QModelIndex selectedIndex = ui->behaviorListView->indexAt(pos);

    if (selectedIndex.isValid()) {
        QMenu behaviorListItemMenu;
        QAction *addAction = new QAction(QIcon(":/add-img-dark.png"), "add to queue", NULL);
        addAction->setIconVisibleInMenu(true);
        behaviorListItemMenu.addAction(addAction);
        QAction *selectedItem = behaviorListItemMenu.exec(globalPos);

        if (selectedItem) {
            if (selectedItem == addAction) {
                AddRoboyBehaviorDialog dialog(this->multiLaneModel->laneCount());
                if(dialog.exec() == AddRoboyBehaviorDialog::Accepted) {
                    qint32 laneIndex = dialog.selectedLane();
                    qint64 timestamp = dialog.selectedTimestamp();
                    RoboyBehavior behavior = this->behaviorListModel->getBehavior(selectedIndex.row());
                    int success = this->multiLaneModel->insertBehaviorExec(laneIndex, timestamp, behavior);
                    switch(success) {
                    case -1: {
                        QMessageBox msgBox;
                        msgBox.setWindowTitle("Invalid timestamp");
                        msgBox.setText("The given timestamp must be a multiple of the samplerate which is 100ms.");
                        msgBox.exec();
                    }
                    break;
                    case -2: {
                        QMessageBox msgBox;
                        msgBox.setWindowTitle("Adding the behavior failed");
                        msgBox.setText("The inserted behavior does overlap with one or more behaviors in the selected lane.");
                        msgBox.exec();
                    }
                    break;
                    }
                }
            }
        }

        delete(addAction);
    }
}

/**
 * @brief PlayerView::scaleFactorComboxBoxIndexChanged indexChanged handler for the scaleFactor ComboBox
 * @param index index of the currently selected item
 */
void PlayerView::scaleFactorComboxBoxIndexChanged(int index) {
    switch (index) {
    case 0:
        ui->multiLaneView->setScaleFactor(MultiLaneView::scaleFactor::millisecond);
        break;
    case 1:
        ui->multiLaneView->setScaleFactor(MultiLaneView::scaleFactor::centisecond);
        break;
    case 2:
        ui->multiLaneView->setScaleFactor(MultiLaneView::scaleFactor::decisecond);
        break;
    case 3:
        ui->multiLaneView->setScaleFactor(MultiLaneView::scaleFactor::second);
        break;
    default:
        QMessageBox msgBox;
        msgBox.setWindowTitle("Non supported scale factor selected");
        msgBox.setText("The scale factor you selected is not available.");
        msgBox.exec();
        break;
    }
}

/**
 * @brief PlayerView::fromPlayerView_getCurrentRoboyPlan method to retrieve the current behavior plan from the MainWindow
 * @return the current behavior plan
 */
RoboyBehaviorMetaplan PlayerView::fromPlayerView_getCurrentRoboyPlan() {
    return this->multiLaneModel->getBehaviorPlan();
}
