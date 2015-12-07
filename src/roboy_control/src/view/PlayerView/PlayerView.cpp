#include "PlayerView.h"
#include "ui_PlayerView.h"

#include "ViewController.h"

/**
* @brief constructor
* @param the IModelService providing the roboy behaviors
* @param non mandatory parent
*/
PlayerView::PlayerView(IModelService *modelService, ViewController * pViewController, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PlayerView)
{
    m_pViewController = pViewController;

	this->ui->setupUi(this);
    this->modelService = modelService;
    this->behaviorListModel = new BehaviorListModel(this->modelService);
    this->behaviorQueueModel = new BehaviorQueueModel();
    this->ui->behaviorListView->setModel(this->behaviorListModel);
    this->ui->behaviorQueueListView->setModel(this->behaviorQueueModel);
    this->setupConnections();
}

/**
*@brief desctructor
**/
PlayerView::~PlayerView()
{
    delete this->ui;
    delete this->behaviorListModel;
    delete this->behaviorQueueModel;
}

/**
*@brief method to notify about data changes implemented from IObserver interface
**/
void PlayerView::notify() 
{
	this->behaviorListModel->notify();
}

/* ui connections */

/**
*@brief method to connect all ui controls to their handler functions
**/
void PlayerView::setupConnections()
{
	/* buttons */
	QObject::connect(ui->playButton, SIGNAL(clicked()), this, SLOT(playButtonClicked()));
	QObject::connect(ui->pauseButton, SIGNAL(clicked()), this, SLOT(pauseButtonClicked()));
	QObject::connect(ui->stopButton, SIGNAL(clicked()), this, SLOT(stopButtonClicked()));
	QObject::connect(ui->skipButton, SIGNAL(clicked()), this, SLOT(skipButtonClicked()));
	QObject::connect(ui->addToQueueButton, SIGNAL(clicked()), this, SLOT(addToQueueButtonClicked()));
	/* behavior listview */
	connect(ui->behaviorListView, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(showBehaviorListItemMenu(const QPoint&)));
	connect(ui->behaviorQueueListView, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(showBehaviorQueueItemMenu(const QPoint&)));
	QObject::connect(ui->behaviorListView, SIGNAL(clicked(const QModelIndex &)), this, SLOT(behaviorQueueListViewCurrentRowChanged(const QModelIndex &)));

}

/**
*@brief click handler for the play button
**/
void PlayerView::playButtonClicked()
{
	m_pViewController->playBehaviorPlan();
}

/**
*@brief click handler for the pause button
**/
void PlayerView::pauseButtonClicked()
{
	
}

/**
*@brief click handler for the stop button
**/
void PlayerView::stopButtonClicked()
{
	
}

/**
*@brief click handler for the skip button
**/
void PlayerView::skipButtonClicked()
{
	
}

/**
*@brief click handler for the add to queue button
**/
void PlayerView::addToQueueButtonClicked()
{
	this->behaviorQueueModel->addBehaviorMetaData(this->currentlyDisplayedBehaviorMetaData);
}

/**
*@brief method for filling the behaviorDetailView with information about the selected behavior
*@param index index of the currently selected behavior in the behaviorListView
**/
void PlayerView::behaviorQueueListViewCurrentRowChanged(const QModelIndex & index)
{	
	this->currentlyDisplayedBehaviorMetaData = this->behaviorListModel->getBehaviorMetaData(index.row());
	this->currentlyDisplayedBehavior = this->modelService->retrieveRoboyBehavior(this->currentlyDisplayedBehaviorMetaData);
	this->ui->addToQueueButton->setEnabled(true);
	ui->behaviorNameValueLabel->setText(this->currentlyDisplayedBehavior.m_metadata.m_sBehaviorName);
	ui->idValueLabel->setText(QString::number(this->currentlyDisplayedBehavior.m_metadata.m_ulBehaviorId));
	ui->motorCountValueLabel->setText(QString::number(this->currentlyDisplayedBehavior.m_mapMotorWaypoints.count()));

	ui->motorListView->clear();
	QString description;
	for(u_int32_t iterator : this->currentlyDisplayedBehavior.m_mapMotorWaypoints.keys())
	{
		ui->motorListView->addItem(QString("MOTOR ID %1 WAYPOINT COUNT %2").arg(iterator).arg(this->currentlyDisplayedBehavior.m_mapMotorWaypoints.value(iterator).count()));
	}
}

/**
*@brief
**/
void PlayerView::showBehaviorListItemMenu(const QPoint& pos)
{
    QPoint globalPos = ui->behaviorListView->mapToGlobal(pos);
    QModelIndex selectedIndex = ui->behaviorListView->indexAt(pos);
    
    if (selectedIndex.isValid()) {
    	QMenu behaviorListItemMenu;
    	behaviorListItemMenu.addAction("add to queue");
    	QAction *selectedItem = behaviorListItemMenu.exec(globalPos);

    	if (selectedItem) {
    		if (selectedItem->text() == "add to queue") {
    			this->behaviorQueueModel->addBehaviorMetaData(this->behaviorListModel->getBehaviorMetaData(selectedIndex.row()));
    		}
		}
	}
}

void PlayerView::showBehaviorQueueItemMenu(const QPoint& pos)
{
    QPoint globalPos = ui->behaviorQueueListView->mapToGlobal(pos);
    QModelIndex selectedIndex = ui->behaviorQueueListView->indexAt(pos);

    if (selectedIndex.isValid()) {
    	QMenu behaviorQueueItemMenu;
    	behaviorQueueItemMenu.addAction("remove from queue");

    	QAction *selectedItem = behaviorQueueItemMenu.exec(globalPos);
    	if (selectedItem)
    	{
     		if (selectedItem->text() == "remove from queue") {
     			this->behaviorQueueModel->removeBehaviorMetaData(selectedIndex.row());
     		}   
    	}
	}
}