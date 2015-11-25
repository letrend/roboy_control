#include "MainWindow.h"
#include "ui_MainWindow.h"

#include "RoboyBehaviorModel.h"
#include "../model/XmlModelService.h"

MainWindow::MainWindow(QWidget *parent, IModelService * modelService) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    this->modelService = modelService;
    RoboyBehaviorModel * model = new RoboyBehaviorModel(0, modelService);
    ui->behaviorListView->setModel(model);

	this->setupConnections();  
}

MainWindow::~MainWindow()
{
    delete ui;
}


/* ui connections */

void MainWindow::setupConnections()
{
	QObject::connect(ui->playButton, SIGNAL(clicked()), this, SLOT(playButtonClicked()));
	QObject::connect(ui->pauseButton, SIGNAL(clicked()), this, SLOT(pauseButtonClicked()));
	QObject::connect(ui->stopButton, SIGNAL(clicked()), this, SLOT(stopButtonClicked()));
	QObject::connect(ui->addToTimelineButton, SIGNAL(clicked()), this, SLOT(addToTimelineButtonClicked()));
	QObject::connect(ui->behaviorListView, SIGNAL(clicked(const QModelIndex)), this, SLOT(behaviorListViewItemClicked(QModelIndex)));
}

void MainWindow::playButtonClicked()
{
	
}

void MainWindow::pauseButtonClicked()
{
	
}

void MainWindow::stopButtonClicked()
{
	
}

void MainWindow::addToTimelineButtonClicked()
{

}

void MainWindow::behaviorListViewItemClicked(QModelIndex index)
{
	RoboyBehaviorMetadata selectedBehaviorMetaData = this->modelService->getBehaviorList().at(index.row());
	RoboyBehavior selectedBehavior = this->modelService->getBehavior(selectedBehaviorMetaData);
	ui->behaviorNameValueLabel->setText(selectedBehaviorMetaData.m_sBehaviorName);
	ui->idValueLabel->setText(QString::number(selectedBehaviorMetaData.m_ulBehaviorId));
	ui->motorCountValueLabel->setText(QString::number(selectedBehavior.m_mapMotorWaypoints.count()));

	QString description;
	for(u_int32_t iterator : selectedBehavior.m_mapMotorWaypoints.keys())
	{
		description.append(QString("MOTOR ID %1 WAYPOINT COUNT %2\n").arg(iterator).arg(selectedBehavior.m_mapMotorWaypoints.value(iterator).count()));
	}
	ui->descriptionValueLabel->setText(description);
}
