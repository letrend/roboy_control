#include "PlayerView.h"
#include "ui_PlayerView.h"

#include <QDebug>

PlayerView::PlayerView(IModelService *modelService, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PlayerView)
{
    this->ui->setupUi(this);
    this->modelService = modelService;
    this->setupConnections();
    this->updateBehaviorList();
}

PlayerView::~PlayerView()
{
    delete this->ui;
}

void PlayerView::notify() 
{
	this->updateBehaviorList();
}

void PlayerView::updateBehaviorList()
{
	this->behaviorList = this->modelService->getBehaviorList();
	this->ui->behaviorListWidget->clear();
	for(RoboyBehaviorMetadata currentBehavior : this->behaviorList)
	{
		this->ui->behaviorListWidget->addItem(currentBehavior.m_sBehaviorName);
	}
}

/* ui connections */

void PlayerView::setupConnections()
{
	QObject::connect(ui->playButton, SIGNAL(clicked()), this, SLOT(playButtonClicked()));
	QObject::connect(ui->pauseButton, SIGNAL(clicked()), this, SLOT(pauseButtonClicked()));
	QObject::connect(ui->stopButton, SIGNAL(clicked()), this, SLOT(stopButtonClicked()));
	QObject::connect(ui->skipButton, SIGNAL(clicked()), this, SLOT(skipButtonClicked()));
	QObject::connect(ui->behaviorListWidget, SIGNAL(currentRowChanged(int)), this, SLOT(behaviorListWidgetCurrentRowChanged(int)));
}

void PlayerView::playButtonClicked()
{
	
}

void PlayerView::pauseButtonClicked()
{
	
}

void PlayerView::stopButtonClicked()
{
	
}

void PlayerView::skipButtonClicked()
{
	
}

void PlayerView::behaviorListWidgetCurrentRowChanged(int row)
{
	RoboyBehavior selectedBehavior = this->modelService->getBehavior(this->behaviorList[row]);
	ui->behaviorNameValueLabel->setText(selectedBehavior.m_metadata.m_sBehaviorName);
	ui->idValueLabel->setText(QString::number(selectedBehavior.m_metadata.m_ulBehaviorId));
	ui->motorCountValueLabel->setText(QString::number(selectedBehavior.m_mapMotorWaypoints.count()));

	QString description;
	for(u_int32_t iterator : selectedBehavior.m_mapMotorWaypoints.keys())
	{
		description.append(QString("MOTOR ID %1 WAYPOINT COUNT %2\n").arg(iterator).arg(selectedBehavior.m_mapMotorWaypoints.value(iterator).count()));
	}
	ui->descriptionValueLabel->setText(description);
}