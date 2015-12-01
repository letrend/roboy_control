#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(IModelService *modelService, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow) 
{
    this->ui->setupUi(this);
    this->mainTabWidget = new QTabWidget();
    this->playerView = new PlayerView(modelService);
    this->recorderView = new RecorderView(modelService);
    this->editorView = new EditorView(modelService);
    this->setCentralWidget(mainTabWidget);
    this->mainTabWidget->addTab(playerView, "player");
    this->mainTabWidget->addTab(recorderView, "recorder");
    this->mainTabWidget->addTab(editorView, "editor");

}

MainWindow::~MainWindow() 
{
	delete this->playerView;
	delete this->recorderView;
	delete this->editorView;
	delete this->mainTabWidget;
    delete this->ui;
}

void MainWindow::notify() 
{
	this->playerView->notify();
	this->recorderView->notify();
	this->editorView->notify();
}

