#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(IModelService *modelService, ViewController * pViewController, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow) 
{
    modelService->subscribe(this);
    this->ui->setupUi(this);
    this->mainTabWidget = new QTabWidget();
    this->playerView = new PlayerView(modelService, pViewController);
    this->recorderView = new RecorderView(modelService);
    this->editorView = new EditorView(modelService);
    this->setContentsMargins(9,9,9,9);
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
    qDebug() << "MainViewController: Notified on data changed";
    this->playerView->notify();
	this->recorderView->notify();
	this->editorView->notify();
}

