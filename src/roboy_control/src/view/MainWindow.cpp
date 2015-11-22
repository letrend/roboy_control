#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{
    QLabel * label = new QLabel("Hello World - Qt GUI in ROS");
    setCentralWidget(label);
}
