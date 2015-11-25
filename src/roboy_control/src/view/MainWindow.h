#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QModelIndex>

#include "../DataTypes.h"
#include "../model/IModelService.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent, IModelService *modelService);
    ~MainWindow();

public slots:
	
    void playButtonClicked();
    void pauseButtonClicked();
    void stopButtonClicked();
    void addToTimelineButtonClicked();
    void behaviorListViewItemClicked(QModelIndex index);

private:
    Ui::MainWindow *ui; 
    IModelService * modelService; 
    void setupConnections();
};

#endif // MAINWINDOW_H