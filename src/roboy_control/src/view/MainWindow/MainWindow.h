#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTabWidget>

#include "../EditorView/EditorView.h"
#include "../PlayerView/PlayerView.h"
#include "../RecorderView/RecorderView.h"

#include "../../interfaces/IObserver.h"
#include "../../model/IModelService.h"



namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow, public IObserver
{
    Q_OBJECT

public:
    explicit MainWindow(IModelService *modelService, QWidget *parent = 0);
    ~MainWindow();

    void notify();

private:
    Ui::MainWindow *ui;
    QTabWidget *mainTabWidget;
    EditorView *editorView;
    PlayerView *playerView;
    RecorderView *recorderView;
};

#endif // MAINWINDOW_H
