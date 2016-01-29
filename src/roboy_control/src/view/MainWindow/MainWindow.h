#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTabWidget>

#include "DataTypes.h"
#include "../EditorView/EditorView.h"
#include "../PlayerView/PlayerView.h"
#include "../RecorderView/RecorderView.h"

#include "../../interfaces/IObserver.h"
#include "../../model/IModelService.h"

class ViewController;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow, public IObserver
{
    Q_OBJECT

public:
    explicit MainWindow(IModelService *modelService, ViewController * pViewController, QWidget *parent = 0);
    ~MainWindow();

    void notify();
    RoboyBehaviorMetaplan fromMainWindow_getCurrentRoboyPlan();

private:
    ViewController * m_pViewController;

    Ui::MainWindow *ui;
    QTabWidget *mainTabWidget;
    EditorView *editorView;
    PlayerView *playerView;
    RecorderView *recorderView;
};

#endif // MAINWINDOW_H
