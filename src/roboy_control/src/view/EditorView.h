#ifndef EDITORVIEW_H
#define EDITORVIEW_H

#include <QWidget>

#include "../DataTypes.h"
#include "../interfaces/IObserver.h"
#include "../model/IModelService.h"

namespace Ui {
class EditorView;
}

class EditorView : public QWidget, public IObserver
{
    Q_OBJECT

public:
    explicit EditorView(IModelService *modelService, QWidget *parent = 0);
    ~EditorView();

    void notify();

private:
    Ui::EditorView *ui;
};

#endif // EDITORVIEW_H
