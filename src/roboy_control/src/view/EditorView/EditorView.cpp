#include "EditorView.h"
#include "ui_EditorView.h"

EditorView::EditorView(IModelService *modelService, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::EditorView)
{
    this->ui->setupUi(this);
}

EditorView::~EditorView()
{
    delete this->ui;
}

void EditorView::notify()
{

}