#include "MultiLaneView.h"

MultiLaneView::MultiLaneView(QWidget *parent) :
    QWidget(parent)
{

}

void MultiLaneView::setModel(IMultiLaneViewModel *model)
{
    this->model = model;
}

void MultiLaneView::dataChangedHandler()
{

}
