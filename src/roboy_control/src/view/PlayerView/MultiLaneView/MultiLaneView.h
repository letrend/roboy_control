#ifndef MULTILANEVIEW_H
#define MULTILANEVIEW_H

#include <QWidget>

#include "IMultiLaneViewModel.h"

class MultiLaneView : public QWidget
{
    Q_OBJECT

public:
    explicit MultiLaneView(QWidget *parent = 0);
    void setModel(IMultiLaneViewModel *model);

public slots:
    void dataChangedHandler();

private:
    IMultiLaneViewModel *model;

};

#endif // MULTILANEVIEW_H
