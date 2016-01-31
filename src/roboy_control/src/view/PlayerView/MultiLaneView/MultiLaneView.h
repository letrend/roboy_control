#ifndef MULTILANEVIEW_H
#define MULTILANEVIEW_H

#include <QScrollArea>

#include "IMultiLaneViewModel.h"
#include "MultiLaneViewLane.h"

class MultiLaneView : public QWidget
{
    Q_OBJECT

public:


    typedef MultiLaneViewLane::scaleFactor scaleFactor;

    explicit MultiLaneView(QWidget *parent = 0);
    ~MultiLaneView();
    void setModel(IMultiLaneViewModel *model);
    void setScaleFactor(scaleFactor factor);

public slots:
    void laneInsertedHandler    (qint32 index);
    void laneRemovedHandler     (qint32 index);
    void itemInsertedHandler    (qint32 laneIndex, qint32 itemIndex);
    void itemRemovedHandler     (qint32 laneIndex, qint32 itemIndex);

    void removeItemWithPointer(MultiLaneViewItem * item);
    void removeLane();

private:
    IMultiLaneViewModel *model;
    QScrollArea         *laneScrollArea;
    QWidget             *laneBackground;
    QList<MultiLaneViewLane *> lanes;
    scaleFactor viewScaleFactor = scaleFactor::millisecond;
    qint64 nextAvailableLaneID = 0;

    void setupConnections();
};

#endif // MULTILANEVIEW_H
