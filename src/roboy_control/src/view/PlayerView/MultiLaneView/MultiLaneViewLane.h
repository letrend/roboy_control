#ifndef MULTILANEVIEWLANE_H
#define MULTILANEVIEWLANE_H

#include <QFrame>
#include <QIcon>
#include <QList>

#include "MultiLaneViewItem.h"

class MultiLaneViewLane : public QFrame
{
    Q_OBJECT

public:
    enum multiLaneViewScaleFactor {
        millisecond = 1,
        centisecond = 10,
        decisecond  = 100,
        second      = 1000
    };

    typedef enum multiLaneViewScaleFactor scaleFactor;

    explicit MultiLaneViewLane(quint32 laneHeight, quint64 minimumLaneWidth, scaleFactor viewScaleFactor, qint64 laneID, QWidget *parent = 0);
    ~MultiLaneViewLane();
    void setScaleFactor(scaleFactor factor);
    void itemInsertedHandler    (qint32 index, QString name, QIcon icon, qint64 timestamp, quint64 duration64, quint64 motorCount);
    void itemRemovedHandler     (qint32 index);
    void paintEvent(QPaintEvent *event);
    qint64 getLaneID();

public slots:
    void showMultiLaneViewLaneMenu(const QPoint &pos);
    void removeItemWithTimestamp(qint64 timestamp);

signals:
        void removeItemWithTimestampAndLaneID(qint64 timestamp, qint64 laneID);
        void removeLaneWithLaneID(qint64 laneID);

private:
    QList<MultiLaneViewItem *> items;
    scaleFactor viewScaleFactor = scaleFactor::millisecond;
    qint64 laneID;

private slots:
    void removeLaneHandler();
};

#endif // MULTILANEVIEWLANE_H
