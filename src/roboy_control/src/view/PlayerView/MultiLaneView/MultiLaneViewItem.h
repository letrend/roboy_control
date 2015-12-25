#ifndef MULTILANEVIEWITEM_H
#define MULTILANEVIEWITEM_H

#include <QIcon>
#include <QLabel>
#include <QWidget>

class MultiLaneViewItem : public QWidget
{
    Q_OBJECT

public:
    explicit MultiLaneViewItem(QString name, quint64 motorCount, QIcon icon, QWidget *parent = 0);
    quint64  getTimestamp();
private:
    quint64 timestamp;
    QLabel *iconLabel;
    QLabel *behaviorNameLabel;

};

#endif // MULTILANEVIEWITEM_H
