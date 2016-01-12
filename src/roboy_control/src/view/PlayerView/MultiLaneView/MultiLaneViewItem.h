#ifndef MULTILANEVIEWITEM_H
#define MULTILANEVIEWITEM_H

#include <QIcon>
#include <QLabel>
#include <QWidget>

class MultiLaneViewItem : public QWidget
{
    Q_OBJECT

public:
    explicit MultiLaneViewItem(QString name, quint64 motorCount, QIcon icon, qint64 timestamp, QWidget *parent = 0);
    qint64  getTimestamp();

public slots:
        void showMultiLaneViewItemMenu(const QPoint& pos);
signals:
        void removeItemWithTimestamp(qint64 timestamp);
private:
    qint64 timestamp;
    QLabel *iconLabel;
    QLabel *behaviorNameLabel;

private slots:
    void removeItemHandler();
};

#endif // MULTILANEVIEWITEM_H
