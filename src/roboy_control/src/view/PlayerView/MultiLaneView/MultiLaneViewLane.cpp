#include <QAction>
#include <QGraphicsDropShadowEffect>
#include <QLabel>
#include <QMenu>
#include <QPainter>
#include <QPalette>
#include <QStaticText>
#include <QVBoxLayout>

#include "GUIColors.h"
#include "MultiLaneView.h"
#include "MultiLaneViewItem.h"
#include "MultiLaneViewLane.h"

#define ITEM_INDENT 5

/**
 * @brief MultiLaneViewLane::MultiLaneViewLane constructor
 * @param parent the non mandatory parent of the widget
 */
MultiLaneViewLane::MultiLaneViewLane(quint32 laneHeight, quint64 minimumLaneWidth, scaleFactor viewScaleFactor, qint64 laneID, QWidget *parent)
{
    Q_UNUSED(parent);

    this->viewScaleFactor = viewScaleFactor;
    this->laneID = laneID;

    this->setFixedHeight(laneHeight);
    QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    this->setSizePolicy(sizePolicy);
    QPalette lanePalette(this->palette());
    lanePalette.setColor(QPalette::Background, normalBackgroundColor);
    this->setAutoFillBackground(true);
    this->setPalette(lanePalette);

    QGraphicsDropShadowEffect *shadow = new QGraphicsDropShadowEffect();
    shadow->setColor(shadowColor);
    shadow->setBlurRadius(10);
    shadow->setXOffset(0);
    shadow->setYOffset(0);
    this->setGraphicsEffect(shadow);

    this->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(this, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(showMultiLaneViewLaneMenu(const QPoint&)));
}

MultiLaneViewLane::~MultiLaneViewLane()
{
    for (MultiLaneViewItem *item : this->items) {
        delete item;
    }
}

/**
 * @brief MultiLaneViewLane::setScaleFactor method for setting a scale factor for the multi lane widget
 */
void MultiLaneViewLane::setScaleFactor(scaleFactor factor)
{
    /* set new scale factor */
    this->viewScaleFactor = factor;
}

/**
 * @brief MultiLaneViewLane::itemInsertedHandler method for handling the insertion of a new behavior item
 * @param name name of the behavior item
 * @param icon icon for the behavior item
 * @param timeStamp timestamp of the behavior item
 * @param duration duration of the behavior item
 */
void MultiLaneViewLane::itemInsertedHandler(qint32 index, QString name, QIcon icon, qint64 timestamp, quint64 duration, quint64 motorCount)
{
    MultiLaneViewItem *item = new MultiLaneViewItem(name, motorCount, icon, timestamp);
    item->setGeometry((timestamp/this->viewScaleFactor), 20 + ITEM_INDENT, (duration/this->viewScaleFactor), this->height()-20-(ITEM_INDENT << 1));
    item->setParent(this);
    connect(item, SIGNAL(removeItemWithTimestamp(qint64)), this, SLOT(removeItemWithTimestamp(qint64)));
    this->items.insert(index, item);
    item->show();
}

/**
 * @brief MultiLaneViewLane::itemRemovedHandler method for handling the removal of a behavior item
 * @param timestamp timestamp of the behavior that should be removed
 */
void MultiLaneViewLane::itemRemovedHandler(qint32 index)
{
    MultiLaneViewItem * item = this->items.at(index);
    this->items.removeAt(index);
    delete item;
}

/**
 * @brief MultiLaneViewLane::paintEvent paint event for the MultiLaneViewLane
 * @param event event on which the view is painted
 */
void MultiLaneViewLane::paintEvent(QPaintEvent *event)
{
    QWidget::paintEvent(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(0,0, this->width(), 20,  lightBackgroundColor);
    QPen finePen(Qt::lightGray, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    QPen boldPen(Qt::white, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    painter.setPen(finePen);
    QFont font = QFont();
    font.setPixelSize(8);
    painter.setFont(font);
    for (int x=10; x < this->width(); x += 10) {
        if (x % 100 == 0) {
            painter.setPen(boldPen);
            painter.drawLine(QLine(x, 10, x, 20));
            painter.drawText(QRect(x-50, 0, 100, 10), Qt::AlignCenter, QString("%1").arg(x));
        } else if (x % 50 == 0) {
            painter.setPen(boldPen);
            painter.drawLine(QLine(x, 13, x, 20));
        } else {
            painter.setPen(finePen);
            painter.drawLine(QLine(x, 15, x, 20));
        }
    }
}

/**
 * @brief MultiLaneViewLane::getLaneID getter method for lane id
 * @return the id of the current lane
 */
qint64 MultiLaneViewLane::getLaneID()
{
    return this->laneID;
}

/**
 * @brief MultiLaneViewLane::showMultiLaneViewLaneMenu method to handle the invokation of a context menu on the MultiLaneViewLane
 * @param pos position at which the context menu is invoked
 */
void MultiLaneViewLane::showMultiLaneViewLaneMenu(const QPoint &pos)
{
    QMenu behaviorListItemMenu;
    QAction removeLaneAction(QIcon(":/delete-img-dark.png"), "remove lane", NULL);
    connect(&removeLaneAction, SIGNAL(triggered()), this, SLOT(removeLaneHandler()));
    removeLaneAction.setIconVisibleInMenu(true);
    behaviorListItemMenu.addAction(&removeLaneAction);
    QPoint globalPos = this->mapToGlobal(pos);
    QAction *selectedItem = behaviorListItemMenu.exec(globalPos);
}

/**
 * @brief MultiLaneViewLane::removeItemWithTimestamp handler method to remove a item from the current lane
 * @param timestamp timestamp of the item
 */
void MultiLaneViewLane::removeItemWithTimestamp(qint64 timestamp)
{
    emit removeItemWithTimestampAndLaneID(timestamp, this->laneID);
}

/**
 * @brief MultiLaneViewLane::removeLaneHandler handler function for the remove lane action in the lanes contextmenu
 */
void MultiLaneViewLane::removeLaneHandler()
{
    emit(removeLaneWithLaneID(this->laneID));
}
