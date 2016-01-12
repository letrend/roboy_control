#include <QAction>
#include <QGraphicsDropShadowEffect>
#include <QBoxLayout>
#include <QMenu>
#include "LogDefines.h"
#include "GUIColors.h"
#include "MultiLaneViewItem.h"

/**
 * @brief MultiLaneViewItem::MultiLaneViewItem constructor
 * @param name name of the MultiLaneViewItem
 * @param icon icon of the MultiLaneViewItem
 * @param parent non mandatory parent of the item
 */
MultiLaneViewItem::MultiLaneViewItem(QString name, quint64 motorCount, QIcon icon, qint64 timestamp, QWidget *parent) : QWidget(parent)
{
    this->timestamp = timestamp;

    QBoxLayout *layout = new QBoxLayout(QBoxLayout::Direction::LeftToRight);
    this->iconLabel = new QLabel();
    this->iconLabel->setPixmap(icon.pixmap(icon.availableSizes().first()));
    layout->addWidget(this->iconLabel);

    this->behaviorNameLabel = new QLabel();
    this->behaviorNameLabel->setTextFormat(Qt::RichText);
    this->behaviorNameLabel->setText("<b>" + name + "</b>" + QString("<br><font size =\"8px\">motor count: %1</font>").arg(motorCount));
    QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    this->behaviorNameLabel->setSizePolicy(sizePolicy);
    this->behaviorNameLabel->setStyleSheet("QLabel { background-color : transparent; color : white; }");
    layout->addWidget(this->behaviorNameLabel);
    this->setLayout(layout);

    QPalette itemPalette(this->palette());
    itemPalette.setColor(QPalette::Background, normalAccentColor);
    this->setAutoFillBackground(true);
    this->setPalette(itemPalette);

    QGraphicsDropShadowEffect *shadow = new QGraphicsDropShadowEffect();
    shadow->setColor(shadowColor);
    shadow->setBlurRadius(10);
    shadow->setXOffset(0);
    shadow->setYOffset(0);
    this->setGraphicsEffect(shadow);

    this->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(this, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(showMultiLaneViewItemMenu(const QPoint&)));
}

/**
 * @brief MultiLaneViewItem::showMultiLaneViewItemMenu method to handle the invokation of a context menu on the MultiLaneViewItem
 * @param pos position at which the context menu is invoked
 */
void MultiLaneViewItem::showMultiLaneViewItemMenu(const QPoint &pos)
{
    QMenu behaviorListItemMenu;
    QAction removeItemAction(QIcon(":/delete-img-dark.png"), "remove behavior", NULL);
    connect(&removeItemAction, SIGNAL(triggered()), this, SLOT(removeItemHandler()));
    removeItemAction.setIconVisibleInMenu(true);
    behaviorListItemMenu.addAction(&removeItemAction);
    QPoint globalPos = this->mapToGlobal(pos);
    QAction *selectedItem = behaviorListItemMenu.exec(globalPos);
}

/**
 * @brief MultiLaneViewItem::getTimestamp method for retrieving the timestamp of the MultiLaneViewItem
 * @return the timestamp of the MultiLaneViewItem
 */
qint64 MultiLaneViewItem::getTimestamp()
{
    return this->timestamp;
}

/**
 * @brief MultiLaneViewItem::removeItemHandler handler function for the remove item action in the items contextmenu
 */
void MultiLaneViewItem::removeItemHandler()
{
    emit removeItemWithTimestamp(this->timestamp);
}
