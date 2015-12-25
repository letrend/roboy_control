#include <QDebug>
#include <QGridLayout>
#include <QIcon>
#include <QLabel>
#include <QVariant>
#include <QVBoxLayout>

#include "MultiLaneView.h"

#define LANE_INDENT 12

/* public methods */

/**
 * @brief MultiLaneView::MultiLaneView constructor
 * @param parent non mandatory parent of the MultiLaneView
 */
MultiLaneView::MultiLaneView(QWidget *parent)
{
    Q_UNUSED(parent);

    this->laneScrollArea = new QScrollArea();
    this->laneBackground = new QWidget();

    QGridLayout *multiLaneViewLayout    = new QGridLayout();
    QGridLayout *laneScrollAreaLayout   = new QGridLayout();

    laneBackground->setLayout(laneScrollAreaLayout);

    multiLaneViewLayout->addWidget(laneScrollArea);

    this->setLayout(multiLaneViewLayout);

    laneScrollArea->setWidget(this->laneBackground);
}

/**
 * @brief MultiLaneView::~MultiLaneView destructor
 */
MultiLaneView::~MultiLaneView()
{
    for (MultiLaneViewLane *lane : this->lanes) {
        delete lane;
    }

    this->lanes.clear();

    delete this->laneBackground;
    delete this->laneScrollArea;
}

/**
 * @brief MultiLaneView::setModel
 * @param model
 */
void MultiLaneView::setModel(IMultiLaneViewModel *model)
{
    this->model = model;
    this->setupConnections();

    this->model->initializeWidget();
}

/**
 * @brief MultiLaneView::setScaleFactor method for setting a scale factor for the multi lane widget
 */
void MultiLaneView::setScaleFactor(scaleFactor factor)
{
    /* set new scale factor */
    this->viewScaleFactor = factor;

    /*dispose old view hierachy */

    for (MultiLaneViewLane *lane : this->lanes) {
        delete lane;
    }

    this->lanes.clear();

    /*initialize widget with new scale factor */
    this->model->initializeWidget();
}

/* public slots */

/**
 * @brief MultiLaneView::laneInsertedHandler slot for handling the insertion of a new lane
 * @param index index where the lane should be inserted
 */
void MultiLaneView::laneInsertedHandler(qint32 index)
{
    MultiLaneViewLane *lane = new MultiLaneViewLane(this->viewScaleFactor);
    this->lanes.insert(index, lane);
    QGridLayout *layout = qobject_cast<QGridLayout *>(this->laneBackground->layout());
    layout->addWidget(lane, index, 0);
    this->laneBackground->adjustSize();
}

/**
 * @brief MultiLaneView::laneRemovedHandler slot for handling the removal of a lane
 * @param index index of the lane that should be removed
 */
void MultiLaneView::laneRemovedHandler(qint32 index)
{
    MultiLaneViewLane *lane = this->lanes.at(index);
    this->lanes.removeAt(index);
    delete lane;
    this->laneBackground->adjustSize();
}

/**
 * @brief MultiLaneView::itemInsertedHandler slot for handling the insertion of a new behavior item
 * @param laneIndex index of the lane the behavior is inserted into
 * @param itemIndex index where the item is inserted
 */
void MultiLaneView::itemInsertedHandler(qint32 laneIndex, qint32 itemIndex)
{
    MultiLaneViewLane *lane = this->lanes.at(laneIndex);

    QString bName       = this->model->data(laneIndex, itemIndex, Qt::DisplayRole       ).value<QString>();
    bName = "Behavior";
    QIcon   bIcon       = this->model->data(laneIndex, itemIndex, Qt::DecorationRole    ).value<QIcon  >();
    bIcon = QIcon("/Users/matthiaslehner/Dropbox/MultiLaneWidget/MultiLaneWidget/behavior-img.png");
    quint64 bTimestamp  = this->model->data(laneIndex, itemIndex, Qt::UserRole          ).value<quint64>();
    quint64 bDuration   = this->model->data(laneIndex, itemIndex, Qt::UserRole + 1      ).value<quint64>();
    quint64 bMotorCount = this->model->data(laneIndex, itemIndex, Qt::UserRole + 2      ).value<quint64>();
    bDuration = 2000;

    quint64 maxSize = ((bTimestamp + bDuration + (0x64*this->viewScaleFactor))/this->viewScaleFactor) + (LANE_INDENT << 1);
    this->laneBackground->setFixedWidth(maxSize);

    lane->itemInsertedHandler(itemIndex, bName, bIcon, bTimestamp, bDuration, bMotorCount);
}

/**
 * @brief MultiLaneView::itemRemovedHandler slot for handling the removal of a behavior i
 * @param laneIndex index of the lane the behavior is removed from
 * @param itemIndex index of the behavior item that should be removed
 */
void MultiLaneView::itemRemovedHandler(qint32 laneIndex, qint32 itemIndex)
{
    MultiLaneViewLane *lane = this->lanes.at(laneIndex);
    lane->itemRemovedHandler(itemIndex);

    quint64 maxSize = 0;
    for (int i = 0; i < this->model->laneCount(); i++) {
        qint32 itemCount  = this->model->itemCount(i);
        quint64 timestamp = this->model->data(i, itemCount-1, Qt::UserRole    ).value<quint64>();
        quint64 duration  = this->model->data(i, itemCount-1, Qt::UserRole + 1).value<quint64>();
        quint64 newMaxSize = ((timestamp + duration + (0x64*this->viewScaleFactor))/this->viewScaleFactor) + (LANE_INDENT << 1);
        if (newMaxSize> maxSize) {
            maxSize = newMaxSize;
        }
    }

    this->laneBackground->setFixedWidth(maxSize);
}

/* private methods */

/**
 * @brief MultiLaneView::setupConnections method for setting up the connections to the model
 */
void MultiLaneView::setupConnections()
{
        QObject::connect(this->model, SIGNAL(laneInserted (qint32)), this, SLOT(laneInsertedHandler(qint32)));
        QObject::connect(this->model, SIGNAL(laneRemoved  (qint32)), this, SLOT(laneRemovedHandler(qint32)));
        QObject::connect(this->model, SIGNAL(itemInserted (qint32, qint32)), this, SLOT(itemInsertedHandler(qint32, qint32)));
        QObject::connect(this->model, SIGNAL(itemRemoved  (qint32, qint32)), this, SLOT(itemRemovedHandler(qint32, qint32)));
}
