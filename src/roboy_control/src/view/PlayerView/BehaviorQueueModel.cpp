#include "BehaviorQueueModel.h"

BehaviorQueueModel::BehaviorQueueModel(QObject *parent) : QAbstractListModel(parent)
{
  
}

int BehaviorQueueModel::rowCount(const QModelIndex &) const
{
  return this->behaviorList.size();
}

QVariant BehaviorQueueModel::data(const QModelIndex &index, int role) const
{
	if(index.isValid())
	{
		if (index.column() == 0 && role == Qt::DecorationRole) return QColor("blue");
  		if (index.column() == 0 && role == Qt::EditRole)       return this->behaviorList[index.row()].m_sBehaviorName;
  		if (index.column() == 0 && role == Qt::DisplayRole)    return this->behaviorList[index.row()].m_sBehaviorName;
	}

  	return QVariant();
}

Qt::ItemFlags BehaviorQueueModel::flags(const QModelIndex &index) const
{
  return QAbstractItemModel::flags(index)
    | Qt::ItemIsEditable
    | Qt::ItemIsDragEnabled
    | Qt::ItemIsDropEnabled;
}

void BehaviorQueueModel::addBehaviorMetaData(RoboyBehaviorMetadata behavior)
{
	this->behaviorList.append(behavior);
	QModelIndex topLeft;
	QModelIndex bottomRight;
	emit dataChanged(topLeft, bottomRight);
}

QList<RoboyBehaviorMetadata> BehaviorQueueModel::getBehaviorMetaDataList() const
{
	return this->behaviorList;
}

void BehaviorQueueModel::removeBehaviorMetaData(int index)
{
	if(index >= 0 && index < this->behaviorList.count())
	{		
		this->behaviorList.removeAt(index);
		QModelIndex topLeft;
		QModelIndex bottomRight;
		emit dataChanged(topLeft, bottomRight);
	}
}
