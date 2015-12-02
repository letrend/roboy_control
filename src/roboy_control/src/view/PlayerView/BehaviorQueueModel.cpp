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

void BehaviorQueueModel::addBehaviorMetaData(RoboyBehaviorMetadata behavior)
{
	this->behaviorList.append(behavior);
	QModelIndex topLeft;
	QModelIndex bottomRight;
	emit dataChanged(topLeft, bottomRight);
}

void BehaviorQueueModel::removeBehaviorMetaData(int index)
{
	if(index > 0 && index < this->behaviorList.count()-1)
	{		
		this->behaviorList.removeAt(index);
		QModelIndex topLeft;
		QModelIndex bottomRight;
		emit dataChanged(topLeft, bottomRight);
	}
}
