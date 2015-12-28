#include "BehaviorListModel.h"

/*constructor*/

/**
 * @brief BehaviorListModel::BehaviorListModel constructor
 * @param modelService modelService from which the RoboyBehaviors are retrieved
 * @param parent non mandatory parent for the MainWindow
 */
BehaviorListModel::BehaviorListModel(IModelService *modelService, QObject *parent) : QAbstractListModel(parent)
{
	this->modelService = modelService;
	this->updateBehaviorList();
}

/* methods implemented from IObserver interface */

/**
 * @brief BehaviorListModel::notify method to notify about data changes implemented from IObserver interface
 */
void BehaviorListModel::notify()
{
	this->updateBehaviorList();
}

/**
 * @brief BehaviorListModel::data implemented method from QAbstracListModel for retrieving data for the listView
 * @param index index of selected list row
 * @param role role role of the data that you have to return
 * @return QVariant representing the data for specified index and role
 */
QVariant BehaviorListModel::data(const QModelIndex &index, int role) const
{
	if(index.isValid())
	{
		if (index.column() == 0 && role == Qt::DecorationRole) return QIcon(":/behavior-img.png");
  		if (index.column() == 0 && role == Qt::EditRole)       return this->behaviorList[index.row()].m_sBehaviorName;
  		if (index.column() == 0 && role == Qt::DisplayRole)    return this->behaviorList[index.row()].m_sBehaviorName;
	}

  	return QVariant();
}

/**
 * @brief BehaviorListModel::rowCount implemented method from QAbstractListModel for retrieving the number of rows for the listView
 * @return number of rows for the listView
 */
int BehaviorListModel::rowCount(const QModelIndex &) const
{
  return this->behaviorList.count();
}

RoboyBehaviorMetadata  BehaviorListModel::getBehaviorMetaData(int index) const
{
	Q_ASSERT_X((index >= 0) && (index < this->behaviorList.count()), "BehaviorListModel::getBehaviorData", "index out of range");
	return this->behaviorList[index];
}

// private methods

/**
 * @brief BehaviorListModel::updateBehaviorList method for triggering a update of the behavior list after a change in the data
 */
void BehaviorListModel::updateBehaviorList()
{
	this->behaviorList = this->modelService->getBehaviorList();
}
