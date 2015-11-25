#include "RoboyBehaviorModel.h"

RoboyBehaviorModel::RoboyBehaviorModel(QObject *parent, IModelService *modelService)
    :QAbstractListModel(parent)
{
	this->behaviorList = modelService->getBehaviorList();
}

int RoboyBehaviorModel::rowCount(const QModelIndex& parent) const {
    return this->behaviorList.count();
}

QVariant RoboyBehaviorModel::data(const QModelIndex& index, int role) const {

    if (!index.isValid()) return QVariant();

    if (role == Qt::DisplayRole) {
        return QVariant(this->behaviorList.at(index.row()).m_sBehaviorName);
    } else {
        return QVariant();
    }

}
