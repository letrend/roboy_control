#include "BehaviorListModel.h"

/* constructor */

/**
 * @brief BehaviorListModel::BehaviorListModel constructor
 * @param modelService modelService from which the RoboyBehaviors are retrieved
 * @param parent non mandatory parent for the BehaviorListModel
 */
BehaviorListModel::BehaviorListModel(IModelService *pModelService, QObject *parent) : QAbstractListModel(parent) {
    m_pModelService = pModelService;
    updateBehaviorList();
}

/**
 * @brief getter method for a behavior at a given index
 * @param index index of the behavior that should be retrieved
 * @return the behavior for given index
 **/
RoboyBehavior BehaviorListModel::behaviorAt(qint32 index) {
	return m_BehaviorList[index];
}

/* methods implemented from QAbstractListModel */

/**
 * @brief BehaviorListModel::data method to retrieve data for a specific index and role
 * @param index index for which the data should be retrieved
 * @param role role for which the data should be retrieved 
 */
QVariant BehaviorListModel::data(const QModelIndex & index, int role) const {
	switch(role) {
		case BehaviorListRoles::IDRole: {
			return m_BehaviorList[index.row()].m_metadata.m_ulBehaviorId;
		}
		break;
		case BehaviorListRoles::TitleRole: {
			return m_BehaviorList[index.row()].m_metadata.m_sBehaviorName;
		}
		break;
		case BehaviorListRoles::IconNameRole: {
			return "action/accessibility";
		}
		break;
		case BehaviorListRoles::DurationRole: {
			return m_BehaviorList[index.row()].getDuration();
		}
		break;
		case BehaviorListRoles::MotorCountRole: {
			return m_BehaviorList[index.row()].m_mapMotorTrajectory.count();
		}
		break;
		case BehaviorListRoles::MotorInfoRole: {
			QList<QString> motorInfo;
			for(u_int32_t iterator : m_BehaviorList[index.row()].m_mapMotorTrajectory.keys())
				motorInfo.append(QString("motor %1: waypoint count %2").arg(iterator).arg(m_BehaviorList[index.row()].m_mapMotorTrajectory.value(iterator).m_listWaypoints.count()));
			QVariant motorInfoVariant(motorInfo);
			return motorInfoVariant;
		}
		break;
	}
}

/**
 * @brief BehaviorListModel::roleNames method to retrieve a HashTable of rolenames
 */
QHash<int, QByteArray> BehaviorListModel::roleNames() const {
    QHash<int, QByteArray> roles;
    roles[IDRole] 			= "id";
    roles[TitleRole]		= "title";
    roles[IconNameRole]		= "iconPath";
    roles[DurationRole]     = "duration";
    roles[MotorCountRole] 	= "motorCount";
    roles[MotorInfoRole] 	= "motorInfo";
    return roles;
}

/**
 * @brief BehaviorListModel::rowCount method to retrieve the current number of rows in the model
 * @param parent index of the parent item
 */
int BehaviorListModel::rowCount (const QModelIndex & parent) const {
	return m_BehaviorList.count();
}

/* methods implemented from IObserver interface */

/**
 * @brief BehaviorListModel::notify method to notify about data changes implemented from IObserver interface
 */
void BehaviorListModel::notify() {
    updateBehaviorList();
}

/* private methods */

/**
 * @brief BehaviorListModel::updateBehaviorList method for triggering a update of the behavior list after a change in the data
 */
void BehaviorListModel::updateBehaviorList() {
    m_BehaviorList = QList<RoboyBehavior>();
    QList<RoboyBehaviorMetadata> metaDataList = m_pModelService->getBehaviorList();
    for (RoboyBehaviorMetadata metaData : metaDataList) {
        m_BehaviorList.append(m_pModelService->retrieveRoboyBehavior(metaData));
    }
}