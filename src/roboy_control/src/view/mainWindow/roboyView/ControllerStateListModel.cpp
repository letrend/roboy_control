#include <QHash>
#include <QModelIndex>

#include "LogDefines.h"
#include "ControllerStateListModel.h"

// constructor

/**
 * @brief ControllerStateListModel::ControllerStateListModel constructor
 * @param parent non mandatory parent for the ControllerStateListModel
 */
ControllerStateListModel::ControllerStateListModel(QObject *parent) : QAbstractListModel(parent) {

}

/**
 * @brief ControllerStateListModel::controllerStateChanged slot to notify the gui about a when the state of a motor changed
 * @param motorId id of the motor of which the state changed
 * @param state state of the motor
 */
void ControllerStateListModel::controllerStatusUpdated(qint32 motorId, ControllerState state) {
	beginResetModel();
	m_states[motorId] = QPair<qint32, ControllerState>(motorId, state);
	endResetModel();
}

/**
 * @brief ControllerStateListModel::dataPoolReset method for handling a reset of the data pool
 */
void ControllerStateListModel::dataPoolReset() {
	beginResetModel();
	m_states.clear();
	m_states = QMap<qint32, QPair<qint32, ControllerState>>();
	endResetModel();
}

// methods implemented from QAbstractListModel

/**
 * @brief ControllerStateListModel::data method to retrieve data for a specific index and role
 * @param index index for which the data should be retrieved
 * @param role role for which the data should be retrieved 
 */
QVariant ControllerStateListModel::data(const QModelIndex & index, int role) const {
	QList<QPair<qint32, ControllerState>> states_list = m_states.values();
	switch(role) {
		case BehaviorListRoles::MotorIDRole: {
			return states_list[index.row()].first;
		}
		case BehaviorListRoles::MotorStateRole: {
			switch(states_list[index.row()].second) {
				case 0: // UNDEFINED
					return "UNDEFINED";
				case 1: // INITIALIZED
					return "INITIALIZED";
				case 2: // PREPROCESS_TRAJECTORY
					return "PREPROCESS_TRAJECTORY";
				case 3: // TRAJECTORY_READY
					return "TRAJECTORY_READY";
				case 4: // TRAJECTORY_FAILED
					return "TRAJECTORY_FAILED";
				case 5: // TRAJECTORY_PLAYING
					return "TRAJECTORY_PLAYING";
				case 6: // TRAJECTORY_PAUSED
					return "TRAJECTORY_PAUSED";
				case 7: // TRAJECTORY_DONE
					return "TRAJECTORY_DONE";
				case 8: // INITIALIZE_ERROR
					return "INITIALIZE_ERROR";
				default:
					return "";
			}
		}
		case BehaviorListRoles::IconNameRole: {
			return "av/album";
		}
	}
	return QVariant();
}

/**
 * @brief ControllerStateListModel::roleNames method to retrieve a HashTable of rolenames
 */
QHash<int, QByteArray> ControllerStateListModel::roleNames() const {
    QHash<int, QByteArray> roles;
    roles[MotorIDRole]    = "id";
    roles[MotorStateRole] = "motorState";
    roles[IconNameRole]   = "iconPath";
    return roles;
}

/**
 * @brief ControllerStateListModel::rowCount method to retrieve the current number of rows in the model
 * @param parent index of the parent item
 */
int ControllerStateListModel::rowCount (const QModelIndex & parent) const {
	return m_states.count();
}