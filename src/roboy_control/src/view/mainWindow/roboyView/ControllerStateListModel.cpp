#include <QHash>
#include <QModelIndex>

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
					return "Undefined";
				case 1: // INITIALIZED
					return "Initialized";
				case 2: // PREPROCESS_TRAJECTORY
					return "Preprocess Trajectory";
				case 3: // TRAJECTORY_READY
					return "Trajectory Ready";
				case 4: // TRAJECTORY_FAILED
					return "Trajectory Failed";
				case 5: // TRAJECTORY_PLAYING
					return "Trajectory Playing";
				case 6: // TRAJECTORY_PAUSED
					return "Trajectory Pause";
				case 7: // TRAJECTORY_DONE
					return "Trajectory Done";
				case 8: // INITIALIZE_ERROR
					return "Initialize Error";
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