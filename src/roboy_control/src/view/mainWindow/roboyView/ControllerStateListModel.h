#ifndef CONTROLLERSTATELISTMODEL_H
#define CONTROLLERSTATELISTMODEL_H

#include <QAbstractListModel>
#include <QVariant>

#include "DataTypes.h"

class ControllerStateListModel : public QAbstractListModel {
    
    Q_OBJECT

public:
    enum BehaviorListRoles {
        MotorIDRole = Qt::UserRole + 1,
        MotorStateRole,
        IconNameRole
    };

    ControllerStateListModel(QObject * parent = 0);
    void controllerStatusUpdated(qint32 motorId, ControllerState state);
    void dataPoolReset();

    // methods implemented from QAbstractListModel
    QVariant               data     (const QModelIndex & index, int role = Qt::DisplayRole) const;
    QHash<int, QByteArray> roleNames()                                                      const;
    int 	               rowCount (const QModelIndex & parent = QModelIndex())            const;

private:
    QMap<qint32, QPair<qint32, ControllerState>> m_states;
};

#endif // CONTROLLERSTATELISTMODEL_H