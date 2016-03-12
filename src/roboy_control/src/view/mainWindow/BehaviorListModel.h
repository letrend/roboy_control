#ifndef BEHAVIORLISTMODEL_H
#define BEHAVIORLISTMODEL_H

#include <QAbstractListModel>
#include <QVariant>

#include "DataTypes.h"
#include "IObserver.h"
#include "IModelService.h"

class BehaviorListModel : public QAbstractListModel, public IObserver {
    Q_OBJECT

public:
	enum BehaviorListRoles {
        IDRole = Qt::UserRole + 1,
        TitleRole,
        IconNameRole,
        DurationRole,
        MotorCountRole,
        MotorInfoRole
    };

    BehaviorListModel(IModelService * pModelService, QObject * parent = 0);

    RoboyBehavior behaviorAt(qint32 index);

    /* methods implemented from QAbstractListModel */
    QVariant 				data     (const QModelIndex & index, int role = Qt::DisplayRole) const;
    QHash<int, QByteArray> 	roleNames()                                                      const;
    int 					rowCount (const QModelIndex & parent = QModelIndex())            const;

    /* methods implemented from IObserver interface */
    void notify();

private:
    QList<RoboyBehavior> m_BehaviorList;
    IModelService * m_pModelService;

    void updateBehaviorList();
};

#endif // BEHAVIORLISTMODEL_H