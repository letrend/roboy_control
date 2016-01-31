#include <QtGui>

#include "../../DataTypes.h"
#include "../../interfaces/IObserver.h"
#include "../../model/IModelService.h"

class BehaviorListModel : public QAbstractListModel, public IObserver {
    Q_OBJECT

public:
    /* constructor */
    BehaviorListModel(IModelService *modelService, QObject *parent = 0);

    /* methods implemented from IObserver interface */
    void notify();

    /* QAbstractListModel methods */
    QVariant data(const QModelIndex &index, int role) const;
    int rowCount(const QModelIndex &parent = QModelIndex()) const;

    /* BehaviorListModel methods */
    RoboyBehavior getBehavior(int index) const;

private:
    QList<RoboyBehavior> behaviorList;
    IModelService *modelService;

    void updateBehaviorList();
};
