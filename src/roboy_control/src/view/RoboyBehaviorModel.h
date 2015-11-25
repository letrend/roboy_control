#include <QAbstractListModel>
#include "../DataTypes.h"
#include "../model/IModelService.h"

class RoboyBehaviorModel : public QAbstractListModel {

    Q_OBJECT

public:

    explicit RoboyBehaviorModel(QObject *parent, IModelService *modelService);
    int rowCount(const QModelIndex &parent = QModelIndex()) const;
    QVariant data(const QModelIndex &index, int role) const;

private:
	QList<RoboyBehaviorMetadata> behaviorList;
	
};
