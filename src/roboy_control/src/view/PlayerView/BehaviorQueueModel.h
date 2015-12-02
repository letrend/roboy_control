#include <QtGui>
#include "../../DataTypes.h"

class BehaviorQueueModel : public QAbstractListModel
{
  Q_OBJECT

public:
  BehaviorQueueModel(QObject *parent = 0);

  int rowCount(const QModelIndex &parent = QModelIndex()) const;
  QVariant data(const QModelIndex &index, int role) const;  
  void addBehaviorMetaData(RoboyBehaviorMetadata behavior);
  void removeBehaviorMetaData(int index);

private:
  QList<RoboyBehaviorMetadata> behaviorList;
};
