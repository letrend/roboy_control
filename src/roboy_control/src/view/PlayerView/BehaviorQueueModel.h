#include <QtGui>
#include "../../DataTypes.h"

class BehaviorQueueModel : public QAbstractListModel
{
  Q_OBJECT

public:
  BehaviorQueueModel(QObject *parent = 0);

  int rowCount(const QModelIndex &parent = QModelIndex()) const;
  QVariant data(const QModelIndex &index, int role) const;
  Qt::ItemFlags flags(const QModelIndex &index) const;  
  void addBehaviorMetaData(RoboyBehaviorMetadata behavior);
  QList<RoboyBehaviorMetadata> getBehaviorMetaDataList() const;
  void removeBehaviorMetaData(int index);

private:
  QList<RoboyBehaviorMetadata> behaviorList;
};
