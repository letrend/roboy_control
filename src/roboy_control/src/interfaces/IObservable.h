#ifndef IOBSERVABLE_H
#define IOBSERVABLE_H

#include "LogDefines.h"
#include "IObserver.h"

#include <QList>

class IObservable {
private:
    QList<IObserver *> m_listObservers;

protected:
    void notifyAll() {
        OBSERVABLE_DBG << "Notify All: Observer List Length: " << m_listObservers.count();
        for(IObserver * p_Observer : m_listObservers) {
            p_Observer->notify();
        }
    }

public:
   void subscribe(IObserver * p_observer) {
       m_listObservers.append(p_observer);
   }
   void unsubscribe(IObserver * p_observer) {
       m_listObservers.removeOne(p_observer);
   }
};

#endif // IOBSERVABLE_H
