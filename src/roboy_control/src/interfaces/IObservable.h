#ifndef IOBSERVABLE_H
#define IOBSERVABLE_H

#include "IObserver.h"

#include <QList>

class Observable {
private:
    QList<Observer> m_listObservers;

public:
   void subscribe(const IObserver * p_observer) = 0;
   void unsubscribe(const IObserver * p_observer) = 0;
   void notifyObservers() = 0;

};

#endif // IOBSERVABLE_H
