/****************************************************************************
** Meta object code from reading C++ file 'IMultiLaneViewModel.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../../src/roboy_control/src/view/PlayerView/MultiLaneView/IMultiLaneViewModel.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'IMultiLaneViewModel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_IMultiLaneViewModel[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: signature, parameters, type, tag, flags
      27,   21,   20,   20, 0x05,
      48,   21,   20,   20, 0x05,
      88,   68,   20,   20, 0x05,
     116,   68,   20,   20, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_IMultiLaneViewModel[] = {
    "IMultiLaneViewModel\0\0index\0"
    "laneInserted(qint32)\0laneRemoved(qint32)\0"
    "laneIndex,itemIndex\0itemInserted(qint32,qint32)\0"
    "itemRemoved(qint32,qint32)\0"
};

void IMultiLaneViewModel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        IMultiLaneViewModel *_t = static_cast<IMultiLaneViewModel *>(_o);
        switch (_id) {
        case 0: _t->laneInserted((*reinterpret_cast< qint32(*)>(_a[1]))); break;
        case 1: _t->laneRemoved((*reinterpret_cast< qint32(*)>(_a[1]))); break;
        case 2: _t->itemInserted((*reinterpret_cast< qint32(*)>(_a[1])),(*reinterpret_cast< qint32(*)>(_a[2]))); break;
        case 3: _t->itemRemoved((*reinterpret_cast< qint32(*)>(_a[1])),(*reinterpret_cast< qint32(*)>(_a[2]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData IMultiLaneViewModel::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject IMultiLaneViewModel::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_IMultiLaneViewModel,
      qt_meta_data_IMultiLaneViewModel, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &IMultiLaneViewModel::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *IMultiLaneViewModel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *IMultiLaneViewModel::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_IMultiLaneViewModel))
        return static_cast<void*>(const_cast< IMultiLaneViewModel*>(this));
    return QObject::qt_metacast(_clname);
}

int IMultiLaneViewModel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void IMultiLaneViewModel::laneInserted(qint32 _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void IMultiLaneViewModel::laneRemoved(qint32 _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void IMultiLaneViewModel::itemInserted(qint32 _t1, qint32 _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void IMultiLaneViewModel::itemRemoved(qint32 _t1, qint32 _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_END_MOC_NAMESPACE
