/****************************************************************************
** Meta object code from reading C++ file 'MultiLaneView.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../../src/roboy_control/src/view/PlayerView/MultiLaneView/MultiLaneView.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'MultiLaneView.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MultiLaneView[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      21,   15,   14,   14, 0x0a,
      49,   15,   14,   14, 0x0a,
      96,   76,   14,   14, 0x0a,
     131,   76,   14,   14, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_MultiLaneView[] = {
    "MultiLaneView\0\0index\0laneInsertedHandler(qint32)\0"
    "laneRemovedHandler(qint32)\0"
    "laneIndex,itemIndex\0"
    "itemInsertedHandler(qint32,qint32)\0"
    "itemRemovedHandler(qint32,qint32)\0"
};

void MultiLaneView::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MultiLaneView *_t = static_cast<MultiLaneView *>(_o);
        switch (_id) {
        case 0: _t->laneInsertedHandler((*reinterpret_cast< qint32(*)>(_a[1]))); break;
        case 1: _t->laneRemovedHandler((*reinterpret_cast< qint32(*)>(_a[1]))); break;
        case 2: _t->itemInsertedHandler((*reinterpret_cast< qint32(*)>(_a[1])),(*reinterpret_cast< qint32(*)>(_a[2]))); break;
        case 3: _t->itemRemovedHandler((*reinterpret_cast< qint32(*)>(_a[1])),(*reinterpret_cast< qint32(*)>(_a[2]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MultiLaneView::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MultiLaneView::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_MultiLaneView,
      qt_meta_data_MultiLaneView, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MultiLaneView::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MultiLaneView::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MultiLaneView::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MultiLaneView))
        return static_cast<void*>(const_cast< MultiLaneView*>(this));
    return QWidget::qt_metacast(_clname);
}

int MultiLaneView::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
