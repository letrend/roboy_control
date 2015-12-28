/****************************************************************************
** Meta object code from reading C++ file 'PlayerView.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/roboy_control/src/view/PlayerView/PlayerView.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'PlayerView.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_PlayerView[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x0a,
      32,   11,   11,   11, 0x0a,
      53,   11,   11,   11, 0x0a,
      73,   11,   11,   11, 0x0a,
      99,   93,   11,   11, 0x0a,
     150,  146,   11,   11, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_PlayerView[] = {
    "PlayerView\0\0playButtonClicked()\0"
    "pauseButtonClicked()\0stopButtonClicked()\0"
    "skipButtonClicked()\0index\0"
    "behaviorListViewCurrentRowChanged(QModelIndex)\0"
    "pos\0showBehaviorListItemMenu(QPoint)\0"
};

void PlayerView::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        PlayerView *_t = static_cast<PlayerView *>(_o);
        switch (_id) {
        case 0: _t->playButtonClicked(); break;
        case 1: _t->pauseButtonClicked(); break;
        case 2: _t->stopButtonClicked(); break;
        case 3: _t->skipButtonClicked(); break;
        case 4: _t->behaviorListViewCurrentRowChanged((*reinterpret_cast< const QModelIndex(*)>(_a[1]))); break;
        case 5: _t->showBehaviorListItemMenu((*reinterpret_cast< const QPoint(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData PlayerView::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject PlayerView::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_PlayerView,
      qt_meta_data_PlayerView, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &PlayerView::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *PlayerView::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *PlayerView::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PlayerView))
        return static_cast<void*>(const_cast< PlayerView*>(this));
    if (!strcmp(_clname, "IObserver"))
        return static_cast< IObserver*>(const_cast< PlayerView*>(this));
    return QWidget::qt_metacast(_clname);
}

int PlayerView::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
