/****************************************************************************
** Meta object code from reading C++ file 'controller.h'
**
** Created: Sun Nov 20 16:57:29 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "controller.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'controller.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_controller[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x05,
      18,   11,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
      30,   11,   11,   11, 0x0a,
      49,   11,   11,   11, 0x0a,
      67,   11,   11,   11, 0x0a,
      83,   11,   11,   11, 0x0a,
      98,   11,   11,   11, 0x0a,
     117,   11,   11,   11, 0x0a,
     135,   11,   11,   11, 0x0a,
     154,   11,   11,   11, 0x0a,
     172,   11,   11,   11, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_controller[] = {
    "controller\0\0fin()\0startplay()\0"
    "startLiveFeeding()\0stopLiveFeeding()\0"
    "startTracking()\0stopTracking()\0"
    "startCalibrating()\0stopCalibrating()\0"
    "startPerspfixing()\0stopPerspfixing()\0"
    "stopThreads()\0"
};

const QMetaObject controller::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_controller,
      qt_meta_data_controller, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &controller::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *controller::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *controller::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_controller))
        return static_cast<void*>(const_cast< controller*>(this));
    return QObject::qt_metacast(_clname);
}

int controller::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: fin(); break;
        case 1: startplay(); break;
        case 2: startLiveFeeding(); break;
        case 3: stopLiveFeeding(); break;
        case 4: startTracking(); break;
        case 5: stopTracking(); break;
        case 6: startCalibrating(); break;
        case 7: stopCalibrating(); break;
        case 8: startPerspfixing(); break;
        case 9: stopPerspfixing(); break;
        case 10: stopThreads(); break;
        default: ;
        }
        _id -= 11;
    }
    return _id;
}

// SIGNAL 0
void controller::fin()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void controller::startplay()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}
QT_END_MOC_NAMESPACE
