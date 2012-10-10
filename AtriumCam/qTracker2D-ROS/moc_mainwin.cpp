/****************************************************************************
** Meta object code from reading C++ file 'mainwin.h'
**
** Created: Sun Nov 27 15:04:16 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "mainwin.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwin.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_mainwin[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
       9,    8,    8,    8, 0x08,
      20,    8,    8,    8, 0x08,
      35,    8,    8,    8, 0x08,
      52,    8,    8,    8, 0x08,
      64,    8,    8,    8, 0x08,
      80,    8,    8,    8, 0x08,
      98,    8,    8,    8, 0x08,
     106,    8,    8,    8, 0x08,
     120,    8,    8,    8, 0x08,
     133,    8,    8,    8, 0x08,
     140,    8,    8,    8, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_mainwin[] = {
    "mainwin\0\0actTrack()\0actCalibrate()\0"
    "actPerspective()\0set_track()\0"
    "set_calibrate()\0set_perspective()\0"
    "about()\0updateStats()\0end_signal()\0"
    "play()\0finished_play()\0"
};

const QMetaObject mainwin::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_mainwin,
      qt_meta_data_mainwin, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &mainwin::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *mainwin::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *mainwin::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_mainwin))
        return static_cast<void*>(const_cast< mainwin*>(this));
    if (!strcmp(_clname, "Ui::mainwin"))
        return static_cast< Ui::mainwin*>(const_cast< mainwin*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int mainwin::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: actTrack(); break;
        case 1: actCalibrate(); break;
        case 2: actPerspective(); break;
        case 3: set_track(); break;
        case 4: set_calibrate(); break;
        case 5: set_perspective(); break;
        case 6: about(); break;
        case 7: updateStats(); break;
        case 8: end_signal(); break;
        case 9: play(); break;
        case 10: finished_play(); break;
        default: ;
        }
        _id -= 11;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
