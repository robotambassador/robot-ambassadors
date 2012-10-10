/****************************************************************************
** Meta object code from reading C++ file 'calibrationDialog.h'
**
** Created: Sun Nov 20 16:57:26 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "calibrationDialog.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'calibrationDialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_calibrationDialog[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      19,   18,   18,   18, 0x08,
      49,   18,   18,   18, 0x08,
      78,   18,   18,   18, 0x08,
     106,   18,   18,   18, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_calibrationDialog[] = {
    "calibrationDialog\0\0on_distortionButton_clicked()\0"
    "on_intrinsicButton_clicked()\0"
    "on_distortionSvAs_clicked()\0"
    "on_intrinsicSvAs_clicked()\0"
};

const QMetaObject calibrationDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_calibrationDialog,
      qt_meta_data_calibrationDialog, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &calibrationDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *calibrationDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *calibrationDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_calibrationDialog))
        return static_cast<void*>(const_cast< calibrationDialog*>(this));
    if (!strcmp(_clname, "Ui::calibrationDialog"))
        return static_cast< Ui::calibrationDialog*>(const_cast< calibrationDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int calibrationDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: on_distortionButton_clicked(); break;
        case 1: on_intrinsicButton_clicked(); break;
        case 2: on_distortionSvAs_clicked(); break;
        case 3: on_intrinsicSvAs_clicked(); break;
        default: ;
        }
        _id -= 4;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
