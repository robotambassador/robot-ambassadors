/****************************************************************************
** Meta object code from reading C++ file 'perspectiveDialog.h'
**
** Created: Sun Nov 20 16:57:23 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "perspectiveDialog.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'perspectiveDialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_perspectiveDialog[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      19,   18,   18,   18, 0x08,
      50,   18,   18,   18, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_perspectiveDialog[] = {
    "perspectiveDialog\0\0on_perspectiveButton_clicked()\0"
    "on_perspectiveSvAs_clicked()\0"
};

const QMetaObject perspectiveDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_perspectiveDialog,
      qt_meta_data_perspectiveDialog, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &perspectiveDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *perspectiveDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *perspectiveDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_perspectiveDialog))
        return static_cast<void*>(const_cast< perspectiveDialog*>(this));
    if (!strcmp(_clname, "Ui::perspectiveDialog"))
        return static_cast< Ui::perspectiveDialog*>(const_cast< perspectiveDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int perspectiveDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: on_perspectiveButton_clicked(); break;
        case 1: on_perspectiveSvAs_clicked(); break;
        default: ;
        }
        _id -= 2;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
