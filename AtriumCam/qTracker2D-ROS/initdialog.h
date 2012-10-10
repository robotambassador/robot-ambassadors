#ifndef INITDIALOG_H
#define INITDIALOG_H


#include <QDialog>
#include <QString>
#include "ui_initdialog.h"
class initdialog : public QDialog, public Ui::initdialog
{
    Q_OBJECT
public:
    initdialog(QWidget *parent = 0);
private slots:
    void on_videoButton_clicked();
    void on_typeCombo_currentIndexChanged(int index);
};

#endif // INITDIALOG_H
