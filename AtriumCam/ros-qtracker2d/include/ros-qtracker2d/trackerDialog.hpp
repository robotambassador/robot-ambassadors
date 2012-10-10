#ifndef TRACKERDIALOG_H
#define TRACKERDIALOG_H

#include <QDialog>
#include "../build/ui_trackerDialog.h"

class trackerDialog : public QDialog, public Ui::trackerDialog
{
    Q_OBJECT
public:
    trackerDialog(QWidget *parent = 0);
private slots:
    void on_outputButton_clicked();
};

#endif // TRACKERDIALOG_H
