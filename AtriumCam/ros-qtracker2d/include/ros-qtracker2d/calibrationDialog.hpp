#ifndef CALIBRATIONDIALOG_H
#define CALIBRATIONDIALOG_H

#include <QDialog>
#include "../build/ui_calibrationDialog.h"

class calibrationDialog : public QDialog, public Ui::calibrationDialog
{
    Q_OBJECT
public:
    calibrationDialog(QWidget *parent = 0);
private slots:
    void on_distortionButton_clicked();
    void on_intrinsicButton_clicked();
    void on_distortionSvAs_clicked();
    void on_intrinsicSvAs_clicked();
};

#endif // CALIBRATIONDIALOG_H
