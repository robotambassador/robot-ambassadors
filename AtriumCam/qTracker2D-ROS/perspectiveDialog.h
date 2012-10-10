#ifndef PERSPECTIVEDIALOG_H
#define PERSPECTIVEDIALOG_H

#include <QDialog>
#include "ui_perspectiveDialog.h"

class perspectiveDialog : public QDialog, public Ui::perspectiveDialog
{
    Q_OBJECT
public:
    perspectiveDialog(QWidget *parent = 0);
private slots:
    void on_perspectiveButton_clicked();
    void on_perspectiveSvAs_clicked();
};

#endif // PERSPECTIVEDIALOG_H
