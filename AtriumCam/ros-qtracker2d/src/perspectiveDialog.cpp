#include <QtGui>
#include <QString>
#include "ros-qtracker2d/perspectiveDialog.hpp"

perspectiveDialog::perspectiveDialog(QWidget    *parent) : QDialog(parent)
{
      QSettings settings("IPAB", "Tracker2D");

      QString qpoint1 = settings.value("point1").toString();
      QString qpoint2 = settings.value("point2").toString();
      QString qpoint3 = settings.value("point3").toString();
      QString qpoint4 = settings.value("point4").toString();
      QString qperspective = settings.value("perspective").toString();

      setupUi(this);

      point1->setText(qpoint1);
      point2->setText(qpoint2);
      point3->setText(qpoint3);
      point4->setText(qpoint4);
      perspective->setText(qperspective);

      setFixedSize(400,320);

}


void perspectiveDialog::on_perspectiveButton_clicked(){

    QString s = QFileDialog::getOpenFileName(this,"Choose a file",".", "XML files (*.xml)");
    perspective->setText(s);

}

void perspectiveDialog::on_perspectiveSvAs_clicked(){

    QString s = QFileDialog::getSaveFileName(this, tr("Save File"),"./perspective.xml", "XML files (*.xml)");
    perspective->setText(s);

}
