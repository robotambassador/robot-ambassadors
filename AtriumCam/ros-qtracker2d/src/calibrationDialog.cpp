#include <QtGui>
#include <QString>
#include "ros-qtracker2d/calibrationDialog.hpp"

calibrationDialog::calibrationDialog(QWidget    *parent) : QDialog(parent)
{
    QSettings settings("IPAB", "Tracker2D");
    QString qchessnum, qrealsize, qsqr_p_col, qsqr_p_row;

    int s_chessnum = settings.value("chessnum").toInt();
    int s_realsize = settings.value("realsize").toInt();
    int s_sqr_p_col = settings.value("sqr_p_col").toInt();
    int s_sqr_p_row = settings.value("sqr_p_row").toInt();
    QString qdistortion = settings.value("distortion").toString();
    QString qintrinsic = settings.value("intrinsic").toString();



    setupUi(this);

    chessnum->setText(qchessnum.setNum(s_chessnum));
    realsize->setText(qrealsize.setNum(s_realsize));
    sqr_p_col->setText(qsqr_p_col.setNum(s_sqr_p_col));
    sqr_p_row->setText(qsqr_p_row.setNum(s_sqr_p_row));
    distortion->setText(qdistortion);
    intrinsic->setText(qintrinsic);

    setFixedSize(420,374);

}

void calibrationDialog::on_distortionButton_clicked(){

    QString s = QFileDialog::getOpenFileName(this,"Choose a file",".", "XML files (*.xml)");
    distortion->setText(s);

}

void calibrationDialog::on_intrinsicButton_clicked(){

    QString s = QFileDialog::getOpenFileName(this,"Choose a file",".", "XML files (*.xml)");
    intrinsic->setText(s);

}

void calibrationDialog::on_distortionSvAs_clicked(){

    QString s = QFileDialog::getSaveFileName(this, tr("Save File"),"./distortion.xml", "XML files (*.xml)");
    distortion->setText(s);

}

void calibrationDialog::on_intrinsicSvAs_clicked(){

    QString s = QFileDialog::getSaveFileName(this, tr("Save File"),"./intrinsic.xml", "XML files (*.xml)");
    intrinsic->setText(s);

}

