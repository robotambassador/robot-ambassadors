#include <QtGui>
#include <QString>
#include <QDebug>
#include "ros-qtracker2d/initdialog.hpp"

initdialog::initdialog(QWidget    *parent) : QDialog(parent)
{
    QSettings settings("IPAB", "Tracker2D");
    QString qcam;

    int res = settings.value("resolution").toInt();
    int cam = settings.value("camera").toInt();

    qcam = qcam.setNum(cam);
    setupUi(this);
    lineEdit->setText(qcam);
    comboBox->setCurrentIndex(res);

    cameraGroup->setVisible(true);
    videoGroup->setVisible(false);
    setFixedHeight(200);
    setFixedWidth(300);

}

void initdialog::on_videoButton_clicked(){
    QString s = QFileDialog::getOpenFileName(this,"Choose a video file",".", "Video files (*.avi)");
    videoEdit->setText(s);
}


void initdialog::on_typeCombo_currentIndexChanged(int index){
    if (index){
        cameraGroup->setVisible(false);
        videoGroup->setVisible(true);
        setFixedHeight(160);
    }else{
        cameraGroup->setVisible(true);
        videoGroup->setVisible(false);
        setFixedHeight(200);
    }
}
