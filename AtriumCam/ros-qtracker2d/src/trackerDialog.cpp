#include <QtGui>
#include <QString>
#include "ros-qtracker2d/trackerDialog.hpp"
#include <stdio.h>

trackerDialog::trackerDialog(QWidget    *parent) : QDialog(parent)
{
    QSettings settings("IPAB", "Tracker2D");
    QString qfile_num, qsearchSize, qtemplateSize, qthreshold;

    setupUi(this);
    int s_file_num = settings.value("file_num").toInt();
    int s_searchSize = settings.value("searchSize").toInt();
    int s_templateSize = settings.value("templateSize").toInt();
    int s_threshold = settings.value("threshold").toInt();
    int s_obj_num = settings.value("obj_num").toInt();


    file_num->setText(qfile_num.setNum(s_file_num));
    searchSize->setText(qsearchSize.setNum(s_searchSize));
    templateSize->setText(qtemplateSize.setNum(s_templateSize));
    threshold->setText(qthreshold.setNum(s_threshold));
    obj_num->setValue(s_obj_num);


    QString qoutput = settings.value("output").toString();
    QString qObj1 = settings.value("nObj1").toString();
    QString qObj2 = settings.value("nObj2").toString();
    QString qObj3 = settings.value("nObj3").toString();
    QString qObj4 = settings.value("nObj4").toString();
    QString qObj5 = settings.value("nObj5").toString();
    QString qObj6 = settings.value("nObj6").toString();

    output->setText(qoutput);
    nObj1->setText(qObj1);
    nObj2->setText(qObj2);
    nObj3->setText(qObj3);
    nObj4->setText(qObj4);
    nObj5->setText(qObj5);
    nObj6->setText(qObj6);

    bool s_useCalibration = settings.value("useCalibration").toBool();
    bool s_usePerspective = settings.value("usePerspective").toBool();
    bool s_saveVideo = settings.value("saveVideo").toBool();

    if(s_useCalibration)
        useCalibration->setCheckState(Qt::Checked);
    if(s_usePerspective)
        usePerspective->setCheckState(Qt::Checked);
    if(s_saveVideo)
        saveVideo->setCheckState(Qt::Checked);

    setFixedSize(620,380);

}


void trackerDialog::on_outputButton_clicked(){

    QString s = QFileDialog::getExistingDirectory(this,tr("Select Directory"),".", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    output->setText(s);

}
