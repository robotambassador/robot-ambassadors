#include <QtGui>
#include <opencv/highgui.h>
#include "ros-qtracker2d/mainwin.hpp"
#include "ros-qtracker2d/processingthread.hpp"
#include <iostream>
#include <assert.h>
#include <QSettings>
#include <QFileDialog>
#include <QTimer>
#include <QDebug>
#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace std;
QSettings settings("IPAB", "Tracker2D");

mainwin::mainwin(int argc, char *argv[]): qNode() {

    setupUi(this);

    // Open dialog for camera number and resolution
    CvCapture *camera;
    initdialog dialog(this);
    if (dialog.exec()) {


        if (dialog.typeCombo->currentIndex()){
            videoinput = true;
            camera = cvCaptureFromAVI(dialog.videoEdit->text().toAscii());
            settings.setValue("avifile", dialog.videoEdit->text());

        }else{
            videoinput = false;
            int cam_num = dialog.lineEdit->text().toInt();
            int idx = dialog.comboBox->currentIndex();
            camera = cvCaptureFromCAM(cam_num);//cvCreateCameraCapture
            assert(camera);

            setCameraWH(idx);
            cvSetCaptureProperty(camera, CV_CAP_PROP_FRAME_WIDTH, cameraWidth);
            cvSetCaptureProperty(camera, CV_CAP_PROP_FRAME_HEIGHT, cameraHeight);
            if(cvGetCaptureProperty(camera, CV_CAP_PROP_FRAME_WIDTH)==320)
                idx = 0;
            if(cvGetCaptureProperty(camera, CV_CAP_PROP_FRAME_WIDTH)==640)
                idx = 1;
            if(cvGetCaptureProperty(camera, CV_CAP_PROP_FRAME_WIDTH)==960)
                idx = 2;
            setCameraWH(idx);

            settings.setValue("resolution", idx);
            settings.setValue("camera", cam_num);
        }
    }else{
        this->close();
    }
    IplImage * image = cvQueryFrame(camera);
    assert(image);
    cvReleaseCapture(&camera);


    renderWidget = new RenderWidget(this);
    Controller = new controller(videoinput);
    Controller->setRootFilter(renderWidget);

    setCentralWidget(renderWidget);
    setFixedSize(cameraWidth, cameraHeight+45);

    updateTimer = new QTimer(this);
    connect(updateTimer, SIGNAL(timeout()), this, SLOT(updateStats()));
    updateTimer->start(1000);
    Controller->startLiveFeeding();

    ProcessingThread* pThread = Controller->getProcessingThread();
    qNode.init(pThread);

    // Signals
    connect(actionAboutQT, SIGNAL(triggered()), qApp, SLOT(aboutQt()));
    connect(actionTrack, SIGNAL(triggered()), this, SLOT(actTrack()));
    connect(actionCalibrate, SIGNAL(triggered()), this, SLOT(actCalibrate()));
    connect(actionPerspective, SIGNAL(triggered()), this, SLOT(actPerspective()));
    connect(actionTracker, SIGNAL(triggered()), this, SLOT(set_track()));
    connect(actionCalibration, SIGNAL(triggered()), this, SLOT(set_calibrate()));
    connect(actionPerspective_2, SIGNAL(triggered()), this, SLOT(set_perspective()));
    connect(actionAbout, SIGNAL(triggered()), this, SLOT(about()));
    connect(Controller, SIGNAL(fin()), this, SLOT(end_signal()));
    connect(Controller, SIGNAL(startplay()), this, SLOT(play()));

}

void mainwin::play() {
	return;
}

void mainwin::finished_play() {
    play();
}

void mainwin::about() {
    QMessageBox::about(this, tr("About QTracker2D"),
                       tr("<h2>QTracker2d</h2>"
                          "<p>Copyright &copy; 2010 Alexandros Asthenidis, Georgios Petrou, and Jan Wessnitzer. ROSified version: Matthew Whitaker 2011"));
}

void mainwin::actTrack(){
    if(Controller->isTracking()==0){
        // Start tracking
        Controller->startTracking();
        disable(1);
    } else {
        // Stop tracking
        end_signal();
        Controller->stopTracking();
        enable();
        cvWaitKey(1000);
    }
}

void mainwin::actCalibrate(){
    if(Controller->isCalibrating()==0){
        // Start tracking
        Controller->startCalibrating();
        disable(2);
    } else {
        // Stop tracking
        Controller->stopCalibrating();
        enable();
    }
}

void mainwin::actPerspective(){
    if(Controller->isPerspfixing()==0){
        // Start tracking
        Controller->startPerspfixing();
        disable(3);
    } else {
        // Stop tracking
        Controller->stopPerspfixing();
        enable();
        cvWaitKey(1000);
    }
}

void mainwin::set_track(){

    trackerDialog dialog(this);
    if (dialog.exec()) {

        int s_file_num = dialog.file_num->text().toInt();
        int s_searchSize = dialog.searchSize->text().toInt();
        int s_templateSize = dialog.templateSize->text().toInt();
        int s_threshold = dialog.threshold->text().toInt();
        int s_obj_num = dialog.obj_num->text().toInt();
        bool s_useCalibration = dialog.useCalibration->isChecked();
        bool s_usePerspective = dialog.usePerspective->isChecked();
        bool s_saveVideo = dialog.saveVideo->isChecked();

        QString qoutput = dialog.output->text();
        QString qObj1 = dialog.nObj1->text();
        QString qObj2 = dialog.nObj2->text();
        QString qObj3 = dialog.nObj3->text();
        QString qObj4 = dialog.nObj4->text();
        QString qObj5 = dialog.nObj5->text();
        QString qObj6 = dialog.nObj6->text();

        // saving
        settings.setValue("file_num", s_file_num);
        settings.setValue("output", qoutput);
        settings.setValue("searchSize", s_searchSize);
        settings.setValue("templateSize", s_templateSize);
        settings.setValue("threshold", s_threshold);
        settings.setValue("obj_num", s_obj_num);
        settings.setValue("nObj1", qObj1);
        settings.setValue("nObj2", qObj2);
        settings.setValue("nObj3", qObj3);
        settings.setValue("nObj4", qObj4);
        settings.setValue("nObj5", qObj5);
        settings.setValue("nObj6", qObj6);
        settings.setValue("useCalibration", s_useCalibration);
        settings.setValue("usePerspective", s_usePerspective);
        settings.setValue("saveVideo", s_saveVideo);
    }
}

void mainwin::set_calibrate(){

    calibrationDialog dialog(this);
    if (dialog.exec()) {

        int s_chessnum = dialog.chessnum->text().toInt();
        int s_realsize = dialog.realsize->text().toInt();
        int s_sqr_p_col = dialog.sqr_p_col->text().toInt();
        int s_sqr_p_row = dialog.sqr_p_row->text().toInt();
        QString qdistortion = dialog.distortion->text();
        QString qintrinsic = dialog.intrinsic->text();

        // saving
        settings.setValue("chessnum", s_chessnum);
        settings.setValue("realsize", s_realsize);
        settings.setValue("sqr_p_col", s_sqr_p_col);
        settings.setValue("sqr_p_row", s_sqr_p_row);
        settings.setValue("distortion", qdistortion);
        settings.setValue("intrinsic", qintrinsic);
    }
}

void mainwin::set_perspective(){

    perspectiveDialog dialog(this);
    if (dialog.exec()) {

        QString qpoint1 = dialog.point1->text();
        QString qpoint2 = dialog.point2->text();
        QString qpoint3 = dialog.point3->text();
        QString qpoint4 = dialog.point4->text();
        QString qperspective = dialog.perspective->text();

        // saving
        settings.setValue("point1", qpoint1);
        settings.setValue("point2", qpoint2);
        settings.setValue("point3", qpoint3);
        settings.setValue("point4", qpoint4);
        settings.setValue("perspective", qperspective);
    }
}

void mainwin::end_signal(){
    if(Controller->isCalibrating()){
        actCalibrate();
        QMessageBox::information(this, "Camera Calibration", "Calibration finished.");
    }else if(Controller->isPerspfixing()){
        actPerspective();
        QMessageBox::information(this, "Camera Perspective fixing", "Perspective fixing finished.");
    }else if (Controller->isTracking()){
    }

}

void mainwin::disable(int m){

    actionTrack->setEnabled(0);
    actionCalibrate->setEnabled(0);
    actionPerspective->setEnabled(0);
    actionExit->setEnabled(0);
    actionTracker->setEnabled(0);
    actionCalibration->setEnabled(0);
    actionPerspective_2->setEnabled(0);
    actionAboutQT->setEnabled(0);
    actionTracker_Help->setEnabled(0);
    actionAbout->setEnabled(0);

    switch (m) {
    case 1:
        actionTrack->setEnabled(1);
        actionTrack->setText("Stop");
        break;
    case 2:
        actionCalibrate->setEnabled(1);
        actionCalibrate->setText("Stop");
        break;
    case 3:
        actionPerspective->setEnabled(1);
        actionPerspective->setText("Stop");
        break;
    }
}

void mainwin::enable(){

    actionTrack->setText("Start &Tracking");
    actionCalibrate->setText("Start &Calibration");
    actionPerspective->setText("Fix &Perspective");

    actionTrack->setEnabled(1);
    actionCalibrate->setEnabled(1);
    actionPerspective->setEnabled(1);
    actionExit->setEnabled(1);
    actionTracker->setEnabled(1);
    actionCalibration->setEnabled(1);
    actionPerspective_2->setEnabled(1);
    actionAboutQT->setEnabled(1);
    actionTracker_Help->setEnabled(1);
    actionAbout->setEnabled(1);

}

void mainwin::setCameraWH(int idx){
    switch(idx){
    case 0:
        cameraWidth = 320;
        cameraHeight = 240;
        break;
    case 1:
        cameraWidth = 640;
        cameraHeight = 480;
        break;
    case 2:
        cameraWidth = 960;
        cameraHeight = 720;
        break;
    }

}

void mainwin::updateStats() {
    statusBar()->showMessage(QString("Res: ")+QString::number(cameraWidth)+QString("x")+QString::number(cameraHeight)+QString("  FPS: ")+QString::number(Controller->getFPS(), 'f', 1));
}

void mainwin::closeEvent(QCloseEvent*) {
    if(Controller->isLiveFeeding()) {
        Controller->stopLiveFeeding();
    }
    Controller->stopThreads();
}

void mainwin::mousePressEvent( QMouseEvent *e ) {

    if ((Controller->isTracking())||(Controller->isPerspfixing())){
        Controller->setmouse(e->x(),e->y()-23);
    }else if (Controller->isCalibrating()){
        Controller->snap();
    }
}

void mainwin::keyPressEvent (QKeyEvent *e){

    switch (e->key()){

    case Qt::Key_1:
        Controller->loseobj(1);
        break;
    case Qt::Key_2:
        Controller->loseobj(2);
        break;
    case Qt::Key_3:
        Controller->loseobj(3);
        break;
    case Qt::Key_4:
        Controller->loseobj(4);
        break;
    case Qt::Key_5:
        Controller->loseobj(5);
        break;
    case Qt::Key_6:
        Controller->loseobj(6);
        break;
    default:
        e->ignore();
    }

}
