#include "ros-qtracker2d/controller.hpp"
#include "ros-qtracker2d/processingthread.hpp"
#include "ros-qtracker2d/imagebuffer.hpp"
#include "vector"

#include <QDebug>

controller::controller(bool videoinput) : frameRate(15) {
    imageBuffer = new ImageBuffer(20);
    captureThread = new CaptureThread(videoinput, imageBuffer);
    processingThread = new ProcessingThread(imageBuffer);
    processingThread->start();
    connect(processingThread,SIGNAL(finale()),this,SIGNAL(fin()));
    connect(processingThread,SIGNAL(start_play()),this,SIGNAL(startplay()));
}

void controller::setRootFilter(Filter* filter) {
    processingThread->setRootFilter(filter);
}

double controller::getFPS() {
    return captureThread->getFPS();
}

//-----------
// Live Feed
//-----------
void controller::startLiveFeeding() {
    captureThread->startCapture(frameRate);
    captureThread->start(QThread::IdlePriority);
    qDebug() << "Started the live feed thread";
}

bool controller::isLiveFeeding() {
    return captureThread->isCapturing();
}


void controller::stopLiveFeeding() {
    captureThread->stopCapture();
}

//-----------
// Tracking
//-----------
void controller::startTracking() {
    processingThread->startTracking();
}

bool controller::isTracking() {
    return processingThread->isTracking();
}


void controller::stopTracking() {
    processingThread->stopTracking();
}

vector<CvPoint> controller::getRobotPath() {
    return processingThread->getRobotPath();
}

//-------------
// Calibrating
//-------------

void controller::startCalibrating() {
    processingThread->startCalibrating();
}

bool controller::isCalibrating() {
    return processingThread->isCalibrating();
}


void controller::stopCalibrating() {
    processingThread->stopCalibrating();
}

//-------------
// PerspectiveFixing
//-------------

void controller::startPerspfixing() {
    processingThread->startPerspfixing();
}

bool controller::isPerspfixing() {
    return processingThread->isPerspfixing();
}


void controller::stopPerspfixing() {
    processingThread->stopPerspfixing();
}




void controller::stopThreads() {
    captureThread->quit();
    processingThread->quit();

}

void controller::setmouse(int x, int y){
    processingThread->mousePosition.x = x;
    processingThread->mousePosition.y = y;
}

void controller::snap(){
    processingThread->takeSnapshot = true;
}

void controller::loseobj(int obj){
    if (isTracking())
        processingThread->loseobj(obj);

}
