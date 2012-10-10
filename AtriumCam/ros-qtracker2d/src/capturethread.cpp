#include "ros-qtracker2d/capturethread.hpp"
#include "ros-qtracker2d/imagebuffer.hpp"
#include <QDebug>
#include <QTime>
#include <QSettings>

QSettings live("IPAB", "Tracker2D");

CaptureThread::CaptureThread(bool videoinput, ImageBuffer* buffer) : QThread(), imageBuffer(buffer), captureActive(false), fps(0.0) {

    if (videoinput){
        capture = cvCreateFileCapture(live.value("avifile").toString().toAscii());
    }else{
        int cam = live.value("camera").toInt();
        capture = cvCaptureFromCAM(cam);
    }
}

//
//  Start the thread
//
void CaptureThread::run() {
    qDebug() << "Running capture thread";	
    QTime time;
    qDebug() << "Starting time";
    time.start();
    while(true) {
        if(!captureActive) {
            captureLock.lock();
            qDebug() << "Waiting on live feed start...";
            fps = 0;
            frameTimes.clear();
            captureWait.wait(&captureLock);
            time.restart();
            updateFPS(time.elapsed());
            qDebug() << "Starting live feed";
            captureLock.unlock();
        }
        imageBuffer->addFrame(cvQueryFrame(capture));
        updateFPS(time.elapsed());
    }
}

void CaptureThread::updateFPS(int time) {
    frameTimes.enqueue(time);
    if(frameTimes.size() > 15) {
        frameTimes.dequeue();
    }
    if(frameTimes.size() > 1) {
        fps = frameTimes.size()/((double)time-frameTimes.head())*1000.0;
    }
    else {
        fps = 0;
    }
}

//
// Start the capture process
//
bool CaptureThread::startCapture(int framerate) {
    if(!captureActive) {
        if(!capture || !imageBuffer){
            qDebug() << "E: Capture not initialized or invalid buffer";
            return false;
        }
        qDebug() << "Listing capture properties...";
        qDebug() << "FRAME_WIDTH :" << cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
        qDebug() << "FRAME_HEIGHT :" << cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);
        //qDebug() << "CV_CAP_PROP_FPS" << cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
        //qDebug() << "CV_CAP_PROP_FOURCC" << cvGetCaptureProperty(capture, CV_CAP_PROP_FOURCC);
        qDebug() << "BRIGHTNESS :" << cvGetCaptureProperty(capture, CV_CAP_PROP_BRIGHTNESS);
        qDebug() << "CONTRAST :" << cvGetCaptureProperty(capture, CV_CAP_PROP_CONTRAST);
        qDebug() << "SATURATION :" << cvGetCaptureProperty(capture, CV_CAP_PROP_SATURATION);
        //qDebug() << "HUE :" << cvGetCaptureProperty(capture, CV_CAP_PROP_HUE);
        //int res = live.value("resolution").toInt();
        //if(res == 0) {
        //    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 320);
        //    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 240);
        //} else if(res == 1) {
            cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 640);
            cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 480);
        //} else if(res == 2) {
        //    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 960);
        //    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 720);
        //}
        //qDebug() << "Attempting to set frame rate...";
        //qDebug() << "Error:" << cvSetCaptureProperty(capture, CV_CAP_PROP_FPS, framerate);
        qDebug() << "Starting to live feed";
        captureActive = true;
        captureWait.wakeAll();
	qDebug() << "End of start capture";
        return true;
    }
    return false;
}


void CaptureThread::stopCapture() {
    qDebug() << "Stop live feed requested.";
    captureActive = false;
}
