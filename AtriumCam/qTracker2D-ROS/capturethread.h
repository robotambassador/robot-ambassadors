#ifndef CAPTURE_THREAD_H
#define CAPTURE_THREAD_H

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QQueue>
#include "opencv/highgui.h"

class ImageBuffer;

class CaptureThread : public QThread {
public: 
        enum FrameSize { Size960, Size640, Size320 };
        CaptureThread(bool videoinput, ImageBuffer* buffer);
	void run();
        bool startCapture(int framerate);
	void stopCapture();
	double getFPS() { return fps; }
	bool isCapturing() { return captureActive; }
private:
	void updateFPS(int time);
	QMutex captureLock;
	QWaitCondition captureWait;
	ImageBuffer* imageBuffer;
	bool captureActive;
	CvCapture* capture;
	double fps;
	QQueue<int> frameTimes;
};

#endif
