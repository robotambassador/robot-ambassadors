#ifndef TRACK_CONTROLLER_H
#define TRACK_CONTROLLER_H

#include "capturethread.hpp"
using std::vector;

class Filter;
class ProcessingThread;
class ImageBuffer;

class controller : public QObject {
Q_OBJECT;
public:
        controller(bool videoinput);
        bool isLiveFeeding();
        bool isTracking();
        bool isCalibrating();
        bool isPerspfixing();
	void setFrameRate(int rate) { frameRate = rate; }
        int getFrameRate() { return frameRate; }
        void setRootFilter(Filter* filter);
	ProcessingThread* getProcessingThread() { return processingThread; }
	double getFPS();
        void setmouse(int x, int y);
        void snap();
        void loseobj(int obj);
        vector<CvPoint> getRobotPath();
public slots:
        void startLiveFeeding();
        void stopLiveFeeding();
        void startTracking();
        void stopTracking();
        void startCalibrating();
        void stopCalibrating();
        void startPerspfixing();
        void stopPerspfixing();
        void stopThreads();
private:
        int frameRate;
	CaptureThread* captureThread;
	ProcessingThread* processingThread;
	ImageBuffer* imageBuffer;
    signals:
        void fin();
        void startplay();
};

#endif
