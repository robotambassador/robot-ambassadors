#ifndef PROCESSING_THREAD_H
#define PROCESSING_THREAD_H

#include <QThread>
#include "opencv/cv.h"
#include "opencv/highgui.h"
using std::vector;

class ImageBuffer;
class Filter;

class ProcessingThread : public QThread {
    Q_OBJECT
public:     
    CvPoint mousePosition;
    bool takeSnapshot;
    ProcessingThread(ImageBuffer* buffer);
    void setRootFilter(Filter* filter) { rootFilter = filter; }
    void run();
    void startTracking(){ tracking = true; };
    void stopTracking(){ tracking = false; };
    void startCalibrating(){ calibrating = true; };
    void stopCalibrating(){ calibrating = false; };
    void startPerspfixing(){ perspfixing = true; };
    void stopPerspfixing(){ perspfixing = false; };
    void livefeed();
    void track();
    void calibration();
    void Perspectivefix();
    bool isTracking(){ return tracking;};
    bool isCalibrating(){ return calibrating;};
    bool isPerspfixing(){ return perspfixing;};
    void loseobj(int key){ lose = key;}
    vector<CvPoint> getRobotPath();
    CvPoint getRobotLocation();
    CvPoint robotLocation;
private:
    ImageBuffer* imageBuffer;
    Filter* rootFilter;
    bool tracking;
    bool calibrating;
    bool perspfixing;
    int lose;
signals:
    void finale();
    void start_play();
};

#endif
