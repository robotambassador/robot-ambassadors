#include "processingthread.h"
#include "filter.h"
#include "imagebuffer.h"
#include <string>
#include <stdio.h>
#include <vector>
#include <QSettings>
#include <QTextStream>
#include <QStringList>
#include <QString>
#include <QFile>
#include <ros/ros.h>

// OpenCV Color BGR
#define PURPLE  cvScalar(139,0,139)
#define ORANGE  cvScalar(0,154,238)
#define GREEN   cvScalar(87,201,0)
#define WHITE   cvScalar(205,205,205)
#define BLACK   cvScalar(10,10,10)
#define BLUE    cvScalar(205,0,0)

using namespace std;
QSettings process("IPAB", "Tracker2D");

CvFont status, ftext;

vector<CvPoint> robotPath;

ProcessingThread::ProcessingThread(ImageBuffer* buffer) : QThread(), imageBuffer(buffer), rootFilter(0) {
    takeSnapshot = false;
    tracking = false;
    calibrating = false;
    perspfixing = false;
    lose = 0;
    cvInitFont(&status,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 0.5, 0.5, 0, 1);
    cvInitFont(&ftext,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 0.8, 0.8, 0, 2);
    emit finale();
}

void ProcessingThread::run() {

    while(true) {
        if (tracking)
            track();
        else if (calibrating)
            calibration();
        else if (perspfixing)
            Perspectivefix();
        else
            livefeed();
    }
    this->exec();
}

void ProcessingThread::track(){

    vector<CvScalar> colorlist;
    colorlist.push_back(BLUE);
    colorlist.push_back(WHITE);
    colorlist.push_back(GREEN);
    colorlist.push_back(ORANGE);
    colorlist.push_back(PURPLE);
    colorlist.push_back(BLACK);

    char buffer [40];
    bool tracking_started = false;

    // Load settings
    int searchSize = process.value("searchSize").toInt();
    int res = process.value("resolution").toInt();
    int templateSize = process.value("templateSize").toInt();
    double threshold = process.value("threshold").toDouble()/10;
    int obj_num = process.value("obj_num").toInt();

    bool useCalibration = process.value("useCalibration").toBool();
    bool usePerspective = process.value("usePerspective").toBool();
    bool saveVideo = process.value("saveVideo").toBool();

    vector<string> objlist;
    objlist.push_back(process.value("nObj1").toString().toStdString());
    objlist.push_back(process.value("nObj2").toString().toStdString());
    objlist.push_back(process.value("nObj3").toString().toStdString());
    objlist.push_back(process.value("nObj4").toString().toStdString());
    objlist.push_back(process.value("nObj5").toString().toStdString());
    objlist.push_back(process.value("nObj6").toString().toStdString());

    // File/Dir  Paths
    string distortion = process.value("distortion").toString().toStdString();
    string intrinsic = process.value("intrinsic").toString().toStdString();
    string perspective = process.value("perspective").toString().toStdString();

    mousePosition.x = 0;
    mousePosition.y = 0;
    
    CvVideoWriter *videoWriter = NULL;

    IplImage *frame = NULL;
    IplImage* searchImage = NULL;

    vector<IplImage*> templateImage(obj_num);
    vector<CvKalman*> kalman(obj_num);
    vector<CvPoint> templatePosition(obj_num);
    vector<bool> lostObjects(obj_num);
    for (int i = 0; i < obj_num; i++) {
        //bool dum1;
        IplImage* dum2;
        CvKalman* dum3;
        CvPoint dum4;
        templateImage.push_back(dum2);
        kalman.push_back(dum3);
        templatePosition.push_back(dum4);
        lostObjects.push_back(false);
        //lostObjects[i] = false;
    }

    vector< vector<CvPoint> > objectsPaths(obj_num);
    vector<CvPoint>  empt;
    for (int i = 0; i < obj_num; i++) {
        objectsPaths.push_back(empt);
        objectsPaths[i].push_back(cvPoint(0, 0));
    }




    CvMat *measurement = NULL;
    CvMat *mapX = NULL;
    CvMat *mapY = NULL;
    CvMat *intrinsicMatrix = NULL;
    CvMat *distortionMatrix = NULL;
    CvMat *perspectiveMatrix = NULL;

    float transitionMatrix[4][4] = {{1.00, 0.00, 1.00, 0.00},/*x*/
                                    {0.00, 1.00, 0.00, 1.00},/*y*/
                                    {0.00, 0.00, 1.00, 0.00},/*dx*/
                                    {0.00, 0.00, 0.00, 1.00}/*dy*/ };
    int selectedTemplates = 0;

    // Start of try
    if (saveVideo) {
        string videoPath = process.value("output").toString().toStdString() + "/video_output_" + process.value("file_num").toString().toStdString() + ".avi";

        if(res==0)
            videoWriter = cvCreateVideoWriter( videoPath.c_str(), CV_FOURCC('M', 'J', 'P', 'G'), 15.0, cvSize(320, 240) );
        else if(res==1)
            videoWriter = cvCreateVideoWriter( videoPath.c_str(), CV_FOURCC('M', 'J', 'P', 'G'), 15.0, cvSize(640, 480) );
        else if(res==2)
            videoWriter = cvCreateVideoWriter( videoPath.c_str(), CV_FOURCC('M', 'J', 'P', 'G'), 15.0, cvSize(960, 720) );
        videoPath = "";
    }

    QFile file( process.value("output").toString() + "/output_" + process.value("file_num").toString() + ".csv" ); // Write the text to a file
    file.open(QIODevice::WriteOnly);
    QTextStream stream( &file );

    measurement = cvCreateMat(2,1,CV_32FC1);
    cvZero(measurement);
    searchImage = cvCreateImage( cvSize( searchSize  - templateSize  + 1, searchSize - templateSize + 1 ), IPL_DEPTH_32F, 1 );
    frame = imageBuffer->getFrame();

    if (useCalibration) {
        mapX = cvCreateMat( frame->height, frame->width, CV_32FC1 );
        mapY = cvCreateMat( frame->height, frame->width, CV_32FC1 );

        intrinsicMatrix = (CvMat*)cvLoad(intrinsic.c_str());
        distortionMatrix = (CvMat*)cvLoad(distortion.c_str());

        cvInitUndistortMap(intrinsicMatrix,distortionMatrix,mapX,mapY);
    }

    if (usePerspective) {
        perspectiveMatrix = (CvMat*)cvLoad(perspective.c_str());
    }

    CvPoint maxLoc;
    double  maxVal;
    int frameNumber = 0;

    while (tracking) {
        frame = imageBuffer->getFrame();

        if (useCalibration){
            IplImage *t = cvCloneImage(frame);
            cvRemap(t,frame,mapX,mapY);
            cvReleaseImage(&t);
        }

        if (usePerspective){
            IplImage *t = cvCloneImage(frame);
            cvWarpPerspective(t, frame, perspectiveMatrix);
            cvReleaseImage(&t);
        }

        if (saveVideo)
            cvWriteFrame (videoWriter, frame);

        if (lose>0){
            lostObjects[(lose-1)] = true;
            lose = 0;
        }


        if (selectedTemplates < obj_num) {
            sprintf(buffer,"Select template for %s",objlist[selectedTemplates].c_str());
            cvPutText (frame,buffer,cvPoint(200,50), &ftext, BLUE);

            if ( mousePosition.x !=0 && mousePosition.y !=0 ) {
                templateImage[selectedTemplates] = cvCreateImage( cvSize( templateSize, templateSize ), frame->depth, frame->nChannels );

                templatePosition[selectedTemplates] = cvPoint(0,0);

                templatePosition[selectedTemplates].x = mousePosition.x - ( templateSize  / 2 );
                templatePosition[selectedTemplates].y = mousePosition.y - ( templateSize  / 2 );

                if(templatePosition[selectedTemplates].x <= 0)
                    templatePosition[selectedTemplates].x = 0;
                else if(templatePosition[selectedTemplates].x+templateSize>= frame->width)
                    templatePosition[selectedTemplates].x = frame->width-templateSize;

                if(templatePosition[selectedTemplates].y <= 0)
                    templatePosition[selectedTemplates].y = 0;
                else if( templatePosition[selectedTemplates].y+templateSize >= frame->height)
                    templatePosition[selectedTemplates].y = frame->height-templateSize;

                cvSetImageROI( frame, cvRect( templatePosition[selectedTemplates].x, templatePosition[selectedTemplates].y, templateSize, templateSize ) );
                cvCopy( frame, templateImage[selectedTemplates]);
                cvResetImageROI( frame );

                kalman[selectedTemplates] = cvCreateKalman(4,2,0);

                memcpy(kalman[selectedTemplates]->transition_matrix->data.fl, transitionMatrix, sizeof(transitionMatrix));

                cvSetIdentity(kalman[selectedTemplates]->measurement_matrix, cvRealScalar(1));
                cvSetIdentity(kalman[selectedTemplates]->process_noise_cov, cvRealScalar(1e-5));
                cvSetIdentity(kalman[selectedTemplates]->measurement_noise_cov, cvRealScalar(1e-1));
                cvSetIdentity(kalman[selectedTemplates]->error_cov_post, cvRealScalar(1));

                kalman[selectedTemplates]->state_post->data.fl[0] = (float)templatePosition[selectedTemplates].x;
                kalman[selectedTemplates]->state_post->data.fl[1] = (float)templatePosition[selectedTemplates].y;

                mousePosition.x = 0;
                mousePosition.y = 0;

                selectedTemplates++;
            }
        } else {
            if (!(tracking_started)){
                emit start_play();
                tracking_started = true;
            }
            for (int i=0; i<obj_num; i++) {
                if(lostObjects[i]) {

                    cvPutText (frame,"Object is lost.",cvPoint(200,50), &ftext, BLUE);
                    sprintf(buffer,"Reselect template for %s",objlist[i].c_str());
                    cvPutText (frame,buffer,cvPoint(200,75), &ftext, BLUE);

                    if ( mousePosition.x !=0 && mousePosition.y !=0 ) {
                        templatePosition[i].x = mousePosition.x - ( templateSize  / 2 );
                        templatePosition[i].y = mousePosition.y - ( templateSize  / 2 );

                        if(templatePosition[i].x <= 0)
                            templatePosition[i].x = 0;
                        else if(templatePosition[i].x+templateSize>= frame->width)
                            templatePosition[i].x = frame->width-templateSize;

                        if(templatePosition[i].y <= 0)
                            templatePosition[i].y = 0;
                        else if( templatePosition[i].y+templateSize >= frame->height)
                            templatePosition[i].y = frame->height-templateSize;

                        cvSetImageROI( frame, cvRect( templatePosition[i].x, templatePosition[i].y, templateSize, templateSize ) );
                        cvCopy( frame, templateImage[i]);
                        cvResetImageROI( frame );

                        memcpy(kalman[i]->transition_matrix->data.fl, transitionMatrix, sizeof(transitionMatrix));

                        cvSetIdentity(kalman[i]->measurement_matrix, cvRealScalar(1));
                        cvSetIdentity(kalman[i]->process_noise_cov, cvRealScalar(1e-5));
                        cvSetIdentity(kalman[i]->measurement_noise_cov, cvRealScalar(1e-1));
                        cvSetIdentity(kalman[i]->error_cov_post, cvRealScalar(1));

                        kalman[i]->state_post->data.fl[0] = (float)templatePosition[i].x;
                        kalman[i]->state_post->data.fl[1] = (float)templatePosition[i].y;

                        mousePosition.x = 0;
                        mousePosition.y = 0;

                        lostObjects[i] = false;
                    }
                } else {
                    CvPoint searchPoint;
                    searchPoint.x = templatePosition[i].x - ((searchSize - templateSize )/2);
                    searchPoint.y = templatePosition[i].y - ((searchSize - templateSize )/2);

                    if(searchPoint.x <= 0)
                        searchPoint.x = 0;
                    else if(searchPoint.x+searchSize >= frame->width)
                        searchPoint.x = frame->width-searchSize;

                    if(searchPoint.y <= 0)
                        searchPoint.y = 0;
                    else if(searchPoint.y+searchSize >= frame->height)
                        searchPoint.y = frame->height-searchSize;

                    cvSetImageROI( frame, cvRect( searchPoint.x,  searchPoint.y, searchSize, searchSize ) );
                    cvMatchTemplate( frame, templateImage[i], searchImage, CV_TM_CCOEFF_NORMED );
                    cvResetImageROI(frame);

                    cvMinMaxLoc( searchImage, NULL, &maxVal, NULL, &maxLoc, 0 );

                    const CvMat* kalmanPrediction = cvKalmanPredict( kalman[i], 0 );

                    cvRectangle(frame, cvPoint(kalmanPrediction->data.fl[0],kalmanPrediction->data.fl[1]),
                                cvPoint(kalmanPrediction->data.fl[0]+templateSize,kalmanPrediction->data.fl[1]+templateSize),PURPLE,1);
                    cvRectangle(frame, cvPoint(searchPoint.x,searchPoint.y),
                                cvPoint(searchPoint.x+searchSize,searchPoint.y+searchSize),ORANGE,1);

                    if( maxVal >= threshold ) {
                        templatePosition[i].x = searchPoint.x + maxLoc.x;
                        templatePosition[i].y = searchPoint.y + maxLoc.y;

                        measurement->data.fl[0] = (float)templatePosition[i].x;
                        measurement->data.fl[1] = (float)templatePosition[i].y;
                        cvKalmanCorrect( kalman[i], measurement);

                        cvRectangle(frame, cvPoint(templatePosition[i].x,templatePosition[i].y),
                                    cvPoint(templatePosition[i].x+templateSize,templatePosition[i].y+templateSize),GREEN,1);
                    } else {
                        if( kalmanPrediction->data.fl[0] >= searchPoint.x &&
                            kalmanPrediction->data.fl[0]+ templateSize <= searchPoint.x + searchSize &&
                            kalmanPrediction->data.fl[1] >= searchPoint.y &&
                            kalmanPrediction->data.fl[1]+ templateSize <= searchPoint.y + searchSize) {

                            templatePosition[i].x = (int)kalmanPrediction->data.fl[0];
                            templatePosition[i].y = (int)kalmanPrediction->data.fl[1];
                        } else {
                            lostObjects[i] = true;
                        }
                    }

                    CvPoint pt =objectsPaths[i][objectsPaths[i].size()-1];
                    cvPutText (frame,objlist[i].c_str(),cvPoint(templatePosition[i].x,templatePosition[i].y), &status, colorlist[i]);

                    // TODO: Use fake_localization as a template for published Pose data from these points
                    // (Quaternion calculated from point history?)

                    // The bit where everything is fine and new points are added to the track (?)
                    if(abs(pt.x - templatePosition[i].x) > 1 || abs(pt.y - templatePosition[i].y) > 1) {
                        CvPoint newPt = cvPoint((templatePosition[i].x + templateSize/2), (templatePosition[i].y + templateSize/2));
                        objectsPaths[i].push_back(newPt);
                        robotLocation = newPt;
                        std::cout << "New point at: X = " << newPt.x << ", Y = " << newPt.y << std::endl;
                    }


                    if (objectsPaths[i].size() > 1) {
                        for(unsigned int j=0; j<objectsPaths[i].size()-1; j++) {
                            if (objectsPaths[i][j].x != 0 && objectsPaths[i][j].y != 0)
                                cvLine(frame, objectsPaths[i][j], objectsPaths[i][j+1], colorlist[i], 1);
                        }
                    }
                }

            robotPath = objectsPaths[0];

            }
            mousePosition.x = 0;
            mousePosition.y = 0;
        }

        // Write to cvs file
        stream << frameNumber;
        for (int i=0; i<obj_num; i++) {
            CvPoint pt =objectsPaths[i][objectsPaths[i].size()-1] ;
            stream << "," << pt.x << "," << pt.y;
        }
        stream << "\n";

        // show image
        cvPutText (frame,"Tracking",cvPoint(20,20), &status, WHITE);

        if(rootFilter) {
            rootFilter->processPoint(frame);
        }
        cvReleaseImage(&frame);

        frameNumber++;
    }
    file.close();
    cvReleaseVideoWriter(&videoWriter);
    cvReleaseImage(&searchImage);
    if(tracking_started)
        process.setValue("file_num",process.value("file_num").toInt()+1);
}

void ProcessingThread::calibration(){

    int chessboardsNumber = process.value("chessnum").toInt();
    int realSquareSize = process.value("realsize").toInt();
    int boardHeight = process.value("sqr_p_col").toInt()-1;
    int boardWidth = process.value("sqr_p_row").toInt()-1;
    int cornersNumber = boardWidth * boardHeight;

    char buffer [40];

    IplImage *frame = NULL;
    IplImage *grayImage = NULL;

    CvMat* imagePoints = NULL;
    CvMat* objectPoints = NULL;
    CvMat* pointCounts = NULL;
    CvMat* intrinsicMatrix = NULL;
    CvMat* distortionMatrix = NULL;
    CvMat* rotationMatrix = NULL;
    CvMat* translationMatrix = NULL;

    CvPoint2D32f* corners = new CvPoint2D32f[cornersNumber];


    CvSize chessboardSize = cvSize( boardWidth, boardHeight);

    imagePoints	= cvCreateMat(chessboardsNumber*cornersNumber, 2, CV_32FC1);
    objectPoints = cvCreateMat(chessboardsNumber*cornersNumber, 3, CV_32FC1);
    pointCounts = cvCreateMat(chessboardsNumber, 1, CV_32SC1);
    intrinsicMatrix = cvCreateMat(3, 3, CV_32FC1);
    distortionMatrix = cvCreateMat(5, 1, CV_32FC1);
    rotationMatrix = cvCreateMat(chessboardsNumber,3,CV_32FC1);
    translationMatrix = cvCreateMat(chessboardsNumber,3,CV_32FC1);

    int cornerCount = 0;
    int successes = 0;
    int step = 0;

    frame = imageBuffer->getFrame();

    grayImage = cvCreateImage(cvGetSize(frame), 8, 1);

    while((successes < chessboardsNumber) && (calibrating)) {

        frame = imageBuffer->getFrame();


        if(takeSnapshot){

            int found = cvFindChessboardCorners(frame, chessboardSize, corners, &cornerCount, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );

            cvCvtColor(frame, grayImage, CV_BGR2GRAY);
            cvFindCornerSubPix(grayImage, corners, cornerCount, cvSize(11, 11), cvSize(-1, -1), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

            cvDrawChessboardCorners(frame, chessboardSize, corners, cornerCount, found);

            if( cornerCount == cornersNumber ){
                step = successes*cornersNumber;
                for( int i=step, j=0; j < cornersNumber; ++i, ++j ){
                    CV_MAT_ELEM( *imagePoints, float, i, 0 ) = corners[j].x;
                    CV_MAT_ELEM( *imagePoints, float, i, 1 ) = corners[j].y;
                    CV_MAT_ELEM( *objectPoints, float, i, 0 ) = (float)(j/boardWidth)*realSquareSize;
                    CV_MAT_ELEM( *objectPoints, float, i, 1 ) = (float)(j%boardWidth)*realSquareSize;
                    CV_MAT_ELEM( *objectPoints, float, i, 2 ) = 0.0f;
                }
                CV_MAT_ELEM( *pointCounts, int, successes, 0 ) = cornersNumber;
                successes++;
            }
        }

        cvPutText (frame,"Left click to take snapshot.",cvPoint(200,50), &ftext, BLUE);
        sprintf(buffer,"Successes: %d / %d",successes,chessboardsNumber);
        cvPutText (frame,buffer,cvPoint(200,75), &ftext, BLUE);

        cvPutText (frame,"Calibrating",cvPoint(20,20), &status, WHITE);

        if(rootFilter) {
            rootFilter->processPoint(frame);
        }

        if(takeSnapshot){
            QThread::sleep(1);
            takeSnapshot = false;
        }
    }

    if(calibrating) {
        cvZero(intrinsicMatrix);
        cvZero(distortionMatrix);

        cvCalibrateCamera2(objectPoints, imagePoints, pointCounts, cvGetSize(frame),intrinsicMatrix, distortionMatrix, rotationMatrix, translationMatrix, CV_CALIB_FIX_K3);

        cvSave(process.value("intrinsic").toString().toAscii(),intrinsicMatrix);
        cvSave(process.value("distortion").toString().toAscii(),distortionMatrix);
        //QMessageBox::information(parent(), "Camera Calibration", "Calibration has finished.");

        emit finale();
        QThread::sleep(1);
        calibrating = false;
    }
}

void ProcessingThread::Perspectivefix(){

    mousePosition.x = 0;
    mousePosition.y = 0;

    QString str;
    QStringList strlist;
    CvPoint points[4];

    str = process.value("point1").toString();
    strlist = str.split(",");
    points[0] = cvPoint(strlist[0].toInt(),strlist[1].toInt());

    str = process.value("point2").toString();
    strlist = str.split(",");
    points[1] = cvPoint(strlist[0].toInt(),strlist[1].toInt());

    str = process.value("point3").toString();
    strlist = str.split(",");
    points[2] = cvPoint(strlist[0].toInt(),strlist[1].toInt());

    str = process.value("point4").toString();
    strlist = str.split(",");
    points[3] = cvPoint(strlist[0].toInt(),strlist[1].toInt());


    char buffer [40];

    IplImage *frame = NULL;
    IplImage *calibratedImage = NULL;

    CvMat *mapX = NULL;
    CvMat *mapY = NULL;
    CvMat *intrinsicMatrix = NULL;
    CvMat *distortionMatrix = NULL;
    CvMat* perspectiveMatrix = NULL;

    int selectedPoints = 0;

    CvPoint2D32f newPoints[4];

    for (int i=0; i<4; i++){
        newPoints[i].x = 0;
        newPoints[i].y = 0;
    }


    frame = imageBuffer->getFrame();

    calibratedImage = cvCreateImage(cvGetSize(frame), frame->depth, frame->nChannels);

    mapX = cvCreateMat( frame->height, frame->width, CV_32FC1 );
    mapY = cvCreateMat( frame->height, frame->width, CV_32FC1 );

    intrinsicMatrix = (CvMat*)cvLoad(process.value("intrinsic").toString().toAscii());
    distortionMatrix = (CvMat*)cvLoad(process.value("distortion").toString().toAscii());

    cvInitUndistortMap(intrinsicMatrix,distortionMatrix,mapX,mapY);


    while ((selectedPoints < 4)&&(perspfixing)){

        frame = imageBuffer->getFrame();
        cvRemap(frame,calibratedImage,mapX,mapY);

        sprintf(buffer,"Select point%d (%d,%d)",(selectedPoints + 1),points[selectedPoints].x,points[selectedPoints].y);
        cvPutText (calibratedImage,buffer,cvPoint(200,50), &ftext, BLUE);

        if ( mousePosition.x !=0 && mousePosition.y !=0 ){
            newPoints[selectedPoints].x = (float)mousePosition.x;
            newPoints[selectedPoints].y = (float)mousePosition.y;

            mousePosition.x = 0;
            mousePosition.y = 0;

            selectedPoints++;
        }

        for (int i=0; i<4; i++){
            if (newPoints[i].x != 0 && newPoints[i].y != 0){
                cvCircle(calibratedImage,cvPoint((int)newPoints[i].x,(int)newPoints[i].y),10,BLUE,1);
                cvLine(calibratedImage,cvPoint((int)newPoints[i].x,(int)newPoints[i].y),cvPoint((int)newPoints[i].x,(int)newPoints[i].y),BLUE,1);
            }

            if (i != 0){
                if (newPoints[i].x != 0 && newPoints[i].y != 0)
                    cvLine(calibratedImage, cvPoint((int)newPoints[i-1].x,(int)newPoints[i-1].y), cvPoint((int)newPoints[i].x,(int)newPoints[i].y), GREEN, 1);
            }
        }

        if (selectedPoints == 4)
            cvLine(calibratedImage, cvPoint((int)newPoints[0].x,(int)newPoints[0].y), cvPoint((int)newPoints[3].x,(int)newPoints[3].y), GREEN, 1);


        cvPutText (calibratedImage,"Perspective Fixing",cvPoint(20,20), &status, WHITE);

        if(rootFilter)
            rootFilter->processPoint(calibratedImage);

    }

    if(perspfixing){

        perspectiveMatrix = cvCreateMat(3,3,CV_32FC1);

        CvPoint2D32f c1[4];
        CvPoint2D32f c2[4];

        c1[0] = cvPoint2D32f (newPoints[0].x, newPoints[0].y);
        c1[1] = cvPoint2D32f (newPoints[1].x, newPoints[1].y);
        c1[2] = cvPoint2D32f (newPoints[2].x, newPoints[2].y);
        c1[3] = cvPoint2D32f (newPoints[3].x, newPoints[3].y);

        c2[0] = cvPoint2D32f (points[0].x, points[0].y);
        c2[1] = cvPoint2D32f (points[1].x, points[1].y);
        c2[2] = cvPoint2D32f (points[2].x, points[2].y);
        c2[3] = cvPoint2D32f (points[3].x, points[3].y);

        cvGetPerspectiveTransform(c1, c2, perspectiveMatrix);

        cvSave(process.value("perspective").toString().toAscii(),perspectiveMatrix);

        emit finale();
        QThread::sleep(1);
        perspfixing = false;

    }

}

void ProcessingThread::livefeed(){

    IplImage* frame = imageBuffer->getFrame();

    cvPutText (frame,"Live Feed",cvPoint(20,20), &status, WHITE);

    if(rootFilter) {
        rootFilter->processPoint(frame);
    }

    cvReleaseImage(&frame);

}

vector<CvPoint> ProcessingThread::getRobotPath() {
    return robotPath;
}
