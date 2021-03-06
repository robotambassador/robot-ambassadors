#ifndef MAINWIN_H
#define MAINWIN_H

#include <QMainWindow>
#include "ui_mainwin.h"
#include "initdialog.h"
#include "calibrationDialog.h"
#include "perspectiveDialog.h"
#include "trackerDialog.h"
#include "renderwidget.h"
#include "controller.h"

#include <phonon/mediaobject.h>
#include <phonon/audiooutput.h>

class RenderWidget;

class mainwin : public QMainWindow, public Ui::mainwin {

    Q_OBJECT
public:
    mainwin(int argc, char *argv[]);
private slots:
    void actTrack();
    void actCalibrate();
    void actPerspective();
    void set_track();
    void set_calibrate();
    void set_perspective();
    void about();
    void updateStats();
    void end_signal();
    void play();
    void finished_play();
protected:
    void closeEvent(QCloseEvent*);
    void mousePressEvent( QMouseEvent *e );
    void keyPressEvent (QKeyEvent *);
private:
    int cameraWidth;
    int cameraHeight;
    bool videoinput;
    RenderWidget* renderWidget;
    controller* Controller;
    QTimer* updateTimer;    
    Phonon::MediaObject *mediaObject;


    void disable(int m);
    void enable();
    void setCameraWH(int idx);

};
#endif // MAINWIN_H
