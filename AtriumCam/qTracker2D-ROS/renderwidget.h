#ifndef RENDER_WIDGET_H
#define RENDER_WIDGET_H

#include <QWidget>
#include "filter.h"
#include "opencv/highgui.h"


class RenderWidget : public QWidget, public Filter {
    Q_OBJECT;
public:
    RenderWidget(QWidget* parent);
    void processPoint(IplImage* image);
public slots:
    void onFrameSizeChanged(int width, int height);
signals:
    void frameSizeChanged(int width, int height);
protected:
    void paintEvent(QPaintEvent*);
private:
    void updatePixmap(const IplImage* frame);
    QPixmap bufferPixmap;
    uchar* imageData;
    int imageWidth, imageHeight;
    int time;
    int frames;
};

#endif
