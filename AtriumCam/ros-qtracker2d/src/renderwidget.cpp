#include "ros-qtracker2d/renderwidget.hpp"
#include <QPainter>
#include <QDebug>
#include <QTime>
#include <QString>
#include <QPen>


RenderWidget::RenderWidget(QWidget* parent) : QWidget(parent), Filter(), imageData(0), imageWidth(0), imageHeight(0) {
    setAttribute(Qt::WA_OpaquePaintEvent, true); // don't clear the area before the paintEvent
    setAttribute(Qt::WA_PaintOnScreen, true); // disable double buffering
    //setFixedSize(imageWidth, imageHeight);
    connect(this, SIGNAL(frameSizeChanged(int, int)), this, SLOT(onFrameSizeChanged(int, int)), Qt::QueuedConnection);
    time = frames = 0;
}

void RenderWidget::onFrameSizeChanged(int width, int height) {
    setFixedSize(width, height);
} 

void RenderWidget::updatePixmap(const IplImage* frame) {
    QTime t;
    t.start();
    bool start = false;
    // check if the frame dimensions have changed
    if(frame->width != imageWidth || frame->height != imageHeight) {
        if(imageData) {
            delete[] imageData;
        }
        start = true;
        imageWidth = frame->width;
        imageHeight = frame->height;
        emit(frameSizeChanged(imageWidth, imageHeight));
        imageData = new unsigned char[4*imageWidth*imageHeight];
        for(int i = 0; i < imageWidth*imageHeight; i++) {
            imageData[i*4+3] = 0xFF;
        }
    }

    int pixels = imageWidth * imageHeight;
    uchar* src = (uchar*)(frame->imageData);
    uchar* srcEnd = src + (3*pixels);
    uchar* dest = imageData;

    do {
        memcpy(dest, src, 3);
        dest += 4;
        src += 3;
    } while(src < srcEnd);

    if(!start) {
        ++frames;
        time += t.elapsed();
    }
}

void RenderWidget::processPoint(IplImage* image) {
    // copy the image to the local pixmap and update the display
    updatePixmap(image);
    update();

    notifyListener(image);
}

void RenderWidget::paintEvent(QPaintEvent*) {
    //	QTime time;
    //	time.start();
    QPainter painter(this);
    if(imageData) {
        //QPixmap temp;
	//	qDebug() << temp.loadFromData(imageData, imageWidth*imageHeight*4);
        //qDebug() << "loading...";
        QImage tImg(imageData, imageWidth, imageHeight, QImage::Format_RGB32);
        painter.drawImage(QPoint(0,0), tImg);
    }
    else {
        painter.setBrush(Qt::black);
        painter.drawRect(rect());
    }
}
