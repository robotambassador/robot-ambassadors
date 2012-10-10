#include <QtGui/QApplication>
#include "ros-qtracker2d/mainwin.hpp"

int main(int argc, char *argv[]) {    

    QApplication app(argc, argv);
    mainwin *mainWin = new mainwin(argc, argv);
    mainWin->show();
    return app.exec();
}

