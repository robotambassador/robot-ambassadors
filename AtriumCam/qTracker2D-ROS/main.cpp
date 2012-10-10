#include <QtGui/QApplication>
#include "mainwin.h"

int main(int argc, char *argv[]) {    

    QApplication app(argc, argv);
    mainwin *mainWin = new mainwin(argc, argv);
    mainWin->show();
    return app.exec();
}

