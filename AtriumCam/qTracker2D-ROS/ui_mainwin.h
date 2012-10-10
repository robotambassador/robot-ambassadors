/********************************************************************************
** Form generated from reading UI file 'mainwin.ui'
**
** Created: Sun Nov 20 16:56:41 2011
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWIN_H
#define UI_MAINWIN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QStatusBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_mainwin
{
public:
    QAction *actionTrack;
    QAction *actionCalibrate;
    QAction *actionPerspective;
    QAction *actionExit;
    QAction *actionTracker;
    QAction *actionCalibration;
    QAction *actionPerspective_2;
    QAction *actionTracker_Help;
    QAction *actionAbout;
    QAction *actionAboutQT;
    QAction *actionPlay_Cricket_Song;
    QWidget *mainwidget;
    QMenuBar *menubar;
    QMenu *menuMenu;
    QMenu *menuSettings;
    QMenu *menuHelp;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *mainwin)
    {
        if (mainwin->objectName().isEmpty())
            mainwin->setObjectName(QString::fromUtf8("mainwin"));
        mainwin->resize(731, 509);
        actionTrack = new QAction(mainwin);
        actionTrack->setObjectName(QString::fromUtf8("actionTrack"));
        actionCalibrate = new QAction(mainwin);
        actionCalibrate->setObjectName(QString::fromUtf8("actionCalibrate"));
        actionPerspective = new QAction(mainwin);
        actionPerspective->setObjectName(QString::fromUtf8("actionPerspective"));
        actionExit = new QAction(mainwin);
        actionExit->setObjectName(QString::fromUtf8("actionExit"));
        actionTracker = new QAction(mainwin);
        actionTracker->setObjectName(QString::fromUtf8("actionTracker"));
        actionTracker->setChecked(false);
        actionCalibration = new QAction(mainwin);
        actionCalibration->setObjectName(QString::fromUtf8("actionCalibration"));
        actionPerspective_2 = new QAction(mainwin);
        actionPerspective_2->setObjectName(QString::fromUtf8("actionPerspective_2"));
        actionTracker_Help = new QAction(mainwin);
        actionTracker_Help->setObjectName(QString::fromUtf8("actionTracker_Help"));
        actionAbout = new QAction(mainwin);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        actionAboutQT = new QAction(mainwin);
        actionAboutQT->setObjectName(QString::fromUtf8("actionAboutQT"));
        actionPlay_Cricket_Song = new QAction(mainwin);
        actionPlay_Cricket_Song->setObjectName(QString::fromUtf8("actionPlay_Cricket_Song"));
        actionPlay_Cricket_Song->setCheckable(true);
        actionPlay_Cricket_Song->setChecked(true);
        mainwidget = new QWidget(mainwin);
        mainwidget->setObjectName(QString::fromUtf8("mainwidget"));
        mainwidget->setMouseTracking(true);
        mainwin->setCentralWidget(mainwidget);
        menubar = new QMenuBar(mainwin);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 731, 23));
        menuMenu = new QMenu(menubar);
        menuMenu->setObjectName(QString::fromUtf8("menuMenu"));
        menuSettings = new QMenu(menubar);
        menuSettings->setObjectName(QString::fromUtf8("menuSettings"));
        menuHelp = new QMenu(menubar);
        menuHelp->setObjectName(QString::fromUtf8("menuHelp"));
        mainwin->setMenuBar(menubar);
        statusbar = new QStatusBar(mainwin);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        mainwin->setStatusBar(statusbar);

        menubar->addAction(menuMenu->menuAction());
        menubar->addAction(menuSettings->menuAction());
        menubar->addAction(menuHelp->menuAction());
        menuMenu->addAction(actionTrack);
        menuMenu->addAction(actionCalibrate);
        menuMenu->addAction(actionPerspective);
        menuMenu->addSeparator();
        menuMenu->addAction(actionExit);
        menuSettings->addAction(actionPlay_Cricket_Song);
        menuSettings->addSeparator();
        menuSettings->addAction(actionTracker);
        menuSettings->addAction(actionCalibration);
        menuSettings->addAction(actionPerspective_2);
        menuHelp->addAction(actionTracker_Help);
        menuHelp->addSeparator();
        menuHelp->addAction(actionAbout);
        menuHelp->addAction(actionAboutQT);

        retranslateUi(mainwin);
        QObject::connect(actionExit, SIGNAL(triggered()), mainwin, SLOT(close()));

        QMetaObject::connectSlotsByName(mainwin);
    } // setupUi

    void retranslateUi(QMainWindow *mainwin)
    {
        mainwin->setWindowTitle(QApplication::translate("mainwin", "QTracker2D", 0, QApplication::UnicodeUTF8));
        actionTrack->setText(QApplication::translate("mainwin", "Start &Tracking", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionTrack->setToolTip(QApplication::translate("mainwin", "Start Tracking", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionTrack->setShortcut(QApplication::translate("mainwin", "Ctrl+T", 0, QApplication::UnicodeUTF8));
        actionCalibrate->setText(QApplication::translate("mainwin", "Start &Calibration", 0, QApplication::UnicodeUTF8));
        actionCalibrate->setShortcut(QApplication::translate("mainwin", "Ctrl+C", 0, QApplication::UnicodeUTF8));
        actionPerspective->setText(QApplication::translate("mainwin", "Fix &Perspective", 0, QApplication::UnicodeUTF8));
        actionPerspective->setShortcut(QApplication::translate("mainwin", "Ctrl+P", 0, QApplication::UnicodeUTF8));
        actionExit->setText(QApplication::translate("mainwin", "Exit", 0, QApplication::UnicodeUTF8));
        actionExit->setShortcut(QApplication::translate("mainwin", "Ctrl+Q", 0, QApplication::UnicodeUTF8));
        actionTracker->setText(QApplication::translate("mainwin", "Tracker", 0, QApplication::UnicodeUTF8));
        actionCalibration->setText(QApplication::translate("mainwin", "Calibration", 0, QApplication::UnicodeUTF8));
        actionPerspective_2->setText(QApplication::translate("mainwin", "Perspective", 0, QApplication::UnicodeUTF8));
        actionTracker_Help->setText(QApplication::translate("mainwin", "Tracker H&elp", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("mainwin", "&About", 0, QApplication::UnicodeUTF8));
        actionAboutQT->setText(QApplication::translate("mainwin", "About QT", 0, QApplication::UnicodeUTF8));
        actionPlay_Cricket_Song->setText(QApplication::translate("mainwin", "Play Cricket Song", 0, QApplication::UnicodeUTF8));
        menuMenu->setTitle(QApplication::translate("mainwin", "&Menu", 0, QApplication::UnicodeUTF8));
        menuSettings->setTitle(QApplication::translate("mainwin", "&Settings", 0, QApplication::UnicodeUTF8));
        menuHelp->setTitle(QApplication::translate("mainwin", "&Help", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class mainwin: public Ui_mainwin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWIN_H
