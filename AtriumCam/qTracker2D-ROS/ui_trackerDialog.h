/********************************************************************************
** Form generated from reading UI file 'trackerDialog.ui'
**
** Created: Sun Nov 20 16:56:41 2011
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TRACKERDIALOG_H
#define UI_TRACKERDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QToolButton>
#include <QtGui/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_trackerDialog
{
public:
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout_5;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QSpacerItem *horizontalSpacer;
    QLineEdit *searchSize;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QSpacerItem *horizontalSpacer_2;
    QLineEdit *templateSize;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_3;
    QSpacerItem *horizontalSpacer_3;
    QLineEdit *threshold;
    QHBoxLayout *horizontalLayout_7;
    QCheckBox *useCalibration;
    QSpacerItem *horizontalSpacer_4;
    QCheckBox *usePerspective;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_4;
    QSpacerItem *horizontalSpacer_5;
    QLineEdit *file_num;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_5;
    QSpacerItem *horizontalSpacer_15;
    QLineEdit *output;
    QToolButton *outputButton;
    QHBoxLayout *horizontalLayout_8;
    QSpacerItem *horizontalSpacer_6;
    QCheckBox *saveVideo;
    QSpacerItem *horizontalSpacer_7;
    QGroupBox *groupBox_3;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_6;
    QSpacerItem *horizontalSpacer_8;
    QSpinBox *obj_num;
    QGroupBox *groupBox_4;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *obj1;
    QLabel *label_7;
    QSpacerItem *horizontalSpacer_9;
    QLineEdit *nObj1;
    QHBoxLayout *obj2;
    QLabel *label_8;
    QSpacerItem *horizontalSpacer_10;
    QLineEdit *nObj2;
    QHBoxLayout *obj3;
    QLabel *label_9;
    QSpacerItem *horizontalSpacer_11;
    QLineEdit *nObj3;
    QHBoxLayout *obj4;
    QLabel *label_10;
    QSpacerItem *horizontalSpacer_12;
    QLineEdit *nObj4;
    QHBoxLayout *obj5;
    QLabel *label_11;
    QSpacerItem *horizontalSpacer_13;
    QLineEdit *nObj5;
    QHBoxLayout *obj6;
    QLabel *label_12;
    QSpacerItem *horizontalSpacer_14;
    QLineEdit *nObj6;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *trackerDialog)
    {
        if (trackerDialog->objectName().isEmpty())
            trackerDialog->setObjectName(QString::fromUtf8("trackerDialog"));
        trackerDialog->resize(620, 380);
        gridLayout = new QGridLayout(trackerDialog);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        groupBox = new QGroupBox(trackerDialog);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        verticalLayout = new QVBoxLayout(groupBox);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout->addWidget(label);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        searchSize = new QLineEdit(groupBox);
        searchSize->setObjectName(QString::fromUtf8("searchSize"));

        horizontalLayout->addWidget(searchSize);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_2->addWidget(label_2);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);

        templateSize = new QLineEdit(groupBox);
        templateSize->setObjectName(QString::fromUtf8("templateSize"));

        horizontalLayout_2->addWidget(templateSize);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_5->addWidget(label_3);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_3);

        threshold = new QLineEdit(groupBox);
        threshold->setObjectName(QString::fromUtf8("threshold"));

        horizontalLayout_5->addWidget(threshold);


        verticalLayout->addLayout(horizontalLayout_5);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        useCalibration = new QCheckBox(groupBox);
        useCalibration->setObjectName(QString::fromUtf8("useCalibration"));
        useCalibration->setLayoutDirection(Qt::RightToLeft);

        horizontalLayout_7->addWidget(useCalibration);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_7->addItem(horizontalSpacer_4);

        usePerspective = new QCheckBox(groupBox);
        usePerspective->setObjectName(QString::fromUtf8("usePerspective"));
        usePerspective->setLayoutDirection(Qt::RightToLeft);

        horizontalLayout_7->addWidget(usePerspective);


        verticalLayout->addLayout(horizontalLayout_7);


        verticalLayout_5->addWidget(groupBox);

        groupBox_2 = new QGroupBox(trackerDialog);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        verticalLayout_2 = new QVBoxLayout(groupBox_2);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_4 = new QLabel(groupBox_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_4->addWidget(label_4);

        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_5);

        file_num = new QLineEdit(groupBox_2);
        file_num->setObjectName(QString::fromUtf8("file_num"));

        horizontalLayout_4->addWidget(file_num);


        verticalLayout_2->addLayout(horizontalLayout_4);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout_6->addWidget(label_5);

        horizontalSpacer_15 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_6->addItem(horizontalSpacer_15);

        output = new QLineEdit(groupBox_2);
        output->setObjectName(QString::fromUtf8("output"));

        horizontalLayout_6->addWidget(output);

        outputButton = new QToolButton(groupBox_2);
        outputButton->setObjectName(QString::fromUtf8("outputButton"));

        horizontalLayout_6->addWidget(outputButton);


        verticalLayout_2->addLayout(horizontalLayout_6);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_8->addItem(horizontalSpacer_6);

        saveVideo = new QCheckBox(groupBox_2);
        saveVideo->setObjectName(QString::fromUtf8("saveVideo"));
        saveVideo->setLayoutDirection(Qt::RightToLeft);

        horizontalLayout_8->addWidget(saveVideo);

        horizontalSpacer_7 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_8->addItem(horizontalSpacer_7);


        verticalLayout_2->addLayout(horizontalLayout_8);


        verticalLayout_5->addWidget(groupBox_2);


        gridLayout->addLayout(verticalLayout_5, 0, 0, 1, 1);

        groupBox_3 = new QGroupBox(trackerDialog);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        verticalLayout_4 = new QVBoxLayout(groupBox_3);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        label_6 = new QLabel(groupBox_3);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        horizontalLayout_9->addWidget(label_6);

        horizontalSpacer_8 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_9->addItem(horizontalSpacer_8);

        obj_num = new QSpinBox(groupBox_3);
        obj_num->setObjectName(QString::fromUtf8("obj_num"));
        obj_num->setMinimum(1);
        obj_num->setMaximum(6);

        horizontalLayout_9->addWidget(obj_num);


        verticalLayout_4->addLayout(horizontalLayout_9);

        groupBox_4 = new QGroupBox(groupBox_3);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        verticalLayout_3 = new QVBoxLayout(groupBox_4);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        obj1 = new QHBoxLayout();
        obj1->setObjectName(QString::fromUtf8("obj1"));
        label_7 = new QLabel(groupBox_4);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        obj1->addWidget(label_7);

        horizontalSpacer_9 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        obj1->addItem(horizontalSpacer_9);

        nObj1 = new QLineEdit(groupBox_4);
        nObj1->setObjectName(QString::fromUtf8("nObj1"));

        obj1->addWidget(nObj1);


        verticalLayout_3->addLayout(obj1);

        obj2 = new QHBoxLayout();
        obj2->setObjectName(QString::fromUtf8("obj2"));
        label_8 = new QLabel(groupBox_4);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        obj2->addWidget(label_8);

        horizontalSpacer_10 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        obj2->addItem(horizontalSpacer_10);

        nObj2 = new QLineEdit(groupBox_4);
        nObj2->setObjectName(QString::fromUtf8("nObj2"));

        obj2->addWidget(nObj2);


        verticalLayout_3->addLayout(obj2);

        obj3 = new QHBoxLayout();
        obj3->setObjectName(QString::fromUtf8("obj3"));
        label_9 = new QLabel(groupBox_4);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        obj3->addWidget(label_9);

        horizontalSpacer_11 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        obj3->addItem(horizontalSpacer_11);

        nObj3 = new QLineEdit(groupBox_4);
        nObj3->setObjectName(QString::fromUtf8("nObj3"));

        obj3->addWidget(nObj3);


        verticalLayout_3->addLayout(obj3);

        obj4 = new QHBoxLayout();
        obj4->setObjectName(QString::fromUtf8("obj4"));
        label_10 = new QLabel(groupBox_4);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        obj4->addWidget(label_10);

        horizontalSpacer_12 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        obj4->addItem(horizontalSpacer_12);

        nObj4 = new QLineEdit(groupBox_4);
        nObj4->setObjectName(QString::fromUtf8("nObj4"));

        obj4->addWidget(nObj4);


        verticalLayout_3->addLayout(obj4);

        obj5 = new QHBoxLayout();
        obj5->setObjectName(QString::fromUtf8("obj5"));
        label_11 = new QLabel(groupBox_4);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        obj5->addWidget(label_11);

        horizontalSpacer_13 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        obj5->addItem(horizontalSpacer_13);

        nObj5 = new QLineEdit(groupBox_4);
        nObj5->setObjectName(QString::fromUtf8("nObj5"));

        obj5->addWidget(nObj5);


        verticalLayout_3->addLayout(obj5);

        obj6 = new QHBoxLayout();
        obj6->setObjectName(QString::fromUtf8("obj6"));
        label_12 = new QLabel(groupBox_4);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        obj6->addWidget(label_12);

        horizontalSpacer_14 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        obj6->addItem(horizontalSpacer_14);

        nObj6 = new QLineEdit(groupBox_4);
        nObj6->setObjectName(QString::fromUtf8("nObj6"));

        obj6->addWidget(nObj6);


        verticalLayout_3->addLayout(obj6);


        verticalLayout_4->addWidget(groupBox_4);


        gridLayout->addWidget(groupBox_3, 0, 1, 1, 1);

        buttonBox = new QDialogButtonBox(trackerDialog);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setEnabled(true);
        buttonBox->setLayoutDirection(Qt::RightToLeft);
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Save);
        buttonBox->setCenterButtons(true);

        gridLayout->addWidget(buttonBox, 1, 0, 1, 2);

#ifndef QT_NO_SHORTCUT
        label->setBuddy(searchSize);
        label_2->setBuddy(templateSize);
        label_3->setBuddy(threshold);
        label_4->setBuddy(file_num);
        label_5->setBuddy(output);
        label_6->setBuddy(obj_num);
        label_7->setBuddy(nObj1);
        label_8->setBuddy(nObj2);
        label_9->setBuddy(nObj3);
        label_10->setBuddy(nObj4);
        label_11->setBuddy(nObj5);
        label_12->setBuddy(nObj6);
#endif // QT_NO_SHORTCUT

        retranslateUi(trackerDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), trackerDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), trackerDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(trackerDialog);
    } // setupUi

    void retranslateUi(QDialog *trackerDialog)
    {
        trackerDialog->setWindowTitle(QApplication::translate("trackerDialog", "Tracker Settings", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("trackerDialog", "Tracker Settings", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("trackerDialog", "S&earch Size:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        searchSize->setToolTip(QApplication::translate("trackerDialog", "The size of the search image. It must be between 6 and 50.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_2->setText(QApplication::translate("trackerDialog", "&Template Size:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        templateSize->setToolTip(QApplication::translate("trackerDialog", "The size of the template image. It must be between 5 and 40.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        templateSize->setStatusTip(QString());
#endif // QT_NO_STATUSTIP
        label_3->setText(QApplication::translate("trackerDialog", "T&hreshold:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        threshold->setToolTip(QApplication::translate("trackerDialog", "The threshold of the template match. It must be between 0 and 10.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_TOOLTIP
        useCalibration->setToolTip(QApplication::translate("trackerDialog", "Use camera calibration.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        useCalibration->setText(QApplication::translate("trackerDialog", "Use C&alibration", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        usePerspective->setToolTip(QApplication::translate("trackerDialog", "Use camera perspective.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        usePerspective->setText(QApplication::translate("trackerDialog", "Use &Perspective", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("trackerDialog", "Output", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("trackerDialog", "File &Number:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        file_num->setToolTip(QApplication::translate("trackerDialog", "The number of the data file.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_5->setText(QApplication::translate("trackerDialog", "Output &Directory:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        output->setToolTip(QApplication::translate("trackerDialog", "The folder to save the data file.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        output->setStatusTip(QApplication::translate("trackerDialog", "The folder to save the data file.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
#ifndef QT_NO_TOOLTIP
        outputButton->setToolTip(QApplication::translate("trackerDialog", "open...", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        outputButton->setText(QApplication::translate("trackerDialog", "...", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        saveVideo->setToolTip(QApplication::translate("trackerDialog", "Save the tracking sequence in a video.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        saveVideo->setText(QApplication::translate("trackerDialog", "Save &Video Capture", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("trackerDialog", "Tracked Objected", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("trackerDialog", "Number of &Objects:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        obj_num->setToolTip(QApplication::translate("trackerDialog", "The number of the objects to track. It must be between 1 and 6.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        groupBox_4->setTitle(QApplication::translate("trackerDialog", "Objects' Name", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("trackerDialog", "Object &1:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        nObj1->setToolTip(QApplication::translate("trackerDialog", "Set name to object 1.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        nObj1->setStatusTip(QString());
#endif // QT_NO_STATUSTIP
        label_8->setText(QApplication::translate("trackerDialog", "Object &2:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        nObj2->setToolTip(QApplication::translate("trackerDialog", "Set name to object 2.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        nObj2->setStatusTip(QString());
#endif // QT_NO_STATUSTIP
        label_9->setText(QApplication::translate("trackerDialog", "Object &3:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        nObj3->setToolTip(QApplication::translate("trackerDialog", "Set name to object 3.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_10->setText(QApplication::translate("trackerDialog", "Object &4:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        nObj4->setToolTip(QApplication::translate("trackerDialog", "Set name to object 4.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_11->setText(QApplication::translate("trackerDialog", "Object &5:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        nObj5->setToolTip(QApplication::translate("trackerDialog", "Set name to object 5.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_12->setText(QApplication::translate("trackerDialog", "Object &6:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        nObj6->setToolTip(QApplication::translate("trackerDialog", "Set name to object 6.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
    } // retranslateUi

};

namespace Ui {
    class trackerDialog: public Ui_trackerDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TRACKERDIALOG_H
