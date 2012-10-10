/********************************************************************************
** Form generated from reading UI file 'calibrationDialog.ui'
**
** Created: Sun Nov 20 16:56:41 2011
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CALIBRATIONDIALOG_H
#define UI_CALIBRATIONDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QToolButton>
#include <QtGui/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_calibrationDialog
{
public:
    QGridLayout *gridLayout;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QSpacerItem *horizontalSpacer;
    QLineEdit *chessnum;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QSpacerItem *horizontalSpacer_2;
    QLineEdit *realsize;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_3;
    QSpacerItem *horizontalSpacer_3;
    QLineEdit *sqr_p_col;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_4;
    QSpacerItem *horizontalSpacer_4;
    QLineEdit *sqr_p_row;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_6;
    QSpacerItem *horizontalSpacer_10;
    QLineEdit *distortion;
    QToolButton *distortionButton;
    QPushButton *distortionSvAs;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_5;
    QSpacerItem *horizontalSpacer_9;
    QLineEdit *intrinsic;
    QToolButton *intrinsicButton;
    QPushButton *intrinsicSvAs;
    QSpacerItem *verticalSpacer;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *calibrationDialog)
    {
        if (calibrationDialog->objectName().isEmpty())
            calibrationDialog->setObjectName(QString::fromUtf8("calibrationDialog"));
        calibrationDialog->resize(420, 374);
        calibrationDialog->setLayoutDirection(Qt::RightToLeft);
        gridLayout = new QGridLayout(calibrationDialog);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        groupBox = new QGroupBox(calibrationDialog);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setLayoutDirection(Qt::LeftToRight);
        verticalLayout = new QVBoxLayout(groupBox);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout->addWidget(label);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        chessnum = new QLineEdit(groupBox);
        chessnum->setObjectName(QString::fromUtf8("chessnum"));

        horizontalLayout->addWidget(chessnum);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_2->addWidget(label_2);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);

        realsize = new QLineEdit(groupBox);
        realsize->setObjectName(QString::fromUtf8("realsize"));

        horizontalLayout_2->addWidget(realsize);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_3->addWidget(label_3);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_3);

        sqr_p_col = new QLineEdit(groupBox);
        sqr_p_col->setObjectName(QString::fromUtf8("sqr_p_col"));

        horizontalLayout_3->addWidget(sqr_p_col);


        verticalLayout->addLayout(horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_4->addWidget(label_4);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_4);

        sqr_p_row = new QLineEdit(groupBox);
        sqr_p_row->setObjectName(QString::fromUtf8("sqr_p_row"));

        horizontalLayout_4->addWidget(sqr_p_row);


        verticalLayout->addLayout(horizontalLayout_4);


        gridLayout->addWidget(groupBox, 0, 0, 1, 1);

        groupBox_2 = new QGroupBox(calibrationDialog);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setLayoutDirection(Qt::LeftToRight);
        verticalLayout_2 = new QVBoxLayout(groupBox_2);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_6 = new QLabel(groupBox_2);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        horizontalLayout_5->addWidget(label_6);

        horizontalSpacer_10 = new QSpacerItem(18, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_10);

        distortion = new QLineEdit(groupBox_2);
        distortion->setObjectName(QString::fromUtf8("distortion"));
        distortion->setMaximumSize(QSize(148, 27));

        horizontalLayout_5->addWidget(distortion);

        distortionButton = new QToolButton(groupBox_2);
        distortionButton->setObjectName(QString::fromUtf8("distortionButton"));
        distortionButton->setMaximumSize(QSize(16777215, 27));

        horizontalLayout_5->addWidget(distortionButton);

        distortionSvAs = new QPushButton(groupBox_2);
        distortionSvAs->setObjectName(QString::fromUtf8("distortionSvAs"));

        horizontalLayout_5->addWidget(distortionSvAs);


        verticalLayout_2->addLayout(horizontalLayout_5);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout_6->addWidget(label_5);

        horizontalSpacer_9 = new QSpacerItem(18, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_6->addItem(horizontalSpacer_9);

        intrinsic = new QLineEdit(groupBox_2);
        intrinsic->setObjectName(QString::fromUtf8("intrinsic"));

        horizontalLayout_6->addWidget(intrinsic);

        intrinsicButton = new QToolButton(groupBox_2);
        intrinsicButton->setObjectName(QString::fromUtf8("intrinsicButton"));

        horizontalLayout_6->addWidget(intrinsicButton);

        intrinsicSvAs = new QPushButton(groupBox_2);
        intrinsicSvAs->setObjectName(QString::fromUtf8("intrinsicSvAs"));

        horizontalLayout_6->addWidget(intrinsicSvAs);


        verticalLayout_2->addLayout(horizontalLayout_6);


        gridLayout->addWidget(groupBox_2, 1, 0, 1, 1);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer, 2, 0, 1, 1);

        buttonBox = new QDialogButtonBox(calibrationDialog);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Save);
        buttonBox->setCenterButtons(true);

        gridLayout->addWidget(buttonBox, 3, 0, 1, 1);

#ifndef QT_NO_SHORTCUT
        label->setBuddy(chessnum);
        label_2->setBuddy(realsize);
        label_3->setBuddy(sqr_p_col);
        label_4->setBuddy(sqr_p_row);
        label_6->setBuddy(distortion);
        label_5->setBuddy(intrinsic);
#endif // QT_NO_SHORTCUT

        retranslateUi(calibrationDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), calibrationDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), calibrationDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(calibrationDialog);
    } // setupUi

    void retranslateUi(QDialog *calibrationDialog)
    {
        calibrationDialog->setWindowTitle(QApplication::translate("calibrationDialog", "Calibration Settings", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        calibrationDialog->setStatusTip(QApplication::translate("calibrationDialog", "Calibration Settings", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        groupBox->setTitle(QApplication::translate("calibrationDialog", "Chessboard", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("calibrationDialog", "&Number of snapshots :", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        chessnum->setToolTip(QApplication::translate("calibrationDialog", "Number of chessboards. It must be between 5 and 40", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        chessnum->setStatusTip(QApplication::translate("calibrationDialog", "Number of chessboards. It must be between 5 and 40", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        label_2->setText(QApplication::translate("calibrationDialog", "Real S&quare size :", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        realsize->setToolTip(QApplication::translate("calibrationDialog", "Real square size in mm", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        realsize->setStatusTip(QApplication::translate("calibrationDialog", "Real square size in mm", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        label_3->setText(QApplication::translate("calibrationDialog", "Squares per C&olumn :", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        sqr_p_col->setToolTip(QApplication::translate("calibrationDialog", "Number of squares per column", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        sqr_p_col->setStatusTip(QApplication::translate("calibrationDialog", "Number of squares per column", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        label_4->setText(QApplication::translate("calibrationDialog", "Squares per &Row :", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        sqr_p_row->setToolTip(QApplication::translate("calibrationDialog", "Number of squares per row", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        sqr_p_row->setStatusTip(QApplication::translate("calibrationDialog", "Number of squares per row", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        groupBox_2->setTitle(QApplication::translate("calibrationDialog", "Matrices Files", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("calibrationDialog", "&Distortion :", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        distortion->setToolTip(QApplication::translate("calibrationDialog", "Xml file to keep distortion parameters", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        distortion->setStatusTip(QApplication::translate("calibrationDialog", "Xml file to keep distortion parameters", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
#ifndef QT_NO_TOOLTIP
        distortionButton->setToolTip(QApplication::translate("calibrationDialog", "open...", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        distortionButton->setText(QApplication::translate("calibrationDialog", "...", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        distortionSvAs->setToolTip(QApplication::translate("calibrationDialog", "Create new xml file to save parameters.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        distortionSvAs->setText(QApplication::translate("calibrationDialog", "Save as", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("calibrationDialog", "&Intrinsic:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        intrinsic->setToolTip(QApplication::translate("calibrationDialog", "Xml file to keep intrinsic parameters", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        intrinsic->setStatusTip(QApplication::translate("calibrationDialog", "Xml file to keep intrinsic parameters", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
#ifndef QT_NO_TOOLTIP
        intrinsicButton->setToolTip(QApplication::translate("calibrationDialog", "open...", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        intrinsicButton->setText(QApplication::translate("calibrationDialog", "...", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        intrinsicSvAs->setToolTip(QApplication::translate("calibrationDialog", "Create new xml file to save parameters.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        intrinsicSvAs->setText(QApplication::translate("calibrationDialog", "Save as", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class calibrationDialog: public Ui_calibrationDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CALIBRATIONDIALOG_H
