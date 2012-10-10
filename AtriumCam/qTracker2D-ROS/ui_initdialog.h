/********************************************************************************
** Form generated from reading UI file 'initdialog.ui'
**
** Created: Sun Nov 20 16:56:41 2011
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_INITDIALOG_H
#define UI_INITDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QSpacerItem>
#include <QtGui/QToolButton>

QT_BEGIN_NAMESPACE

class Ui_initdialog
{
public:
    QGridLayout *gridLayout_3;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_3;
    QComboBox *typeCombo;
    QGroupBox *cameraGroup;
    QGridLayout *gridLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLineEdit *lineEdit;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QComboBox *comboBox;
    QGroupBox *videoGroup;
    QGridLayout *gridLayout_2;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_5;
    QSpacerItem *horizontalSpacer_15;
    QLineEdit *videoEdit;
    QToolButton *videoButton;
    QSpacerItem *verticalSpacer;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *initdialog)
    {
        if (initdialog->objectName().isEmpty())
            initdialog->setObjectName(QString::fromUtf8("initdialog"));
        initdialog->setWindowModality(Qt::WindowModal);
        initdialog->resize(344, 284);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(initdialog->sizePolicy().hasHeightForWidth());
        initdialog->setSizePolicy(sizePolicy);
        initdialog->setSizeGripEnabled(false);
        initdialog->setModal(false);
        gridLayout_3 = new QGridLayout(initdialog);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_3 = new QLabel(initdialog);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_3->addWidget(label_3);

        typeCombo = new QComboBox(initdialog);
        typeCombo->setObjectName(QString::fromUtf8("typeCombo"));

        horizontalLayout_3->addWidget(typeCombo);


        gridLayout_3->addLayout(horizontalLayout_3, 0, 0, 1, 1);

        cameraGroup = new QGroupBox(initdialog);
        cameraGroup->setObjectName(QString::fromUtf8("cameraGroup"));
        gridLayout = new QGridLayout(cameraGroup);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(cameraGroup);
        label->setObjectName(QString::fromUtf8("label"));
        label->setTextFormat(Qt::AutoText);
        label->setWordWrap(false);

        horizontalLayout->addWidget(label);

        lineEdit = new QLineEdit(cameraGroup);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(lineEdit->sizePolicy().hasHeightForWidth());
        lineEdit->setSizePolicy(sizePolicy1);

        horizontalLayout->addWidget(lineEdit);


        gridLayout->addLayout(horizontalLayout, 0, 0, 1, 1);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_2 = new QLabel(cameraGroup);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_2->addWidget(label_2);

        comboBox = new QComboBox(cameraGroup);
        comboBox->setObjectName(QString::fromUtf8("comboBox"));

        horizontalLayout_2->addWidget(comboBox);


        gridLayout->addLayout(horizontalLayout_2, 1, 0, 1, 1);


        gridLayout_3->addWidget(cameraGroup, 1, 0, 1, 1);

        videoGroup = new QGroupBox(initdialog);
        videoGroup->setObjectName(QString::fromUtf8("videoGroup"));
        gridLayout_2 = new QGridLayout(videoGroup);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        label_5 = new QLabel(videoGroup);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout_6->addWidget(label_5);

        horizontalSpacer_15 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_6->addItem(horizontalSpacer_15);

        videoEdit = new QLineEdit(videoGroup);
        videoEdit->setObjectName(QString::fromUtf8("videoEdit"));

        horizontalLayout_6->addWidget(videoEdit);

        videoButton = new QToolButton(videoGroup);
        videoButton->setObjectName(QString::fromUtf8("videoButton"));

        horizontalLayout_6->addWidget(videoButton);


        gridLayout_2->addLayout(horizontalLayout_6, 0, 0, 1, 1);


        gridLayout_3->addWidget(videoGroup, 2, 0, 1, 1);

        verticalSpacer = new QSpacerItem(108, 13, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_3->addItem(verticalSpacer, 3, 0, 1, 1);

        buttonBox = new QDialogButtonBox(initdialog);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setLayoutDirection(Qt::RightToLeft);
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Close|QDialogButtonBox::Ok);
        buttonBox->setCenterButtons(true);

        gridLayout_3->addWidget(buttonBox, 4, 0, 1, 1);

#ifndef QT_NO_SHORTCUT
        label_3->setBuddy(typeCombo);
        label->setBuddy(lineEdit);
        label_2->setBuddy(comboBox);
        label_5->setBuddy(videoEdit);
#endif // QT_NO_SHORTCUT
        QWidget::setTabOrder(lineEdit, comboBox);
        QWidget::setTabOrder(comboBox, buttonBox);

        retranslateUi(initdialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), initdialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), initdialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(initdialog);
    } // setupUi

    void retranslateUi(QDialog *initdialog)
    {
        initdialog->setWindowTitle(QApplication::translate("initdialog", "Camera Settings", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        initdialog->setStatusTip(QApplication::translate("initdialog", "Set Camera Settings", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
#ifndef QT_NO_WHATSTHIS
        initdialog->setWhatsThis(QApplication::translate("initdialog", "Set Camera Settings", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_WHATSTHIS
        label_3->setText(QApplication::translate("initdialog", "Select &Input Type:", 0, QApplication::UnicodeUTF8));
        typeCombo->clear();
        typeCombo->insertItems(0, QStringList()
         << QApplication::translate("initdialog", "Camera", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("initdialog", "Video File", 0, QApplication::UnicodeUTF8)
        );
        cameraGroup->setTitle(QApplication::translate("initdialog", "Select Camera Settings", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("initdialog", "C&amera number:", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("initdialog", "&Resolution:", 0, QApplication::UnicodeUTF8));
        comboBox->clear();
        comboBox->insertItems(0, QStringList()
         << QApplication::translate("initdialog", "320x240", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("initdialog", "640x480", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("initdialog", "960x720", 0, QApplication::UnicodeUTF8)
        );
        videoGroup->setTitle(QApplication::translate("initdialog", "Select Video file", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("initdialog", "&Video File:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        videoEdit->setToolTip(QApplication::translate("initdialog", "The folder to save the data file.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        videoEdit->setStatusTip(QApplication::translate("initdialog", "The folder to save the data file.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
#ifndef QT_NO_TOOLTIP
        videoButton->setToolTip(QApplication::translate("initdialog", "open...", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        videoButton->setText(QApplication::translate("initdialog", "...", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class initdialog: public Ui_initdialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_INITDIALOG_H
