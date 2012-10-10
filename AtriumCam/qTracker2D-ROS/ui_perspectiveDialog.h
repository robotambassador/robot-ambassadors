/********************************************************************************
** Form generated from reading UI file 'perspectiveDialog.ui'
**
** Created: Sun Nov 20 16:56:41 2011
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PERSPECTIVEDIALOG_H
#define UI_PERSPECTIVEDIALOG_H

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

class Ui_perspectiveDialog
{
public:
    QGridLayout *gridLayout;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QSpacerItem *horizontalSpacer;
    QLineEdit *point1;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QSpacerItem *horizontalSpacer_2;
    QLineEdit *point2;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_3;
    QSpacerItem *horizontalSpacer_3;
    QLineEdit *point3;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_4;
    QSpacerItem *horizontalSpacer_4;
    QLineEdit *point4;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_5;
    QLineEdit *perspective;
    QToolButton *perspectiveButton;
    QPushButton *perspectiveSvAs;
    QSpacerItem *verticalSpacer_2;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *perspectiveDialog)
    {
        if (perspectiveDialog->objectName().isEmpty())
            perspectiveDialog->setObjectName(QString::fromUtf8("perspectiveDialog"));
        perspectiveDialog->resize(400, 320);
        gridLayout = new QGridLayout(perspectiveDialog);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        groupBox = new QGroupBox(perspectiveDialog);
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

        point1 = new QLineEdit(groupBox);
        point1->setObjectName(QString::fromUtf8("point1"));

        horizontalLayout->addWidget(point1);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_2->addWidget(label_2);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);

        point2 = new QLineEdit(groupBox);
        point2->setObjectName(QString::fromUtf8("point2"));

        horizontalLayout_2->addWidget(point2);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_3->addWidget(label_3);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_3);

        point3 = new QLineEdit(groupBox);
        point3->setObjectName(QString::fromUtf8("point3"));

        horizontalLayout_3->addWidget(point3);


        verticalLayout->addLayout(horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_4->addWidget(label_4);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_4);

        point4 = new QLineEdit(groupBox);
        point4->setObjectName(QString::fromUtf8("point4"));

        horizontalLayout_4->addWidget(point4);


        verticalLayout->addLayout(horizontalLayout_4);


        gridLayout->addWidget(groupBox, 0, 0, 1, 1);

        groupBox_2 = new QGroupBox(perspectiveDialog);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        verticalLayout_2 = new QVBoxLayout(groupBox_2);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout_6->addWidget(label_5);

        perspective = new QLineEdit(groupBox_2);
        perspective->setObjectName(QString::fromUtf8("perspective"));

        horizontalLayout_6->addWidget(perspective);

        perspectiveButton = new QToolButton(groupBox_2);
        perspectiveButton->setObjectName(QString::fromUtf8("perspectiveButton"));

        horizontalLayout_6->addWidget(perspectiveButton);

        perspectiveSvAs = new QPushButton(groupBox_2);
        perspectiveSvAs->setObjectName(QString::fromUtf8("perspectiveSvAs"));

        horizontalLayout_6->addWidget(perspectiveSvAs);


        verticalLayout_2->addLayout(horizontalLayout_6);


        gridLayout->addWidget(groupBox_2, 1, 0, 1, 1);

        verticalSpacer_2 = new QSpacerItem(380, 18, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer_2, 2, 0, 1, 1);

        buttonBox = new QDialogButtonBox(perspectiveDialog);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setLayoutDirection(Qt::RightToLeft);
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Save);
        buttonBox->setCenterButtons(true);

        gridLayout->addWidget(buttonBox, 3, 0, 1, 1);

#ifndef QT_NO_SHORTCUT
        label->setBuddy(point1);
        label_2->setBuddy(point2);
        label_3->setBuddy(point3);
        label_4->setBuddy(point4);
        label_5->setBuddy(perspective);
#endif // QT_NO_SHORTCUT

        retranslateUi(perspectiveDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), perspectiveDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), perspectiveDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(perspectiveDialog);
    } // setupUi

    void retranslateUi(QDialog *perspectiveDialog)
    {
        perspectiveDialog->setWindowTitle(QApplication::translate("perspectiveDialog", "Perspective Settings", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("perspectiveDialog", "Points", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("perspectiveDialog", "Point 1 :", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        point1->setToolTip(QApplication::translate("perspectiveDialog", "1st Point Coordinates [X, Y].", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        point1->setStatusTip(QApplication::translate("perspectiveDialog", "1st Point Coordinates [X, Y].", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        label_2->setText(QApplication::translate("perspectiveDialog", "Point 2 :", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        point2->setToolTip(QApplication::translate("perspectiveDialog", "2nd Point Coordinates [X, Y].", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        point2->setStatusTip(QApplication::translate("perspectiveDialog", "2nd Point Coordinates [X, Y].", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        label_3->setText(QApplication::translate("perspectiveDialog", "Point 3 :", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        point3->setToolTip(QApplication::translate("perspectiveDialog", "3rd Point Coordinates [X, Y].", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        point3->setStatusTip(QApplication::translate("perspectiveDialog", "3rd Point Coordinates [X, Y].", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        label_4->setText(QApplication::translate("perspectiveDialog", "Point 4 :", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        point4->setToolTip(QApplication::translate("perspectiveDialog", "Number of squares per row", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        point4->setStatusTip(QApplication::translate("perspectiveDialog", "Number of squares per row", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        groupBox_2->setTitle(QApplication::translate("perspectiveDialog", "Matrix File", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("perspectiveDialog", "Perspective:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        perspective->setToolTip(QApplication::translate("perspectiveDialog", "Xml file to keep perspective parameters", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        perspective->setStatusTip(QApplication::translate("perspectiveDialog", "Xml file to keep perspective parameters", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
#ifndef QT_NO_TOOLTIP
        perspectiveButton->setToolTip(QApplication::translate("perspectiveDialog", "open...", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        perspectiveButton->setText(QApplication::translate("perspectiveDialog", "...", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        perspectiveSvAs->setToolTip(QApplication::translate("perspectiveDialog", "Create new xml file to save parameters.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        perspectiveSvAs->setText(QApplication::translate("perspectiveDialog", "Save as", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class perspectiveDialog: public Ui_perspectiveDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PERSPECTIVEDIALOG_H
