/********************************************************************************
** Form generated from reading UI file 'RecorderView.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_RECORDERVIEW_H
#define UI_RECORDERVIEW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_RecorderView
{
public:

    void setupUi(QWidget *RecorderView)
    {
        if (RecorderView->objectName().isEmpty())
            RecorderView->setObjectName(QString::fromUtf8("RecorderView"));
        RecorderView->resize(400, 300);

        retranslateUi(RecorderView);

        QMetaObject::connectSlotsByName(RecorderView);
    } // setupUi

    void retranslateUi(QWidget *RecorderView)
    {
        RecorderView->setWindowTitle(QApplication::translate("RecorderView", "Form", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class RecorderView: public Ui_RecorderView {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_RECORDERVIEW_H
