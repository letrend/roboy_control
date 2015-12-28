/********************************************************************************
** Form generated from reading UI file 'EditorView.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_EDITORVIEW_H
#define UI_EDITORVIEW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_EditorView
{
public:

    void setupUi(QWidget *EditorView)
    {
        if (EditorView->objectName().isEmpty())
            EditorView->setObjectName(QString::fromUtf8("EditorView"));
        EditorView->resize(400, 300);

        retranslateUi(EditorView);

        QMetaObject::connectSlotsByName(EditorView);
    } // setupUi

    void retranslateUi(QWidget *EditorView)
    {
        EditorView->setWindowTitle(QApplication::translate("EditorView", "Form", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class EditorView: public Ui_EditorView {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_EDITORVIEW_H
