/********************************************************************************
** Form generated from reading UI file 'PlayerView.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PLAYERVIEW_H
#define UI_PLAYERVIEW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QFrame>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QListView>
#include <QtGui/QListWidget>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QSplitter>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "view/PlayerView/MultiLaneView/MultiLaneView.h"

QT_BEGIN_NAMESPACE

class Ui_PlayerView
{
public:
    QVBoxLayout *verticalLayout;
    QFrame *playControlsFrame;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *playButton;
    QPushButton *pauseButton;
    QPushButton *stopButton;
    QPushButton *skipButton;
    QSpacerItem *horizontalSpacer;
    QLabel *label;
    QPushButton *pushButton_2;
    QComboBox *comboBox;
    QSplitter *splitter_2;
    MultiLaneView *multiLaneView;
    QSplitter *splitter;
    QFrame *behaviorListFrame;
    QVBoxLayout *verticalLayout_3;
    QListView *behaviorListView;
    QFrame *behaviorDetailFrame;
    QVBoxLayout *verticalLayout_2;
    QLabel *behaviorNameLabel;
    QLabel *behaviorNameValueLabel;
    QLabel *idLabel;
    QLabel *idValueLabel;
    QLabel *motorCountLabel;
    QLabel *motorCountValueLabel;
    QLabel *descriptionLabel;
    QListWidget *motorListView;
    QPushButton *addToQueueButton;

    void setupUi(QWidget *PlayerView)
    {
        if (PlayerView->objectName().isEmpty())
            PlayerView->setObjectName(QString::fromUtf8("PlayerView"));
        PlayerView->resize(827, 724);
        PlayerView->setStyleSheet(QString::fromUtf8(""));
        verticalLayout = new QVBoxLayout(PlayerView);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        playControlsFrame = new QFrame(PlayerView);
        playControlsFrame->setObjectName(QString::fromUtf8("playControlsFrame"));
        playControlsFrame->setFrameShape(QFrame::StyledPanel);
        playControlsFrame->setFrameShadow(QFrame::Raised);
        horizontalLayout_2 = new QHBoxLayout(playControlsFrame);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        playButton = new QPushButton(playControlsFrame);
        playButton->setObjectName(QString::fromUtf8("playButton"));
        playButton->setMinimumSize(QSize(85, 0));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/play-img.png"), QSize(), QIcon::Normal, QIcon::Off);
        playButton->setIcon(icon);
        playButton->setFlat(false);

        horizontalLayout_2->addWidget(playButton);

        pauseButton = new QPushButton(playControlsFrame);
        pauseButton->setObjectName(QString::fromUtf8("pauseButton"));
        pauseButton->setMinimumSize(QSize(85, 0));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/pause-img.png"), QSize(), QIcon::Normal, QIcon::Off);
        pauseButton->setIcon(icon1);

        horizontalLayout_2->addWidget(pauseButton);

        stopButton = new QPushButton(playControlsFrame);
        stopButton->setObjectName(QString::fromUtf8("stopButton"));
        stopButton->setMinimumSize(QSize(85, 0));
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/stop-img.png"), QSize(), QIcon::Normal, QIcon::Off);
        stopButton->setIcon(icon2);

        horizontalLayout_2->addWidget(stopButton);

        skipButton = new QPushButton(playControlsFrame);
        skipButton->setObjectName(QString::fromUtf8("skipButton"));
        skipButton->setMinimumSize(QSize(85, 0));
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/skip-img.png"), QSize(), QIcon::Normal, QIcon::Off);
        skipButton->setIcon(icon3);

        horizontalLayout_2->addWidget(skipButton);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);

        label = new QLabel(playControlsFrame);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout_2->addWidget(label);

        pushButton_2 = new QPushButton(playControlsFrame);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/add-img.png"), QSize(), QIcon::Normal, QIcon::Off);
        pushButton_2->setIcon(icon4);

        horizontalLayout_2->addWidget(pushButton_2);

        comboBox = new QComboBox(playControlsFrame);
        comboBox->setObjectName(QString::fromUtf8("comboBox"));

        horizontalLayout_2->addWidget(comboBox);


        verticalLayout->addWidget(playControlsFrame);

        splitter_2 = new QSplitter(PlayerView);
        splitter_2->setObjectName(QString::fromUtf8("splitter_2"));
        splitter_2->setOrientation(Qt::Vertical);
        multiLaneView = new MultiLaneView(splitter_2);
        multiLaneView->setObjectName(QString::fromUtf8("multiLaneView"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(multiLaneView->sizePolicy().hasHeightForWidth());
        multiLaneView->setSizePolicy(sizePolicy);
        multiLaneView->setMinimumSize(QSize(100, 100));
        splitter_2->addWidget(multiLaneView);
        splitter = new QSplitter(splitter_2);
        splitter->setObjectName(QString::fromUtf8("splitter"));
        splitter->setOrientation(Qt::Horizontal);
        behaviorListFrame = new QFrame(splitter);
        behaviorListFrame->setObjectName(QString::fromUtf8("behaviorListFrame"));
        sizePolicy.setHeightForWidth(behaviorListFrame->sizePolicy().hasHeightForWidth());
        behaviorListFrame->setSizePolicy(sizePolicy);
        behaviorListFrame->setMinimumSize(QSize(100, 100));
        behaviorListFrame->setFrameShape(QFrame::StyledPanel);
        behaviorListFrame->setFrameShadow(QFrame::Raised);
        verticalLayout_3 = new QVBoxLayout(behaviorListFrame);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        behaviorListView = new QListView(behaviorListFrame);
        behaviorListView->setObjectName(QString::fromUtf8("behaviorListView"));
        behaviorListView->setContextMenuPolicy(Qt::CustomContextMenu);

        verticalLayout_3->addWidget(behaviorListView);

        splitter->addWidget(behaviorListFrame);
        behaviorDetailFrame = new QFrame(splitter);
        behaviorDetailFrame->setObjectName(QString::fromUtf8("behaviorDetailFrame"));
        sizePolicy.setHeightForWidth(behaviorDetailFrame->sizePolicy().hasHeightForWidth());
        behaviorDetailFrame->setSizePolicy(sizePolicy);
        behaviorDetailFrame->setMinimumSize(QSize(100, 100));
        behaviorDetailFrame->setFrameShape(QFrame::StyledPanel);
        behaviorDetailFrame->setFrameShadow(QFrame::Raised);
        verticalLayout_2 = new QVBoxLayout(behaviorDetailFrame);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        behaviorNameLabel = new QLabel(behaviorDetailFrame);
        behaviorNameLabel->setObjectName(QString::fromUtf8("behaviorNameLabel"));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        behaviorNameLabel->setFont(font);

        verticalLayout_2->addWidget(behaviorNameLabel);

        behaviorNameValueLabel = new QLabel(behaviorDetailFrame);
        behaviorNameValueLabel->setObjectName(QString::fromUtf8("behaviorNameValueLabel"));

        verticalLayout_2->addWidget(behaviorNameValueLabel);

        idLabel = new QLabel(behaviorDetailFrame);
        idLabel->setObjectName(QString::fromUtf8("idLabel"));
        idLabel->setFont(font);

        verticalLayout_2->addWidget(idLabel);

        idValueLabel = new QLabel(behaviorDetailFrame);
        idValueLabel->setObjectName(QString::fromUtf8("idValueLabel"));

        verticalLayout_2->addWidget(idValueLabel);

        motorCountLabel = new QLabel(behaviorDetailFrame);
        motorCountLabel->setObjectName(QString::fromUtf8("motorCountLabel"));
        motorCountLabel->setFont(font);

        verticalLayout_2->addWidget(motorCountLabel);

        motorCountValueLabel = new QLabel(behaviorDetailFrame);
        motorCountValueLabel->setObjectName(QString::fromUtf8("motorCountValueLabel"));

        verticalLayout_2->addWidget(motorCountValueLabel);

        descriptionLabel = new QLabel(behaviorDetailFrame);
        descriptionLabel->setObjectName(QString::fromUtf8("descriptionLabel"));
        descriptionLabel->setFont(font);

        verticalLayout_2->addWidget(descriptionLabel);

        motorListView = new QListWidget(behaviorDetailFrame);
        motorListView->setObjectName(QString::fromUtf8("motorListView"));

        verticalLayout_2->addWidget(motorListView);

        addToQueueButton = new QPushButton(behaviorDetailFrame);
        addToQueueButton->setObjectName(QString::fromUtf8("addToQueueButton"));
        addToQueueButton->setEnabled(false);
        addToQueueButton->setIcon(icon4);

        verticalLayout_2->addWidget(addToQueueButton);

        splitter->addWidget(behaviorDetailFrame);
        splitter_2->addWidget(splitter);

        verticalLayout->addWidget(splitter_2);


        retranslateUi(PlayerView);

        QMetaObject::connectSlotsByName(PlayerView);
    } // setupUi

    void retranslateUi(QWidget *PlayerView)
    {
        PlayerView->setWindowTitle(QApplication::translate("PlayerView", "Form", 0, QApplication::UnicodeUTF8));
        playButton->setText(QString());
        pauseButton->setText(QString());
        stopButton->setText(QString());
        skipButton->setText(QString());
        label->setText(QApplication::translate("PlayerView", "add lane", 0, QApplication::UnicodeUTF8));
        pushButton_2->setText(QString());
        behaviorNameLabel->setText(QApplication::translate("PlayerView", "behavior name", 0, QApplication::UnicodeUTF8));
        behaviorNameValueLabel->setText(QApplication::translate("PlayerView", "-", 0, QApplication::UnicodeUTF8));
        idLabel->setText(QApplication::translate("PlayerView", "id", 0, QApplication::UnicodeUTF8));
        idValueLabel->setText(QApplication::translate("PlayerView", "-", 0, QApplication::UnicodeUTF8));
        motorCountLabel->setText(QApplication::translate("PlayerView", "motor count", 0, QApplication::UnicodeUTF8));
        motorCountValueLabel->setText(QApplication::translate("PlayerView", "-", 0, QApplication::UnicodeUTF8));
        descriptionLabel->setText(QApplication::translate("PlayerView", "description", 0, QApplication::UnicodeUTF8));
        addToQueueButton->setText(QApplication::translate("PlayerView", "add to queue", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class PlayerView: public Ui_PlayerView {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PLAYERVIEW_H
