/********************************************************************************
** Form generated from reading UI file 'HujoonWidget.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_HUJOONWIDGET_H
#define UI_HUJOONWIDGET_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>
#include <QtGui/QScrollBar>
#include <QtGui/QTableWidget>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_HujoonGUIWidget
{
public:
    QScrollBar *hzScrl_LED;
    QTableWidget *table_motorstate;
    QPushButton *Btn_Action;

    void setupUi(QWidget *HujoonGUIWidget)
    {
        if (HujoonGUIWidget->objectName().isEmpty())
            HujoonGUIWidget->setObjectName(QString::fromUtf8("HujoonGUIWidget"));
        HujoonGUIWidget->resize(542, 229);
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(HujoonGUIWidget->sizePolicy().hasHeightForWidth());
        HujoonGUIWidget->setSizePolicy(sizePolicy);
        HujoonGUIWidget->setMinimumSize(QSize(0, 0));
        HujoonGUIWidget->setMaximumSize(QSize(16777215, 16777215));
        hzScrl_LED = new QScrollBar(HujoonGUIWidget);
        hzScrl_LED->setObjectName(QString::fromUtf8("hzScrl_LED"));
        hzScrl_LED->setGeometry(QRect(10, 140, 440, 16));
        hzScrl_LED->setOrientation(Qt::Horizontal);
        table_motorstate = new QTableWidget(HujoonGUIWidget);
        if (table_motorstate->columnCount() < 5)
            table_motorstate->setColumnCount(5);
        if (table_motorstate->rowCount() < 4)
            table_motorstate->setRowCount(4);
        table_motorstate->setObjectName(QString::fromUtf8("table_motorstate"));
        table_motorstate->setGeometry(QRect(10, 30, 440, 100));
        sizePolicy.setHeightForWidth(table_motorstate->sizePolicy().hasHeightForWidth());
        table_motorstate->setSizePolicy(sizePolicy);
        table_motorstate->setFrameShape(QFrame::NoFrame);
        table_motorstate->setFrameShadow(QFrame::Plain);
        table_motorstate->setAutoScroll(true);
        table_motorstate->setRowCount(4);
        table_motorstate->setColumnCount(5);
        table_motorstate->horizontalHeader()->setVisible(false);
        table_motorstate->horizontalHeader()->setDefaultSectionSize(100);
        table_motorstate->horizontalHeader()->setHighlightSections(false);
        table_motorstate->horizontalHeader()->setMinimumSectionSize(20);
        table_motorstate->verticalHeader()->setVisible(false);
        table_motorstate->verticalHeader()->setDefaultSectionSize(25);
        table_motorstate->verticalHeader()->setHighlightSections(false);
        table_motorstate->verticalHeader()->setMinimumSectionSize(20);
        Btn_Action = new QPushButton(HujoonGUIWidget);
        Btn_Action->setObjectName(QString::fromUtf8("Btn_Action"));
        Btn_Action->setGeometry(QRect(10, 170, 85, 27));

        retranslateUi(HujoonGUIWidget);

        QMetaObject::connectSlotsByName(HujoonGUIWidget);
    } // setupUi

    void retranslateUi(QWidget *HujoonGUIWidget)
    {
        HujoonGUIWidget->setWindowTitle(QApplication::translate("HujoonGUIWidget", "Hujoon", 0, QApplication::UnicodeUTF8));
        Btn_Action->setText(QApplication::translate("HujoonGUIWidget", "Reset Pos", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class HujoonGUIWidget: public Ui_HujoonGUIWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_HUJOONWIDGET_H
