#-------------------------------------------------
#
# Project created by QtCreator 2019-03-23T11:20:47
#
#-------------------------------------------------

QT       += core gui
QT       += serialport

INCLUDEPATH += ./DK
INCLUDEPATH += /home/young-forever


greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = serial_test
TEMPLATE = app

#include(./DK/DK.pri)

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        mainwindow.cpp \
    mycommand.cpp \
    DK/alglibinternal.cpp \
    DK/alglibmisc.cpp \
    DK/ap.cpp \
    DK/cablelencal.cpp \
    DK/DKofCDPR.cpp \
    DK/iMath.cpp \
    DK/linalg.cpp \
    DK/mythread.cpp \
    DK/optimization.cpp \
    DK/solvers.cpp \
    trj_plan.cpp \
    DK/invsknmtc_cdpr.cpp \
    controlalgorithm.cpp \
    remote_dialog.cpp \
    position_mode_set_dialog.cpp \
    complaint_mode_set_dialog.cpp \
    threadfromqthread.cpp \
    forcedistribute.cpp \
    statecheckset_dialog.cpp \
    mathtool.cpp

HEADERS += \
        mainwindow.h \
    mycommand.h \
    DK/alglibinternal.h \
    DK/alglibmisc.h \
    DK/ap.h \
    DK/cablelencal.h \
    DK/DKofCDPR.h \
    DK/iMath.h \
    DK/linalg.h \
    DK/optimization.h \
    DK/solvers.h \
    DK/stdafx.h \
    trj_plan.h \
    DK/invsknmtc_cdpr.h \
    controlalgorithm.h \
    remote_dialog.h \
    position_mode_set_dialog.h \
    complaint_mode_set_dialog.h \
    threadfromqthread.h \
    forcedistribute.h \
    statecheckset_dialog.h \
    mathtool.h

FORMS += \
        mainwindow.ui \
    remote_dialog.ui \
    position_mode_set_dialog.ui \
    complaint_mode_set_dialog.ui \
    statecheckset_dialog.ui
