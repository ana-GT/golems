#-------------------------------------------------
#
# Project created by QtCreator 2015-04-19T12:23:21
#
#-------------------------------------------------

QT       += core gui widgets bluetooth

TARGET = client_pap
TEMPLATE = app

INCLUDEPATH= include ../

SOURCES += src/main.cpp\
        src/client_pap.cpp \
    src/client_unit.cpp \
    src/connectDialog.cpp 
HEADERS  += include/client_pap.h \
    include/client_unit.h \
    include/connectDialog.h

FORMS    += ui/client_pap.ui \
    ui/connectDialog.ui

CONFIG += mobility
MOBILITY =

