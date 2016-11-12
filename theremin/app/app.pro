
QT  += core gui widgets

TEMPLATE = app
TARGET = theremin

INCLUDEPATH += include
INCLUDEPATH += ../tinyalsa/include

SOURCES += src/main.cpp\
        src/theremin.cpp \
 
HEADERS  += include/theremin.h
LIBS += ../tinyalsa/libtinyalsa.so

FORMS    += ui/theremin.ui

CONFIG += mobility
MOBILITY =



