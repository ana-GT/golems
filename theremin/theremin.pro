TEMPLATE = subdirs
CONFIG += ordered
SUBDIRS = tinyalsa \
          app
app.depends = tinyalsa

INCLUDEPATH += ../tinyalsa/include
