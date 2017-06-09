QT += core
QT -= gui

TARGET = g2o_ORB
CONFIG += console c++11
CONFIG -= app_bundle

TEMPLATE = app

SOURCES +=


SOURCES += \
    src/Converter.cpp \
    src/Optimizer.cpp \
    g2o_main.cpp \
    PNP_main.cpp

INCLUDEPATH += /usr/local/include/opencv \
/usr/local/include/opencv2 \
/usr/local/include \
/usr/include/eigen3/ \

LIBS += -L/usr/local/lib      -lopencv_ccalib -lopencv_cvv    -lopencv_calib3d -lopencv_features2d -lopencv_flann  -lopencv_objdetect -lopencv_ml -lopencv_highgui -lopencv_videoio -lopencv_photo -lopencv_imgcodecs   -lopencv_imgproc  -lopencv_core   \

#lib/libgo.so
#/home/eric/Documents/workspace/CMAKE_pros/ORB-SLAM2/Thirdparty/g2o/lib/libg2o.so

HEADERS += \
    include/Converter.h \
    include/Optimizer.h

unix:!macx: LIBS += -L$$PWD/lib/ -lg2o

INCLUDEPATH += $$PWD/include
DEPENDPATH += $$PWD/include
