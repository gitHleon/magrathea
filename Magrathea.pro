#-------------------------------------------------
#
# Project created by QtCreator 2017-10-07T07:29:37
#
#-------------------------------------------------

QT += core gui multimedia multimediawidgets serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Magrathea
TEMPLATE = app
CONFIG += static

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

include(QJoysticks-master\QJoysticks.pri)

SOURCES += \
    MatrixTransform.cc \
    PetalCoordinates.cc \
    Point.cc \
    QPetalLocator.cpp \
    StreamViewer.cc \
    CameraView.cpp \
    alglibtools.cc \
    flatness.cc \
    magrathea.cpp \
    calibrator.cpp \
    focus_finder.cpp \
    MotionHandler.cpp \
    Fiducial_finder.cpp \
    fiducial_locations.cpp \
    verticalalignmenttool.cpp \
    logger.cc \
    main.cpp \



HEADERS += \
        magrathea.h \
        calibrator.h \
        focus_finder.h \
        MotionHandler.h \
        Fiducial_finder.h \
        fiducial_locations.h \
        verticalalignmenttool.h \
        StreamViewer.h \
        CameraView.h \
        QPetalLocator.h

FORMS += \
        magrathea.ui

INCLUDEPATH += F:/Code/include/alglib
INCLUDEPATH += F:/Code/include
INCLUDEPATH += .\mvIMPACT_CPP
INCLUDEPATH += F:\opencv-build\install\include
       LIBS += F:\opencv-build\bin\libopencv_flann401.dll
       LIBS += F:\opencv-build\bin\libopencv_core401.dll
       LIBS += F:\opencv-build\bin\libopencv_highgui401.dll
       LIBS += F:\opencv-build\bin\libopencv_videoio401.dll
       LIBS += F:\opencv-build\bin\libopencv_imgcodecs401.dll
       LIBS += F:\opencv-build\bin\libopencv_imgproc401.dll
       LIBS += F:\opencv-build\bin\libopencv_features2d401.dll
       LIBS += F:\opencv-build\bin\libopencv_xfeatures2d401.dll
       LIBS += F:\opencv-build\bin\libopencv_calib3d401.dll
       LIBS += F:\opencv-build\bin\libopencv_aruco401.dll
       LIBS += F:/Code/lib/libxlnt.a D:/Code/lib/libalglib.a

#Vancouver
win32 : exists(C:/Program Files (x86)/Aerotech/A3200/CLibrary/Include/A3200.h) {
        message("Vancouver, Aerotech A3200 gantry")
        DEFINES += VANCOUVER
        DEFINES += AEROTECH
        SOURCES += AerotechMotionHandler.cpp
        HEADERS += AerotechMotionHandler.h
        INCLUDEPATH += "C:/Program Files (x86)/Aerotech/A3200/CLibrary/Include/"
        INCLUDEPATH += "C:/Program Files (x86)/Aerotech/A3200/CLibrary/Bin/"
        LIBS += -L'C:/Program Files (x86)/Aerotech/A3200/CLibrary/Lib/' -lA3200C
}

#Valencia
win32 : exists(ACSC/C_CPP/ACSC.h) {
        SOURCES += ACSCMotionHandler.cpp
        HEADERS += ACSCMotionHandler.h \
        ACSC\C_CPP\ACSC.h \
        DEFINES += ACSC
        DEFINES += VALENCIA
        LIBS += D:\Code\magrathea\ACSC\C_CPP\ACSCL_x86.LIB
        #LIBS += C:\Users\Silicio\WORK\Opencv_Qt_proj\Loader\ACSC\C_CPP\ACSCL_x86.LIB
}

