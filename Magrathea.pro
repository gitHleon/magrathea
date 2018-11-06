#-------------------------------------------------
#
# Project created by QtCreator 2017-10-07T07:29:37
#
#-------------------------------------------------

QT += core gui multimedia multimediawidgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Magrathea
TEMPLATE = app

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
        magrathea.cpp \
        calibrator.cpp \
        focus_finder.cpp \
alglib-3.12.0\cpp\src\alglibinternal.cpp \
alglib-3.12.0\cpp\src\ap.cpp \
alglib-3.12.0\cpp\src\alglibmisc.cpp \
alglib-3.12.0\cpp\src\dataanalysis.cpp \
alglib-3.12.0\cpp\src\diffequations.cpp \
alglib-3.12.0\cpp\src\fasttransforms.cpp \
alglib-3.12.0\cpp\src\integration.cpp \
alglib-3.12.0\cpp\src\interpolation.cpp \
alglib-3.12.0\cpp\src\linalg.cpp \
alglib-3.12.0\cpp\src\optimization.cpp \
alglib-3.12.0\cpp\src\solvers.cpp \
alglib-3.12.0\cpp\src\specialfunctions.cpp \
alglib-3.12.0\cpp\src\statistics.cpp \
        MotionHandler.cpp \
Fiducial_finder.cpp \
    fiducial_locations.cpp \
    verticalalignmenttool.cpp

HEADERS += \
        magrathea.h \
        calibrator.h \
        focus_finder.h \
alglib-3.12.0\cpp\src\alglibmisc.h \
alglib-3.12.0\cpp\src\alglibinternal.h \
alglib-3.12.0\cpp\src\ap.h \
alglib-3.12.0\cpp\src\dataanalysis.h \
alglib-3.12.0\cpp\src\diffequations.h \
alglib-3.12.0\cpp\src\fasttransforms.h \
alglib-3.12.0\cpp\src\integration.h \
alglib-3.12.0\cpp\src\interpolation.h \
alglib-3.12.0\cpp\src\linalg.h \
alglib-3.12.0\cpp\src\optimization.h \
alglib-3.12.0\cpp\src\solvers.h \
alglib-3.12.0\cpp\src\specialfunctions.h \
alglib-3.12.0\cpp\src\statistics.h \
alglib-3.12.0\cpp\src\stdafx.h \
        MotionHandler.h \
Fiducial_finder.h \
    fiducial_locations.h \
    verticalalignmenttool.h

FORMS += \
        magrathea.ui

INCLUDEPATH += alglib-3.12.0\cpp\src
INCLUDEPATH += D:\opencv-build\install\include
       LIBS += D:\opencv-build\bin\libopencv_flann341.dll
       LIBS += D:\opencv-build\bin\libopencv_core341.dll
       LIBS += D:\opencv-build\bin\libopencv_highgui341.dll
       LIBS += D:\opencv-build\bin\libopencv_videoio341.dll
       LIBS += D:\opencv-build\bin\libopencv_imgcodecs341.dll
       LIBS += D:\opencv-build\bin\libopencv_imgproc341.dll
       LIBS += D:\opencv-build\bin\libopencv_features2d341.dll
       LIBS += D:\opencv-build\bin\libopencv_xfeatures2d341.dll
       LIBS += D:\opencv-build\bin\libopencv_calib3d341.dll
       LIBS += D:\opencv-build\bin\libopencv_aruco341.dll

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
