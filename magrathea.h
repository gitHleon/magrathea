#ifndef MAGRATHEA_H
#define MAGRATHEA_H

#include <QWidget>
#include <QTextEdit>
#include <QLabel>
#include <QPushButton>
//#include <QShortcut>
#include <opencv2/opencv.hpp>
#include "fiducial_locations.h"

namespace Ui {
class Magrathea;
}

class QCamera;
class QCameraViewfinder;
class QCameraImageCapture;
class QVBoxLayout;
class MotionHandler;
class QTimer;

#ifdef AEROTECH
class AerotechMotionHandler;
#elif VALENCIA
class ACSCMotionHandler;
#endif

class Magrathea : public QWidget
{
    Q_OBJECT

public:
    explicit Magrathea(QWidget *parent = 0);
    ~Magrathea();

    static QTextEdit *outputLogTextEdit;

private slots:

    //timer
    void updatePosition();

    //camera
    void enableCameraBoxClicked(bool checked);
    void focusButtonClicked();
    void captureButtonClicked();
    void calibrationCaller(int input);
    void Calibration_ButtonClicked();
    void Calibration_2_ButtonClicked();
    void FiducialFinderCaller(const int &input);
    void Fiducial_finder_button_1_Clicked();
    void Fiducial_finder_button_2_Clicked();
    void Camera_test();

    //gantry
    void connectGantryBoxClicked(bool checked);
    void enableAxesClicked();

    //motion
    void enableJoystickFreeRun(bool checked);
    void freeRun();
    void enableJoystickStepMotion(bool checked);
    void stepMotion();
    void positionMove();
    void axisStepRepeatBoxClicked(bool checked);
    void AxisEnableDisableButton();
    void led_label(QLabel *label, bool value);

private:
    Ui::Magrathea *ui;

    //cv::VideoCapture cap;
    cv::Mat sub_frame;

    double mCalibration = -100000.1;
    QCamera *mCamera;
    QCameraViewfinder *mCameraViewfinder;
    QCameraImageCapture *mCameraImageCapture;
    QVBoxLayout *mCameraLayout;
    MotionHandler *mMotionHandler;
    QTimer *mPositionTimer;
    fiducial_locations *f_locations;

    int autoRepeatDelay;
    int autoRepeatInterval;

    //keyboard shortcut for joystick
//    QShortcut *shortcut_PX;
//    QShortcut *shortcut_NX;
//    QShortcut *shortcut_PY;
//    QShortcut *shortcut_NY;
//    QShortcut *shortcut_PZ;
//    QShortcut *shortcut_NZ;
//    QShortcut *shortcut_PZ_2;
//    QShortcut *shortcut_NZ_2;
//    QShortcut *shortcut_PU;
//    QShortcut *shortcut_NU;
//    QShortcut *shortcut_STOP;

};

#endif // MAGRATHEA_H
