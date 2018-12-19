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
    bool FiducialFinderCaller(const int &input,std::vector <double> & F_point);
    void Fiducial_finder_button_Clicked();
    void VignetteButton_clicked();
    void Camera_test();
    void Aruco_test();
    void FocusAlgoTest_Func();

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

    //test
    void color_test();
    void destroy_all();
    void loop_test();
    void createTemplate_F();
    void capture_fid_and_move();
    void calibration_plate_measure();

private:
    Ui::Magrathea *ui;

    //cv::VideoCapture cap;
    cv::Mat sub_frame;

    double mCalibration = 3.43;//4.5; //[px/um]
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
