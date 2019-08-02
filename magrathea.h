#ifndef MAGRATHEA_H
#define MAGRATHEA_H

#include <string>
#include <exception>
#include <QWidget>
#include <QTextEdit>
#include <QLabel>
#include <QPushButton>
#include <opencv2/opencv.hpp>
#include <StreamViewer.h>
#include <Point.h>
#include <PetalCoordinates.h>

/*
 * Exception thrown when Magrathea finds any trouble
 * TODO: add an error code
 */
class MagratheaException : public std::exception
{
    protected:
        /// stores the exception message
        std::string _why;

    public:
        /// Constructor
        MagratheaException() {}
        MagratheaException(const std::string &msg): _why(msg) {}
        MagratheaException(const MagratheaException &e): _why(e._why) {}
        virtual ~MagratheaException() throw() {}

        // returns the message
        const char* what() const throw() { return _why.c_str(); }
};


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
    bool focusButtonClicked();
    void captureButtonClicked();
    void calibrationCaller(int input);
    void Calibration_ButtonClicked();
    bool FiducialFinderCaller(const int &input, Point &F_point);
    void Fiducial_finder_button_Clicked();
    void Circles_button_Clicked();
    void VignetteButton_clicked();
    void Camera_test();
    void FocusAlgoTest_Func();
    bool loop_fid_finder();
    bool loop_fid_finder(int input);
    bool CVCaptureButtonClicked();
    bool loop_find_circles();
    void J_translator(int index, int button, bool pressed);
    void J_axes_translator(int index, int button, double value);

    //gantry
    void connectGantryBoxClicked(bool checked);
    void enableAxesClicked();

    //motion
    void enableJoystickFreeRun(bool checked);
    void freeRun();
    void enableJoystickStepMotion(bool checked);
    void stepMotion();
    void positionMove();
    void AxisEnableDisableButton();
    void led_label(QLabel *label, bool value);
    void led_label(QLabel *label, bool value, const std::vector<QString> &input);

    /*
     * Find the coordinates of a fiducial located in the given position
     * @param estimated_point an approx position close to the fiducial
     * @param fiducial_type the type of fiducial to search
     */
    Point find_coordinates_at_position(const Point &estimated_point, int fiducial_type, double speed=1.0)
        throw(MagratheaException);

    //Loading
    /*
     * This will open a dialog that will help entering the coordinates
     * of the petal locators
     */
    bool set_petal_coordinates();

    /**
     * Accepts approximate coordinates of the locators and find the
     * exact position. The positions will be stored in
     * PetalCoordinates which will set up the coordinate system from
     * these points.
     */
    int FindPetal(Point &top_locator, Point &bottom_locator);

    int PickAndPlaceModule(const double &PetalAngle,const std::vector<cv::Point3d> &Coordinates );
    bool touchDown(const double &threshold);
    bool Survey(const int &selected_module_index, const double &PetalAngle, std::vector<cv::Point3d> &Module_offsets);
    bool Adjust_module(const cv::Point3d &module_bridge_coordinates, const std::vector<cv::Point3d> &Module_offsets);
    bool GlueLines(const std::vector<cv::Point3d> &line_points);
    bool TalkSR232(const std::vector<std::string> &arguments);

    //test
    void color_test();
    void destroy_all();
    bool loop_test();
    bool loop_test_pressure();
    bool loop_test_images();
    void createTemplate_F();
    bool calibration_plate_measure();
    bool fiducial_chip_measure();
    int TestButtonClick();

signals:
    void Run_focus_signal();
    void Test_signal();

private:
    Ui::Magrathea *ui;

    cv::Mat sub_frame;

    StreamViewer log_viewer;
    /*
     * This will contain the transformation from image to Gantry
     */
    MatrixTransform cameraM;
    double mCamera_angle =  1.268;
    double mCalibration = 3.62483; //SCT optics 10.035;//3.43;//4.5; //[px/um]
    QCamera *mCamera;
    QCameraViewfinder *mCameraViewfinder;
    QCameraImageCapture *mCameraImageCapture;
    QVBoxLayout *mCameraLayout;
    MotionHandler *mMotionHandler;
    QTimer *mPositionTimer;
    PetalCoordinates petal_locations;

    int autoRepeatDelay;
    int autoRepeatInterval;

    bool J_control_Z_1      = true;
    bool J_control_Rotation = true;
    double J_axis_speed     = 1.0;

};

#endif // MAGRATHEA_H
