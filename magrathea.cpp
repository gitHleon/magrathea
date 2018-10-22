#include "magrathea.h"
#include "ui_magrathea.h"
#include <QTimer>
#include <QFont>
#include <QCamera>
#include <QCameraViewfinder>
#include <QCameraImageCapture>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QTextStream>
#include <QtMessageHandler>
#include <MotionHandler.h>
#include <cmath>
#include <QMessageBox>
#include "calibrator.h"
#include "focus_finder.h"
#include "Fiducial_finder.h"
#include "verticalalignmenttool.h"
#include <conio.h>
#include <fstream>
#ifdef VANCOUVER
#include <AerotechMotionhandler.h>
#elif VALENCIA
#include <ACSCMotionHandler.h>
#endif

//std::string type2str(int type) {
//  std::string r;

//  uchar depth = type & CV_MAT_DEPTH_MASK;
//  uchar chans = 1 + (type >> CV_CN_SHIFT);

//  switch ( depth ) {
//    case CV_8U:  r = "8U"; break;
//    case CV_8S:  r = "8S"; break;
//    case CV_16U: r = "16U"; break;
//    case CV_16S: r = "16S"; break;
//    case CV_32S: r = "32S"; break;
//    case CV_32F: r = "32F"; break;
//    case CV_64F: r = "64F"; break;
//    default:     r = "User"; break;
//  }

//  r += "C";
//  r += (chans+'0');

//  return r;
//}

//******************************************
Magrathea::Magrathea(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Magrathea)
{
    ui->setupUi(this);

    //------------------------------------------
    //output log
    outputLogTextEdit = new QTextEdit;
    outputLogTextEdit->setReadOnly(true);
    ui->leftTabWidget->addTab(outputLogTextEdit, "log");
    ui->leftTabWidget->setCurrentWidget(outputLogTextEdit);

    //------------------------------------------
#ifdef VANCOUVER
    qInfo("Vancouver, Aerotech A3200 gantry");
    mMotionHandler = new AerotechMotionHandler();
#elif VALENCIA
    qInfo("Valencia, ACSC gantry");
    mMotionHandler = new ACSCMotionHandler();
#else
#define DEBUG
    qInfo("where is your gantry?");
    mMotionHandler = new MotionHandler();
#endif

    //------------------------------------------
    //font
    //QFont font=ui->pushButton->property("font").value<QFont>();
    //qDebug()<<font.family()<<font.pointSize();
    QFont font("");
    font.setStyleHint(QFont::Monospace);

    //position
    ui->xAxisPositionLine->setFont(font);
    ui->yAxisPositionLine->setFont(font);
    ui->zAxisPositionLine->setFont(font);
    ui->z_2_AxisPositionLine->setFont(font);
    ui->uAxisPositionLine->setFont(font);

    ui->xAxisPositionLine2->setFont(font);
    ui->yAxisPositionLine2->setFont(font);
    ui->zAxisPositionLine2->setFont(font);
    ui->z_2_AxisPositionLine2->setFont(font);
    ui->uAxisPositionLine2->setFont(font);

    //step move
    ui->xAxisStepDoubleSpinBox->setFont(font);
    ui->xAxisStepDoubleSpinBox->setValue(10.0);
    ui->xAxisStepDoubleSpinBox->setMinimum(-1000.0);
    ui->xAxisStepDoubleSpinBox->setMaximum(1000.0);
    ui->xAxisStepDoubleSpinBox->setDecimals(3);
    ui->xAxisStepDoubleSpinBox->setAlignment(Qt::AlignRight);
    ui->yAxisStepDoubleSpinBox->setFont(font);
    ui->yAxisStepDoubleSpinBox->setValue(10.0);
    ui->yAxisStepDoubleSpinBox->setMinimum(-1000.0);
    ui->yAxisStepDoubleSpinBox->setMaximum(1000.0);
    ui->yAxisStepDoubleSpinBox->setDecimals(3);
    ui->yAxisStepDoubleSpinBox->setAlignment(Qt::AlignRight);
    ui->zAxisStepDoubleSpinBox->setFont(font);
    ui->zAxisStepDoubleSpinBox->setValue(10.0);
    ui->zAxisStepDoubleSpinBox->setMinimum(-300.0);
    ui->zAxisStepDoubleSpinBox->setMaximum(300.0);
    ui->zAxisStepDoubleSpinBox->setDecimals(3);
    ui->zAxisStepDoubleSpinBox->setAlignment(Qt::AlignRight);
    ui->z_2_AxisStepDoubleSpinBox->setFont(font);
    ui->z_2_AxisStepDoubleSpinBox->setValue(10.0);
    ui->z_2_AxisStepDoubleSpinBox->setMinimum(-100.0);
    ui->z_2_AxisStepDoubleSpinBox->setMaximum(100.0);
    ui->z_2_AxisStepDoubleSpinBox->setDecimals(3);
    ui->z_2_AxisStepDoubleSpinBox->setAlignment(Qt::AlignRight);
    ui->uAxisStepDoubleSpinBox->setFont(font);
    ui->uAxisStepDoubleSpinBox->setValue(10.0);
    ui->uAxisStepDoubleSpinBox->setDecimals(3);
    ui->uAxisStepDoubleSpinBox->setAlignment(Qt::AlignRight);

    //position move
    ui->xAxisPositionMoveDoubleSpinBox->setFont(font);
    ui->xAxisPositionMoveDoubleSpinBox->setValue(0.0);
    ui->xAxisPositionMoveDoubleSpinBox->setMinimum(-1000.0);
    ui->xAxisPositionMoveDoubleSpinBox->setMaximum(1000.0);
    ui->xAxisPositionMoveDoubleSpinBox->setDecimals(3);
    ui->xAxisPositionMoveDoubleSpinBox->setAlignment(Qt::AlignRight);
    ui->yAxisPositionMoveDoubleSpinBox->setFont(font);
    ui->yAxisPositionMoveDoubleSpinBox->setValue(0.0);
    ui->yAxisPositionMoveDoubleSpinBox->setMinimum(-1000.0);
    ui->yAxisPositionMoveDoubleSpinBox->setMaximum(1000.0);
    ui->yAxisPositionMoveDoubleSpinBox->setDecimals(3);
    ui->yAxisPositionMoveDoubleSpinBox->setAlignment(Qt::AlignRight);
    ui->zAxisPositionMoveDoubleSpinBox->setFont(font);
    ui->zAxisPositionMoveDoubleSpinBox->setValue(0.0);
    ui->zAxisPositionMoveDoubleSpinBox->setMinimum(-300.0);
    ui->zAxisPositionMoveDoubleSpinBox->setMaximum(300.0);
    ui->zAxisPositionMoveDoubleSpinBox->setDecimals(3);
    ui->zAxisPositionMoveDoubleSpinBox->setAlignment(Qt::AlignRight);
    ui->z_2_AxisPositionMoveDoubleSpinBox->setFont(font);
    ui->z_2_AxisPositionMoveDoubleSpinBox->setValue(0.0);
    ui->z_2_AxisPositionMoveDoubleSpinBox->setMinimum(-100.0);
    ui->z_2_AxisPositionMoveDoubleSpinBox->setMaximum(100.0);
    ui->z_2_AxisPositionMoveDoubleSpinBox->setDecimals(3);
    ui->z_2_AxisPositionMoveDoubleSpinBox->setAlignment(Qt::AlignRight);
    ui->uAxisPositionMoveDoubleSpinBox->setFont(font);
    ui->uAxisPositionMoveDoubleSpinBox->setValue(0.0);
    ui->uAxisPositionMoveDoubleSpinBox->setMinimum(0.0);
    ui->uAxisPositionMoveDoubleSpinBox->setMaximum(360.0);
    ui->uAxisPositionMoveDoubleSpinBox->setDecimals(3);
    ui->uAxisPositionMoveDoubleSpinBox->setAlignment(Qt::AlignRight);

    //speed
    ui->xAxisSpeedDoubleSpinBox->setFont(font);
    ui->xAxisSpeedDoubleSpinBox->setValue(10.0);
    ui->xAxisSpeedDoubleSpinBox->setMinimum(0.0);
    ui->xAxisSpeedDoubleSpinBox->setMaximum(300.0);
    ui->xAxisSpeedDoubleSpinBox->setDecimals(1);
    ui->xAxisSpeedDoubleSpinBox->setAlignment(Qt::AlignRight);
    ui->yAxisSpeedDoubleSpinBox->setFont(font);
    ui->yAxisSpeedDoubleSpinBox->setValue(10.0);
    ui->yAxisSpeedDoubleSpinBox->setMinimum(0.0);
    ui->yAxisSpeedDoubleSpinBox->setMaximum(300.0);
    ui->yAxisSpeedDoubleSpinBox->setDecimals(1);
    ui->yAxisSpeedDoubleSpinBox->setAlignment(Qt::AlignRight);
    ui->zAxisSpeedDoubleSpinBox->setFont(font);
    ui->zAxisSpeedDoubleSpinBox->setValue(10.0);
    ui->zAxisSpeedDoubleSpinBox->setMinimum(0.0);
    ui->zAxisSpeedDoubleSpinBox->setMaximum(300.0);
    ui->zAxisSpeedDoubleSpinBox->setDecimals(1);
    ui->zAxisSpeedDoubleSpinBox->setAlignment(Qt::AlignRight);
    ui->z_2_AxisSpeedDoubleSpinBox->setFont(font);
    ui->z_2_AxisSpeedDoubleSpinBox->setValue(10.0);
    ui->z_2_AxisSpeedDoubleSpinBox->setMinimum(0.0);
    ui->z_2_AxisSpeedDoubleSpinBox->setMaximum(50.0);
    ui->z_2_AxisSpeedDoubleSpinBox->setDecimals(1);
    ui->z_2_AxisSpeedDoubleSpinBox->setAlignment(Qt::AlignRight);
    ui->uAxisSpeedDoubleSpinBox->setFont(font);
    ui->uAxisSpeedDoubleSpinBox->setValue(10.0);
    ui->uAxisSpeedDoubleSpinBox->setMinimum(0.0);
    ui->uAxisSpeedDoubleSpinBox->setMaximum(300.0);
    ui->uAxisSpeedDoubleSpinBox->setDecimals(1);
    ui->uAxisSpeedDoubleSpinBox->setAlignment(Qt::AlignRight);

    //------------------------------------------
    //timer
    mPositionTimer = new QTimer(this);
    connect(mPositionTimer, SIGNAL(timeout()), this, SLOT(updatePosition()));
    mPositionTimer->start(100);//ms

    //------------------------------------------
    //camera

    //create camera objects
    mCamera = new QCamera(this);
    mCameraViewfinder = new QCameraViewfinder(this);
    mCameraImageCapture = new QCameraImageCapture(mCamera, this);
    mCameraLayout = new QVBoxLayout;

    //add the camera to the layout
    mCamera->setViewfinder(mCameraViewfinder);
    mCameraLayout->addWidget(mCameraViewfinder);
    mCameraLayout->setContentsMargins(0,0,0,0);

    //add the layout to the frame area in the GUI
    ui->frame->setLayout(mCameraLayout);

    //////////////////7
    f_locations = new fiducial_locations();
    ///////////////////

    //------------------------------------------
    //gantry
    autoRepeatDelay=1000;//ms
    autoRepeatInterval=1000;//ms
    ui->enableAxesButton->setEnabled(false);
    ui->disableAxesButton->setEnabled(false);
    ui->resetErrorButton->setEnabled(false);

    //------------------------------------------
    //position tab
    ui->xAxisPositionLine2->setReadOnly(true);
    ui->yAxisPositionLine2->setReadOnly(true);
    ui->zAxisPositionLine2->setReadOnly(true);
    ui->z_2_AxisPositionLine2->setReadOnly(true);
    ui->uAxisPositionLine2->setReadOnly(true);

    //------------------------------------------
    //navigation tab
    ui->xAxisPositionLine->setReadOnly(true);
    ui->yAxisPositionLine->setReadOnly(true);
    ui->zAxisPositionLine->setReadOnly(true);
    ui->z_2_AxisPositionLine->setReadOnly(true);
    ui->uAxisPositionLine->setReadOnly(true);

    //------------------------------------------
    //connect signals and slots

    //camera
    connect(ui->enableCameraBox, SIGNAL(toggled(bool)), this, SLOT(enableCameraBoxClicked(bool)));
    connect(ui->focusButton,     SIGNAL(clicked(bool)), this, SLOT(focusButtonClicked()));
    connect(ui->captureButton,   SIGNAL(clicked(bool)), this, SLOT(captureButtonClicked()));
    connect(ui->Calib_button,    SIGNAL(clicked(bool)), this, SLOT(Calibration_ButtonClicked()));
    connect(ui->Calib_button_2,  SIGNAL(clicked(bool)), this, SLOT(Calibration_2_ButtonClicked()));
    connect(ui->pushButton_dummy,SIGNAL(clicked(bool)), this, SLOT(Camera_test()));
    connect(ui->focusButton     ,SIGNAL(clicked(bool)), this, SLOT(focusButtonClicked()));
    connect(ui->std_dev_button  ,SIGNAL(clicked(bool)), this, SLOT(focusButtonClicked()));
    connect(ui->std_dev_many_button,SIGNAL(clicked(bool)), this, SLOT(focusButtonClicked()));
    connect(ui->Fiducial_finder_button_1,SIGNAL(clicked(bool)), this, SLOT(Fiducial_finder_button_1_Clicked()));
    connect(ui->Fiducial_finder_button_2,SIGNAL(clicked(bool)), this, SLOT(Fiducial_finder_button_2_Clicked()));
    connect(ui->VignetteButton,SIGNAL(clicked(bool)),this,SLOT(VignetteButton_clicked()));

    //gantry
    connect(ui->connectGantryBox, SIGNAL(toggled(bool)), this, SLOT(connectGantryBoxClicked(bool)));
    connect(ui->enableAxesButton, SIGNAL(clicked(bool)), this, SLOT(enableAxesClicked()));
    connect(ui->disableAxesButton,SIGNAL(clicked(bool)), this, SLOT(enableAxesClicked()));
    connect(ui->resetErrorButton, &QPushButton::clicked, mMotionHandler, &MotionHandler::acknowledgeMotionFaultGantry);
    connect(ui->stopButton, &QPushButton::clicked, mMotionHandler, &MotionHandler::stop);
    connect(ui->EnableButton_X,SIGNAL(clicked(bool)), this, SLOT(AxisEnableDisableButton()));
    connect(ui->EnableButton_Y,SIGNAL(clicked(bool)), this, SLOT(AxisEnableDisableButton()));
    connect(ui->EnableButton_Z,SIGNAL(clicked(bool)), this, SLOT(AxisEnableDisableButton()));
    connect(ui->EnableButton_Z_2,SIGNAL(clicked(bool)), this, SLOT(AxisEnableDisableButton()));
    connect(ui->EnableButton_U,SIGNAL(clicked(bool)), this, SLOT(AxisEnableDisableButton()));

    //joystick
    connect(ui->freeRunRadioButton, SIGNAL(clicked(bool)), this, SLOT(enableJoystickFreeRun(bool)));
    connect(ui->stepRadioButton,    SIGNAL(clicked(bool)), this, SLOT(enableJoystickStepMotion(bool)));
    //SHORTCUTS
    //    shortcut_PX   = Qshortcut(QKeySequence("Alt+W"),this);
    //    shortcut_NX   = Qshortcut(QKeySequence("Alt+S"),this);
    //    shortcut_PY   = Qshortcut(QKeySequence("Alt+A"),this);
    //    shortcut_NY   = Qshortcut(QKeySequence("Alt+D"),this);
    //    shortcut_PZ   = Qshortcut(QKeySequence("Alt+Q"),this);
    //    shortcut_NZ   = Qshortcut(QKeySequence("Alt+Z"),this);
    //    shortcut_PZ_2 = Qshortcut(QKeySequence("Alt+E"),this);
    //    shortcut_NZ_2 = Qshortcut(QKeySequence("Alt+C"),this);
    //    shortcut_PU   = Qshortcut(QKeySequence("Alt+R"),this);
    //    shortcut_NU   = Qshortcut(QKeySequence("Alt+V"),this);
    //    shortcut_STOP = Qshortcut(QKeySequence("Alt+X"),this);

    //home axes
    connect(ui->axesHomeButton,  &QPushButton::clicked, mMotionHandler, &MotionHandler::home);
    connect(ui->xAxisHomeButton, &QPushButton::clicked, mMotionHandler, &MotionHandler::homeX);
    connect(ui->yAxisHomeButton, &QPushButton::clicked, mMotionHandler, &MotionHandler::homeY);
    connect(ui->zAxisHomeButton, &QPushButton::clicked, mMotionHandler, &MotionHandler::homeZ);
    connect(ui->z_2_AxisHomeButton, &QPushButton::clicked, mMotionHandler, &MotionHandler::homeZ_2);
    connect(ui->uAxisHomeButton, &QPushButton::clicked, mMotionHandler, &MotionHandler::homeU);

    //position move
    connect(ui->xAxisPositionMoveButton, SIGNAL(clicked(bool)), this, SLOT(positionMove()));
    connect(ui->yAxisPositionMoveButton, SIGNAL(clicked(bool)), this, SLOT(positionMove()));
    connect(ui->zAxisPositionMoveButton, SIGNAL(clicked(bool)), this, SLOT(positionMove()));
    connect(ui->z_2_AxisPositionMoveButton, SIGNAL(clicked(bool)), this, SLOT(positionMove()));
    connect(ui->uAxisPositionMoveButton, SIGNAL(clicked(bool)), this, SLOT(positionMove()));

    //step motion
    connect(ui->xAxisStepMoveButton, SIGNAL(clicked(bool)), this, SLOT(stepMotion()));
    connect(ui->yAxisStepMoveButton, SIGNAL(clicked(bool)), this, SLOT(stepMotion()));
    connect(ui->zAxisStepMoveButton, SIGNAL(clicked(bool)), this, SLOT(stepMotion()));
    connect(ui->z_2_AxisStepMoveButton, SIGNAL(clicked(bool)), this, SLOT(stepMotion()));
    connect(ui->uAxisStepMoveButton, SIGNAL(clicked(bool)), this, SLOT(stepMotion()));

    //step motion autorepeat box
    connect(ui->xAxisStepRepeatBox, SIGNAL(clicked(bool)), this, SLOT(axisStepRepeatBoxClicked(bool)));
    connect(ui->yAxisStepRepeatBox, SIGNAL(clicked(bool)), this, SLOT(axisStepRepeatBoxClicked(bool)));
    connect(ui->zAxisStepRepeatBox, SIGNAL(clicked(bool)), this, SLOT(axisStepRepeatBoxClicked(bool)));
    connect(ui->z_2_AxisStepRepeatBox, SIGNAL(clicked(bool)), this, SLOT(axisStepRepeatBoxClicked(bool)));
    connect(ui->uAxisStepRepeatBox, SIGNAL(clicked(bool)), this, SLOT(axisStepRepeatBoxClicked(bool)));

    //test
    connect(ui->color_button,SIGNAL(clicked(bool)), this, SLOT(color_test()));
    connect(ui->destroy_Button,SIGNAL(clicked(bool)), this, SLOT(destroy_all()));
    connect(ui->f_loop_button,SIGNAL(clicked(bool)), this, SLOT(loop_test()));
    connect(ui->DelLogButton,SIGNAL(clicked(bool)),outputLogTextEdit,SLOT(clear()));
}

//******************************************
Magrathea::~Magrathea()
{
    delete ui;
    delete mMotionHandler;
}

//******************************************
//timer

//position update
void Magrathea::updatePosition(){
    std::vector <double> pos_t = mMotionHandler->whereAmI();

    ui->xAxisPositionLine->setText(QString::number(    pos_t[0], 'f', 3));
    ui->yAxisPositionLine->setText(QString::number(    pos_t[1], 'f', 3));
    ui->zAxisPositionLine->setText(QString::number(    pos_t[2], 'f', 3));
    ui->z_2_AxisPositionLine->setText(QString::number( pos_t[4], 'f', 3));
    ui->uAxisPositionLine->setText(QString::number(    pos_t[3], 'f', 3));
    ui->xAxisPositionLine2->setText(QString::number(   pos_t[0], 'f', 3));
    ui->yAxisPositionLine2->setText(QString::number(   pos_t[1], 'f', 3));
    ui->zAxisPositionLine2->setText(QString::number(   pos_t[2], 'f', 3));
    ui->z_2_AxisPositionLine2->setText(QString::number(pos_t[4], 'f', 3));
    ui->uAxisPositionLine2->setText(QString::number(   pos_t[3], 'f', 3));

//    if(mMotionHandler->validate_target_pos(pos_t.at(0),pos_t.at(1),pos_t.at(2),pos_t.at(4))){
//        mMotionHandler->stop();
//    }

//    ui->xAxisPositionLine->setText(QString::number(mMotionHandler->whereAmI()[0], 'f', 3));
//    ui->yAxisPositionLine->setText(QString::number(mMotionHandler->whereAmI()[1], 'f', 3));
//    ui->zAxisPositionLine->setText(QString::number(mMotionHandler->whereAmI()[2], 'f', 3));
//    ui->z_2_AxisPositionLine->setText(QString::number(mMotionHandler->whereAmI()[4], 'f', 3));
//    ui->uAxisPositionLine->setText(QString::number(mMotionHandler->whereAmI()[3], 'f', 3));
//    ui->xAxisPositionLine2->setText(QString::number(mMotionHandler->whereAmI()[0], 'f', 3));
//    ui->yAxisPositionLine2->setText(QString::number(mMotionHandler->whereAmI()[1], 'f', 3));
//    ui->zAxisPositionLine2->setText(QString::number(mMotionHandler->whereAmI()[2], 'f', 3));
//    ui->z_2_AxisPositionLine2->setText(QString::number(mMotionHandler->whereAmI()[4], 'f', 3));
//    ui->uAxisPositionLine2->setText(QString::number(mMotionHandler->whereAmI()[3], 'f', 3));

    //axes status update
    bool current =  mMotionHandler->getXAxisState();
    led_label(ui->label_8,  mMotionHandler->getXAxisState());
    ui->EnableButton_X->setText((current ? "Disable" : "Enable"));

    current =  mMotionHandler->getYAxisState();
    led_label(ui->label_10, current);
    ui->EnableButton_Y->setText((current ? "Disable" : "Enable"));

    current =  mMotionHandler->getZAxisState();
    led_label(ui->label_12, current);
    ui->EnableButton_Z->setText((current ? "Disable" : "Enable"));

    current =  mMotionHandler->getZ_2_AxisState();
    led_label(ui->label_14, current);
    ui->EnableButton_Z_2->setText((current ? "Disable" : "Enable"));

    current =  mMotionHandler->getUAxisState();
    led_label(ui->label_16, current);
    ui->EnableButton_U->setText((current ? "Disable" : "Enable"));
    return;
}

//******************************************
//camera

//------------------------------------------
//enable
void Magrathea::enableCameraBoxClicked(bool clicked)
{
    if (clicked) mCamera->start();
    else mCamera->stop();
    return;
}

//------------------------------------------
//focus
void Magrathea::focusButtonClicked()
{
    qInfo(" > camera focus ... ");

    Focus_finder * FocusFinder = new Focus_finder(this);
    mCamera->stop(); //closing QCamera

    cv::VideoCapture cap(ui->spinBox_dummy->value()); // open the video camera no. 0
    if (!cap.isOpened()){
        //    if(!cap.open(0)){     //Opening opencv-camera, needed for easier image manipulation
        QMessageBox::critical(this, tr("Error"), tr("Could not open camera"));
        return;}
    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    qInfo("Frame size : %6.0f x %6.0f",dWidth,dHeight);
    cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', 'U', 'Y', 'V'));
    //cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', '8', '0', '0'));
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 3856);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 2764);
    cap.set(CV_CAP_PROP_FPS, 5.0);
    dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
    qInfo("Frame size : %6.0f x %6.0f",dWidth,dHeight);

    FocusFinder->Set_camera(cap);
    FocusFinder->Set_gantry(mMotionHandler);
    FocusFinder->Set_log(outputLogTextEdit);
    double focus_position = -1.;
    FocusFinder->Set_color_int(ui->ColorBox->value());
    FocusFinder->Set_use_laplacian(ui->LaplacianBox->isChecked());
    if(sender() == ui->focusButton){
    FocusFinder->find_focus(focus_position);
    } else if (sender() == ui->std_dev_button){
        cv::Mat mat_from_camera;
        bool bSuccess = cap.read(mat_from_camera);
        if (!bSuccess){ //if not success
            qInfo("Cannot read a frame from video stream");
            return;
        }
        double value_std_dev = FocusFinder->eval_stddev_ROI(mat_from_camera);
        qInfo(" std dev value : %5.5f",value_std_dev);
    } else if(sender() == ui->std_dev_many_button){
        FocusFinder->Eval_syst_scan();
    }
    qInfo(" > camera focus : %3.5f",focus_position);
    delete FocusFinder;
    cap.release();         //Going back to QCameraa
    mCamera->start();
    return;
}

//------------------------------------------
//capture picture
void Magrathea::captureButtonClicked()
{
    auto filename = QFileDialog::getSaveFileName(this, "capture", "/ ","image (*.jpg;*.jpeg)");
    if (filename.isEmpty()) {
        return;
    }
    mCameraImageCapture->setCaptureDestination(QCameraImageCapture::CaptureToFile);
    QImageEncoderSettings imageEncoderSettings;
    imageEncoderSettings.setCodec("image/jpeg");
    //imageEncoderSettings.setResolution(1600, 1200);
    imageEncoderSettings.setResolution(3856, 2764);
    mCameraImageCapture->setEncodingSettings(imageEncoderSettings);
    mCamera->setCaptureMode(QCamera::CaptureStillImage);
    mCamera->start();
    mCamera->searchAndLock();
    mCameraImageCapture->capture(filename);
    mCamera->unlock();
    return;
}
//------------------------------------------
//------------------------------------------
//https://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html#videocapture-open
void Magrathea::Camera_test(){
    //CvCapture* capture = cvCaptureFromCAM(CV_CAP_DSHOW);

    cv::VideoCapture cap(0); // open the video camera no. 0

    if (!cap.isOpened())  // if not success, exit program
    {
        qWarning("Cannot open the video cam");
        _getch();
        return;
    }

    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    qInfo("Frame size : %6.0f x %6.0f",dWidth,dHeight);

    cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', 'U', 'Y', 'V'));
    //cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', '8', '0', '0'));
    //cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', 'U', 'Y', '2'));//https://www.fourcc.org/yuv.php
    //cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', 'U', 'Y', 'V'));

    //cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 3856);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 2764);
    cap.set(CV_CAP_PROP_FPS, 4.0);

    dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    qInfo("Frame size : %6.0f x %6.0f",dWidth,dHeight);

    qInfo("cap.get(CV_CAP_PROP_POS_MSEC);      : %5.5f",cap.get(CV_CAP_PROP_POS_MSEC));
    qInfo("cap.get(CV_CAP_PROP_POS_FRAMES );   : %5.5f",cap.get(CV_CAP_PROP_POS_FRAMES ));
    qInfo("cap.get(CV_CAP_PROP_POS_AVI_RATIO); : %5.5f",cap.get(CV_CAP_PROP_POS_AVI_RATIO));
    qInfo("cap.get(CV_CAP_PROP_FRAME_WIDTH );  : %5.5f",cap.get(CV_CAP_PROP_FRAME_WIDTH ));
    qInfo("cap.get(CV_CAP_PROP_FRAME_HEIGHT);  : %5.5f",cap.get(CV_CAP_PROP_FRAME_HEIGHT));
    qInfo("cap.get(CV_CAP_PROP_FPS );          : %5.5f",cap.get(CV_CAP_PROP_FPS ));
    qInfo("cap.get(CV_CAP_PROP_FOURCC);        : %5.5f",cap.get(CV_CAP_PROP_FOURCC));
    qInfo("cap.get(CV_CAP_PROP_FRAME_COUNT );  : %5.5f",cap.get(CV_CAP_PROP_FRAME_COUNT ));
    qInfo("cap.get(CV_CAP_PROP_FORMAT );       : %5.5f",cap.get(CV_CAP_PROP_FORMAT ));
    qInfo("cap.get(CV_CAP_PROP_MODE );         : %5.5f",cap.get(CV_CAP_PROP_MODE ));
    qInfo("cap.get(CV_CAP_PROP_BRIGHTNESS);    : %5.5f",cap.get(CV_CAP_PROP_BRIGHTNESS));
    qInfo("cap.get(CV_CAP_PROP_CONTRAST);      : %5.5f",cap.get(CV_CAP_PROP_CONTRAST));
    qInfo("cap.get(CV_CAP_PROP_SATURATION);    : %5.5f",cap.get(CV_CAP_PROP_SATURATION));
    qInfo("cap.get(CV_CAP_PROP_HUE);           : %5.5f",cap.get(CV_CAP_PROP_HUE));
    qInfo("cap.get(CV_CAP_PROP_GAIN);          : %5.5f",cap.get(CV_CAP_PROP_GAIN));
    qInfo("cap.get(CV_CAP_PROP_EXPOSURE);      : %5.5f",cap.get(CV_CAP_PROP_EXPOSURE));
    qInfo("cap.get(CV_CAP_PROP_CONVERT_RGB);   : %5.5f",cap.get(CV_CAP_PROP_CONVERT_RGB));
    qInfo("cap.get(CV_CAP_PROP_RECTIFICATION); : %5.5f",cap.get(CV_CAP_PROP_RECTIFICATION));
    qInfo("cap.get(CV_CAP_PROP_ISO_SPEED );    : %5.5f",cap.get(CV_CAP_PROP_ISO_SPEED ));
    qInfo("cap.get(CV_CAP_PROP_BUFFERSIZE );   : %5.5f",cap.get(CV_CAP_PROP_BUFFERSIZE ));

    cv::namedWindow("MyVideo", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
    bool bEnd = false;
    int img = 10;
    while (!bEnd && img > 0)
    {
        cv::Mat frame;

        bool bSuccess = cap.read(frame); // read a new frame from video

        if (!bSuccess) //if not success, break loop
        {
            qInfo("Cannot read a frame from video stream");
            break;
        }

        cv::imshow("MyVideo", frame); //show the frame in "MyVideo" window

        if (cv::waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            qInfo("esc key is pressed by user");
            bEnd = true;
        }
        img--;
        qInfo("Image : %i",img);
    }
    cap.release();
    qInfo("end");
    _getch();
    return;
}

//------------------------------------------
//------------------------------------------
void Magrathea::VignetteButton_clicked(){
    cv::destroyAllWindows();
    mCamera->stop(); //closing QCamera
    cv::VideoCapture cap(ui->spinBox_dummy->value()); // open the video camera no. 0
    if (!cap.isOpened()){
        QMessageBox::critical(this, tr("Error"), tr("Could not open camera"));
        return;}
    cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', 'U', 'Y', 'V'));
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 3856);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 2764);
    cap.set(CV_CAP_PROP_FPS, 5.0);
    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
    qInfo("Frame size : %6.0f x %6.0f",dWidth,dHeight);
    VerticalAlignmentTool * tool = new VerticalAlignmentTool(this);
    tool->Set_camera(cap);
    tool->Evaluate_vignette();
    delete tool;
    cap.release();         //Going back to QCameraa
    mCamera->start();
    return;
}


//------------------------------------------
//------------------------------------------


void Magrathea::Fiducial_finder_button_1_Clicked()
{    FiducialFinderCaller(1); }

void Magrathea::Fiducial_finder_button_2_Clicked()
{    FiducialFinderCaller(2); }


void Magrathea::FiducialFinderCaller(const int &input){
    cv::destroyAllWindows();
    mCamera->stop(); //closing QCamera

    //opening camera with opencv
    cv::Mat mat_from_camera;
    cv::VideoCapture cap(ui->spinBox_dummy->value()); // open the video camera no. 0
    if (!cap.isOpened()){
        //Opening opencv-camera, needed for easier image manipulation
        QMessageBox::critical(this, tr("Error"), tr("Could not open camera"));
        return;}
    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    qInfo("Frame size : %6.0f x %6.0f",dWidth,dHeight);
    cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', 'U', 'Y', 'V'));
    //cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', '8', '0', '0'));
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 3856);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 2764);
    cap.set(CV_CAP_PROP_FPS, 4.0);
    dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
    qInfo("Frame size : %6.0f x %6.0f",dWidth,dHeight);

    FiducialFinder * Ffinder = new FiducialFinder(this);

    bool from_file = ui->calib_from_file_Box->isChecked();
    std::string tmp_filename = "";
    if(from_file){
        std::string address = "C:/Users/Silicio/WORK/MODULE_ON_CORE/medidas_fiduciales_CNM/Imagenes_fiduciales/mag_15X/Sensor_defectos/Todas/Aruco_M/";
        //std::string address = "C:/Users/Silicio/WORK/MODULE_ON_CORE/medidas_fiduciales_CNM/Imagenes_fiduciales/mag_15X/Sensor_estandar/Todas/Aruco_M/";
        std::string Images[] = {
"chip_1_10_pos_1.TIF",
"chip_1_10_pos_2.TIF",
"chip_1_10_pos_3.TIF",
"chip_1_10_pos_4.TIF",
"chip_1_10_pos_5.TIF",
"chip_1_10_pos_6.TIF",
"chip_1_10_pos_7.TIF",
"chip_1_10_pos_8.TIF",
"chip_1_11_pos_1.TIF",
"chip_1_11_pos_2.TIF",
"chip_1_11_pos_3.TIF",
"chip_1_11_pos_4.TIF",
"chip_1_11_pos_5.TIF",
"chip_1_11_pos_6.TIF",
"chip_1_11_pos_7.TIF",
"chip_1_11_pos_8.TIF",
"chip_1_1_pos_1.TIF",
"chip_1_1_pos_2.TIF",
"chip_1_1_pos_3.TIF",
"chip_1_1_pos_4.TIF",
"chip_1_1_pos_5.TIF",
"chip_1_1_pos_6.TIF",
"chip_1_1_pos_7.TIF",
"chip_1_1_pos_8.TIF",
"chip_1_2_pos_1.TIF",
"chip_1_2_pos_2.TIF",
"chip_1_2_pos_3.TIF",
"chip_1_2_pos_4.TIF",
"chip_1_2_pos_5.TIF",
"chip_1_2_pos_6.TIF",
"chip_1_2_pos_7.TIF",
"chip_1_2_pos_8.TIF",
"chip_1_3_pos_1.TIF",
"chip_1_3_pos_2.TIF",
"chip_1_3_pos_3.TIF",
"chip_1_3_pos_4.TIF",
"chip_1_3_pos_5.TIF",
"chip_1_3_pos_6.TIF",
"chip_1_3_pos_7.TIF",
"chip_1_3_pos_8.TIF",
"chip_1_4_pos_1.TIF",
"chip_1_4_pos_2.TIF",
"chip_1_4_pos_3.TIF",
"chip_1_4_pos_4.TIF",
"chip_1_4_pos_5.TIF",
"chip_1_4_pos_6.TIF",
"chip_1_4_pos_7.TIF",
"chip_1_4_pos_8.TIF",
"chip_1_5_pos_1.TIF",
"chip_1_5_pos_2.TIF",
"chip_1_5_pos_3.TIF",
"chip_1_5_pos_4.TIF",
"chip_1_5_pos_5.TIF",
"chip_1_5_pos_6.TIF",
"chip_1_5_pos_7.TIF",
"chip_1_5_pos_8.TIF",
"chip_1_6_pos_1.TIF",
"chip_1_6_pos_2.TIF",
"chip_1_6_pos_3.TIF",
"chip_1_6_pos_4.TIF",
"chip_1_6_pos_5.TIF",
"chip_1_6_pos_6.TIF",
"chip_1_6_pos_7.TIF",
"chip_1_6_pos_8.TIF",
"chip_1_7_pos_1.TIF",
"chip_1_7_pos_2.TIF",
"chip_1_7_pos_3.TIF",
"chip_1_7_pos_4.TIF",
"chip_1_7_pos_5.TIF",
"chip_1_7_pos_6.TIF",
"chip_1_7_pos_7.TIF",
"chip_1_7_pos_8.TIF",
"chip_1_8_pos_1.TIF",
"chip_1_8_pos_2.TIF",
"chip_1_8_pos_3.TIF",
"chip_1_8_pos_4.TIF",
"chip_1_8_pos_5.TIF",
"chip_1_8_pos_6.TIF",
"chip_1_8_pos_7.TIF",
"chip_1_8_pos_8.TIF",
"chip_1_9_pos_1.TIF",
"chip_1_9_pos_2.TIF",
"chip_1_9_pos_3.TIF",
"chip_1_9_pos_4.TIF",
"chip_1_9_pos_5.TIF",
"chip_1_9_pos_6.TIF",
"chip_1_9_pos_7.TIF",
"chip_1_9_pos_8.TIF",
"chip_2_10_pos_1.TIF",
"chip_2_10_pos_2.TIF",
"chip_2_10_pos_3.TIF",
"chip_2_10_pos_4.TIF",
"chip_2_10_pos_5.TIF",
"chip_2_10_pos_6.TIF",
"chip_2_10_pos_7.TIF",
"chip_2_10_pos_8.TIF",
"chip_2_11_pos_1.TIF",
"chip_2_11_pos_2.TIF",
"chip_2_11_pos_3.TIF",
"chip_2_11_pos_4.TIF",
"chip_2_11_pos_5.TIF",
"chip_2_11_pos_6.TIF",
"chip_2_11_pos_7.TIF",
"chip_2_11_pos_8.TIF",
"chip_2_1_pos_1.TIF",
"chip_2_1_pos_2.TIF",
"chip_2_1_pos_3.TIF",
"chip_2_1_pos_4.TIF",
"chip_2_1_pos_5.TIF",
"chip_2_1_pos_6.TIF",
"chip_2_1_pos_7.TIF",
"chip_2_1_pos_8.TIF",
"chip_2_2_pos_1.TIF",
"chip_2_2_pos_2.TIF",
"chip_2_2_pos_3.TIF",
"chip_2_2_pos_4.TIF",
"chip_2_2_pos_5.TIF",
"chip_2_2_pos_6.TIF",
"chip_2_2_pos_7.TIF",
"chip_2_2_pos_8.TIF",
"chip_2_3_pos_1.TIF",
"chip_2_3_pos_2.TIF",
"chip_2_3_pos_3.TIF",
"chip_2_3_pos_4.TIF",
"chip_2_3_pos_5.TIF",
"chip_2_3_pos_6.TIF",
"chip_2_3_pos_7.TIF",
"chip_2_3_pos_8.TIF",
"chip_2_4_pos_1.TIF",
"chip_2_4_pos_2.TIF",
"chip_2_4_pos_3.TIF",
"chip_2_4_pos_4.TIF",
"chip_2_4_pos_5.TIF",
"chip_2_4_pos_6.TIF",
"chip_2_4_pos_7.TIF",
"chip_2_4_pos_8.TIF",
"chip_2_5_pos_1.TIF",
"chip_2_5_pos_2.TIF",
"chip_2_5_pos_3.TIF",
"chip_2_5_pos_4.TIF",
"chip_2_5_pos_5.TIF",
"chip_2_5_pos_6.TIF",
"chip_2_5_pos_7.TIF",
"chip_2_5_pos_8.TIF",
"chip_2_6_pos_1.TIF",
"chip_2_6_pos_2.TIF",
"chip_2_6_pos_3.TIF",
"chip_2_6_pos_4.TIF",
"chip_2_6_pos_5.TIF",
"chip_2_6_pos_6.TIF",
"chip_2_6_pos_7.TIF",
"chip_2_6_pos_8.TIF",
"chip_2_7_pos_1.TIF",
"chip_2_7_pos_2.TIF",
"chip_2_7_pos_3.TIF",
"chip_2_7_pos_4.TIF",
"chip_2_7_pos_5.TIF",
"chip_2_7_pos_6.TIF",
"chip_2_7_pos_7.TIF",
"chip_2_7_pos_8.TIF",
"chip_2_8_pos_1.TIF",
"chip_2_8_pos_2.TIF",
"chip_2_8_pos_3.TIF",
"chip_2_8_pos_4.TIF",
"chip_2_8_pos_5.TIF",
"chip_2_8_pos_6.TIF",
"chip_2_8_pos_7.TIF",
"chip_2_8_pos_8.TIF",
"chip_2_9_pos_1.TIF",
"chip_2_9_pos_2.TIF",
"chip_2_9_pos_3.TIF",
"chip_2_9_pos_4.TIF",
"chip_2_9_pos_5.TIF",
"chip_2_9_pos_6.TIF",
"chip_2_9_pos_7.TIF",
"chip_2_9_pos_8.TIF",
"chip_3_10_pos_1.TIF",
"chip_3_10_pos_2.TIF",
"chip_3_10_pos_3.TIF",
"chip_3_10_pos_4.TIF",
"chip_3_10_pos_5.TIF",
"chip_3_10_pos_6.TIF",
"chip_3_10_pos_7.TIF",
"chip_3_10_pos_8.TIF",
"chip_3_11_pos_1.TIF",
"chip_3_11_pos_2.TIF",
"chip_3_11_pos_3.TIF",
"chip_3_11_pos_4.TIF",
"chip_3_11_pos_5.TIF",
"chip_3_11_pos_6.TIF",
"chip_3_11_pos_7.TIF",
"chip_3_11_pos_8.TIF",
"chip_3_1_pos_1.TIF",
"chip_3_1_pos_2.TIF",
"chip_3_1_pos_3.TIF",
"chip_3_1_pos_4.TIF",
"chip_3_1_pos_5.TIF",
"chip_3_1_pos_6.TIF",
"chip_3_1_pos_7.TIF",
"chip_3_1_pos_8.TIF",
"chip_3_2_pos_1.TIF",
"chip_3_2_pos_2.TIF",
"chip_3_2_pos_3.TIF",
"chip_3_2_pos_4.TIF",
"chip_3_2_pos_5.TIF",
"chip_3_2_pos_6.TIF",
"chip_3_2_pos_7.TIF",
"chip_3_2_pos_8.TIF",
"chip_3_3_pos_1.TIF",
"chip_3_3_pos_2.TIF",
"chip_3_3_pos_3.TIF",
"chip_3_3_pos_4.TIF",
"chip_3_3_pos_5.TIF",
"chip_3_3_pos_6.TIF",
"chip_3_3_pos_7.TIF",
"chip_3_3_pos_8.TIF",
"chip_3_4_pos_1.TIF",
"chip_3_4_pos_2.TIF",
"chip_3_4_pos_3.TIF",
"chip_3_4_pos_4.TIF",
"chip_3_4_pos_5.TIF",
"chip_3_4_pos_6.TIF",
"chip_3_4_pos_7.TIF",
"chip_3_4_pos_8.TIF",
"chip_3_5_pos_1.TIF",
"chip_3_5_pos_2.TIF",
"chip_3_5_pos_3.TIF",
"chip_3_5_pos_4.TIF",
"chip_3_5_pos_5.TIF",
"chip_3_5_pos_6.TIF",
"chip_3_5_pos_7.TIF",
"chip_3_5_pos_8.TIF",
"chip_3_6_pos_1.TIF",
"chip_3_6_pos_2.TIF",
"chip_3_6_pos_3.TIF",
"chip_3_6_pos_4.TIF",
"chip_3_6_pos_5.TIF",
"chip_3_6_pos_6.TIF",
"chip_3_6_pos_7.TIF",
"chip_3_6_pos_8.TIF",
"chip_3_7_pos_1.TIF",
"chip_3_7_pos_2.TIF",
"chip_3_7_pos_3.TIF",
"chip_3_7_pos_4.TIF",
"chip_3_7_pos_5.TIF",
"chip_3_7_pos_6.TIF",
"chip_3_7_pos_7.TIF",
"chip_3_7_pos_8.TIF",
"chip_3_8_pos_1.TIF",
"chip_3_8_pos_2.TIF",
"chip_3_8_pos_3.TIF",
"chip_3_8_pos_4.TIF",
"chip_3_8_pos_5.TIF",
"chip_3_8_pos_6.TIF",
"chip_3_8_pos_7.TIF",
"chip_3_8_pos_8.TIF",
"chip_3_9_pos_1.TIF",
"chip_3_9_pos_2.TIF",
"chip_3_9_pos_3.TIF",
"chip_3_9_pos_4.TIF",
"chip_3_9_pos_5.TIF",
"chip_3_9_pos_6.TIF",
"chip_3_9_pos_7.TIF",
"chip_3_9_pos_8.TIF",
"chip_4_10_pos_1.TIF",
"chip_4_10_pos_2.TIF",
"chip_4_10_pos_3.TIF",
"chip_4_10_pos_4.TIF",
"chip_4_10_pos_5.TIF",
"chip_4_10_pos_6.TIF",
"chip_4_10_pos_7.TIF",
"chip_4_10_pos_8.TIF",
"chip_4_11_pos_1.TIF",
"chip_4_11_pos_2.TIF",
"chip_4_11_pos_3.TIF",
"chip_4_11_pos_4.TIF",
"chip_4_11_pos_5.TIF",
"chip_4_11_pos_6.TIF",
"chip_4_11_pos_7.TIF",
"chip_4_11_pos_8.TIF",
"chip_4_1_pos_1.TIF",
"chip_4_1_pos_2.TIF",
"chip_4_1_pos_3.TIF",
"chip_4_1_pos_4.TIF",
"chip_4_1_pos_5.TIF",
"chip_4_1_pos_6.TIF",
"chip_4_1_pos_7.TIF",
"chip_4_1_pos_8.TIF",
"chip_4_2_pos_1.TIF",
"chip_4_2_pos_2.TIF",
"chip_4_2_pos_3.TIF",
"chip_4_2_pos_4.TIF",
"chip_4_2_pos_5.TIF",
"chip_4_2_pos_6.TIF",
"chip_4_2_pos_7.TIF",
"chip_4_2_pos_8.TIF",
"chip_4_3_pos_1.TIF",
"chip_4_3_pos_2.TIF",
"chip_4_3_pos_3.TIF",
"chip_4_3_pos_4.TIF",
"chip_4_3_pos_5.TIF",
"chip_4_3_pos_6.TIF",
"chip_4_3_pos_7.TIF",
"chip_4_3_pos_8.TIF",
"chip_4_4_pos_1.TIF",
"chip_4_4_pos_2.TIF",
"chip_4_4_pos_3.TIF",
"chip_4_4_pos_4.TIF",
"chip_4_4_pos_5.TIF",
"chip_4_4_pos_6.TIF",
"chip_4_4_pos_7.TIF",
"chip_4_4_pos_8.TIF",
"chip_4_5_pos_1.TIF",
"chip_4_5_pos_2.TIF",
"chip_4_5_pos_3.TIF",
"chip_4_5_pos_4.TIF",
"chip_4_5_pos_5.TIF",
"chip_4_5_pos_6.TIF",
"chip_4_5_pos_7.TIF",
"chip_4_5_pos_8.TIF",
"chip_4_6_pos_1.TIF",
"chip_4_6_pos_2.TIF",
"chip_4_6_pos_3.TIF",
"chip_4_6_pos_4.TIF",
"chip_4_6_pos_5.TIF",
"chip_4_6_pos_6.TIF",
"chip_4_6_pos_7.TIF",
"chip_4_6_pos_8.TIF",
"chip_4_7_pos_1.TIF",
"chip_4_7_pos_2.TIF",
"chip_4_7_pos_3.TIF",
"chip_4_7_pos_4.TIF",
"chip_4_7_pos_5.TIF",
"chip_4_7_pos_6.TIF",
"chip_4_7_pos_7.TIF",
"chip_4_7_pos_8.TIF",
"chip_4_8_pos_1.TIF",
"chip_4_8_pos_2.TIF",
"chip_4_8_pos_3.TIF",
"chip_4_8_pos_4.TIF",
"chip_4_8_pos_5.TIF",
"chip_4_8_pos_6.TIF",
"chip_4_8_pos_7.TIF",
"chip_4_8_pos_8.TIF",
"chip_4_9_pos_1.TIF",
"chip_4_9_pos_2.TIF",
"chip_4_9_pos_3.TIF",
"chip_4_9_pos_4.TIF",
"chip_4_9_pos_5.TIF",
"chip_4_9_pos_6.TIF",
"chip_4_9_pos_7.TIF",
"chip_4_9_pos_8.TIF",
"chip_5_10_pos_1.TIF",
"chip_5_10_pos_2.TIF",
"chip_5_10_pos_3.TIF",
"chip_5_10_pos_4.TIF",
"chip_5_10_pos_5.TIF",
"chip_5_10_pos_6.TIF",
"chip_5_10_pos_7.TIF",
"chip_5_10_pos_8.TIF",
"chip_5_11_pos_1.TIF",
"chip_5_11_pos_2.TIF",
"chip_5_11_pos_3.TIF",
"chip_5_11_pos_4.TIF",
"chip_5_11_pos_5.TIF",
"chip_5_11_pos_6.TIF",
"chip_5_11_pos_7.TIF",
"chip_5_11_pos_8.TIF",
"chip_5_1_pos_1.TIF",
"chip_5_1_pos_2.TIF",
"chip_5_1_pos_3.TIF",
"chip_5_1_pos_4.TIF",
"chip_5_1_pos_5.TIF",
"chip_5_1_pos_6.TIF",
"chip_5_1_pos_7.TIF",
"chip_5_1_pos_8.TIF",
"chip_5_2_pos_1.TIF",
"chip_5_2_pos_2.TIF",
"chip_5_2_pos_3.TIF",
"chip_5_2_pos_4.TIF",
"chip_5_2_pos_5.TIF",
"chip_5_2_pos_6.TIF",
"chip_5_2_pos_7.TIF",
"chip_5_2_pos_8.TIF",
"chip_5_3_pos_1.TIF",
"chip_5_3_pos_2.TIF",
"chip_5_3_pos_3.TIF",
"chip_5_3_pos_4.TIF",
"chip_5_3_pos_5.TIF",
"chip_5_3_pos_6.TIF",
"chip_5_3_pos_7.TIF",
"chip_5_3_pos_8.TIF",
"chip_5_4_pos_1.TIF",
"chip_5_4_pos_2.TIF",
"chip_5_4_pos_3.TIF",
"chip_5_4_pos_4.TIF",
"chip_5_4_pos_5.TIF",
"chip_5_4_pos_6.TIF",
"chip_5_4_pos_7.TIF",
"chip_5_4_pos_8.TIF",
"chip_5_5_pos_1.TIF",
"chip_5_5_pos_2.TIF",
"chip_5_5_pos_3.TIF",
"chip_5_5_pos_4.TIF",
"chip_5_5_pos_5.TIF",
"chip_5_5_pos_6.TIF",
"chip_5_5_pos_7.TIF",
"chip_5_5_pos_8.TIF",
"chip_5_6_pos_1.TIF",
"chip_5_6_pos_2.TIF",
"chip_5_6_pos_3.TIF",
"chip_5_6_pos_4.TIF",
"chip_5_6_pos_5.TIF",
"chip_5_6_pos_6.TIF",
"chip_5_6_pos_7.TIF",
"chip_5_6_pos_8.TIF",
"chip_5_7_pos_1.TIF",
"chip_5_7_pos_2.TIF",
"chip_5_7_pos_3.TIF",
"chip_5_7_pos_4.TIF",
"chip_5_7_pos_5.TIF",
"chip_5_7_pos_6.TIF",
"chip_5_7_pos_7.TIF",
"chip_5_7_pos_8.TIF",
"chip_5_8_pos_1.TIF",
"chip_5_8_pos_2.TIF",
"chip_5_8_pos_3.TIF",
"chip_5_8_pos_4.TIF",
"chip_5_8_pos_5.TIF",
"chip_5_8_pos_6.TIF",
"chip_5_8_pos_7.TIF",
"chip_5_8_pos_8.TIF",
"chip_5_9_pos_1.TIF",
"chip_5_9_pos_2.TIF",
"chip_5_9_pos_3.TIF",
"chip_5_9_pos_4.TIF",
"chip_5_9_pos_5.TIF",
"chip_5_9_pos_6.TIF",
"chip_5_9_pos_7.TIF",
"chip_5_9_pos_8.TIF"
	};
        tmp_filename = Images[ui->spinBox_input->value()];
        Ffinder->SetImage(address + Images[ui->spinBox_input->value()]
                ,CV_LOAD_IMAGE_COLOR);
        Ffinder->Set_calibration(1); //get calibration from a private variable
    }else{
        bool bSuccess = cap.read(mat_from_camera);
        if (!bSuccess){ //if not success
            qInfo("Cannot read a frame from video stream");
            return;
        }
        Ffinder->Set_calibration(mCalibration); //get calibration from a private variable
        Ffinder->SetImage(mat_from_camera);
    }
    if(!from_file && mCalibration < 0.){
        qInfo("Calibration not set!! value is : %5.3f",mCalibration);
        return;
    }else{
        qInfo("Calibration value is : %5.3f [px/um]",mCalibration);
    }

    Ffinder->Set_log(outputLogTextEdit);

    double distance_x = 888888.8;
    double distance_y = 888888.8;

    if(input == 1){
        Ffinder->Find_circles(distance_x,distance_y);
        std::cout<<"1. "<<std::endl;
    } else if (input == 2){
        std::string address = "C:/Users/Silicio/WORK/MODULE_ON_CORE/medidas_fiduciales_CNM/Imagenes_fiduciales/mag_15X/Sensor_estandar/Todas/";
        std::string Images[] = {address + "Aruco_M/aruco_M_fiducial_chip_1_1_pos_1.TIF",
            address + "Atlas_E/atlasE_fiducial_chip_1_1_pos_1.TIF",
            address + "Atlas_F/Fiducial_chip_1_1_pos_1.TIF",
            "C:/Users/Silicio/WORK/Full_Size/W080/0004_f_fid_2.bmp",
            "C:/Users/Silicio/WORK/Full_Size/R0_W80_gantry_7.3/007_F.jpg",
            "C:/Users/Silicio/WORK/Full_Size/SCT_gantry_7.3/032_F.jpg",
            "C:/Users/Silicio/WORK/Full_Size/R0_W80_gantry_7.3/007_F_2.jpg",
            "C:/Users/Silicio/WORK/Full_Size/R0_W80_gantry_5.5/005_F_2.jpg",
            "C:/Users/Silicio/WORK/Full_Size/R0_W80_gantry_5.5/005_F_3.jpg"
                               };
        Ffinder->SetImageFiducial(Images[ui->spinBox_input_F->value()]
                ,CV_LOAD_IMAGE_COLOR);
        Ffinder->Find_F(ui->algorithm_box->value(),distance_x,distance_y,ui->spinBox_input->value());
    }
    qInfo("Displacement from expected position is: %5.2f um along X, %5.2f um along Y",distance_x,distance_y);
    std::ofstream ofs ("output.txt", std::ofstream::app);
    ofs << ui->spinBox_input->value()<<" "<< tmp_filename<<" "<<distance_x<<" "<<distance_x<<std::endl;
    ofs.close();
    delete Ffinder;
    mCamera->start();
    return;
}


//------------------------------------------
//calibrate

void Magrathea::Calibration_ButtonClicked()
{    calibrationCaller(0); }

void Magrathea::Calibration_2_ButtonClicked()
{    calibrationCaller(1); }

void Magrathea::calibrationCaller(int input){
    //Eventually add conmmand to move the gantry to the place where the
    //calibration area is.
    /////////////////////////////////////////
    //    cv::Point2f temp_coord;
    //    std::string temp_str = "dumb";
    //    f_locations->initialise();
    //    bool status = f_locations->get_value(1,temp_str,temp_coord);
    //    std::cout<<"st: "<<status<<" identifier: "<<temp_str<<" ; x: "<< temp_coord.x<<" y: "<<temp_coord.y<<std::endl;
    //    return;
    ////////////////////////////////////////
    cv::destroyAllWindows();
    mCamera->stop(); //closing QCamera
    Calibrator * calibrator = new Calibrator(this);

    bool from_file = ui->calib_from_file_Box->isChecked();
    cv::VideoCapture cap(ui->spinBox_dummy->value()); // open the video camera no. 0

    if (!cap.isOpened()){
        //Opening opencv-camera, needed for easier image manipulation
        QMessageBox::critical(this, tr("Error"), tr("Could not open camera"));
        return;}

    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    qInfo("Frame size : %6.0f x %6.0f",dWidth,dHeight);

    cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', 'U', 'Y', 'V'));
    //cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', '8', '0', '0'));
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 3856);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 2764);
    cap.set(CV_CAP_PROP_FPS, 4.0);

    dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    qInfo("Frame size : %6.0f x %6.0f",dWidth,dHeight);
    cv::Mat mat_from_camera;
    if(from_file){
        std::string Images[12] = {"C:/Temporary_files/BNL_images/image_000_600_60_15.png",
                                  "C:/Temporary_files/BNL_images/image_000_600_60_15_rotated_5_degrees.png",
                                  "C:/Temporary_files/BNL_images/image_000_600_60_15_rotated_30_degrees.png",
                                  "C:/Temporary_files/BNL_images/image_000_600_60_15_rotated_45_degrees.png",//4
                                  "C:/Temporary_files/BNL_images/image_000_600_60_15_rotated_stereo.png",
                                  "C:/Temporary_files/BNL_images/perfect_strips_0_degrees.png",
                                  "C:/Temporary_files/BNL_images/perfect_strips_1.49_degrees.png",
                                  "C:/Temporary_files/BNL_images/perfect_strips_5_degrees.png",
                                  "C:/Temporary_files/BNL_images/perfect_strips_45_degrees.png",//9
                                  "C:/Temporary_files/image_007_600_60_15_dan.png",
                                  "C:/Temporary_files/image_007_600_60_15_dan_rot_min20.png"
                                 };
        calibrator->SetImage(Images[2]
                ,CV_LOAD_IMAGE_COLOR);
    }else{
        bool bSuccess = cap.read(mat_from_camera);
        if (!bSuccess){ //if not success
            qInfo("Cannot read a frame from video stream");
            return;
        }
        calibrator->SetImage(mat_from_camera);
    }
    calibrator->Set_log(outputLogTextEdit);
    double calibration_value     = -100;
    double calibration_value_err = -10;
    bool is_px_over_micron = (input == 0);
    calibrator->Calibration_strips(calibration_value,calibration_value_err, is_px_over_micron);
    QString unit = (is_px_over_micron ? " px/um : Value set!" : " um/px : Value NOT set");
    QString output = "C: "+QString::number(calibration_value)+ " +- "+QString::number(calibration_value_err)+ unit;
    ui->Calib_value_lineEdit->setText(output);
    if(is_px_over_micron)
        mCalibration = calibration_value;
    delete calibrator;
    cap.release();         //Going back to QCameraa
    mCamera->start();
    return;
}


//******************************************
//gantry

//------------------------------------------
//connect gantry
//proxy function to handle the ConnectGantry and DisconnectGantry functions from MotionHandler
void Magrathea::connectGantryBoxClicked(bool checked)
{
    if (checked)
    {
        if (!mMotionHandler->connectGantry()) {
            ui->connectGantryBox->setChecked(false);
            qWarning("could not connect to gantry");
        } else {
            ui->connectGantryBox->setChecked(true);
            mMotionHandler->gantryConnected = true;
        }
    } else {
        if (    mMotionHandler->getXAxisState() ||
                mMotionHandler->getYAxisState() ||
                mMotionHandler->getZAxisState() ||
                mMotionHandler->getZ_2_AxisState() ||
                mMotionHandler->getUAxisState()) {
            ui->connectGantryBox->setChecked(true);
            qWarning("disable axes before disconnecting from gantry");
        } else {
            if(!mMotionHandler->disconnectGantry()) {
                ui->connectGantryBox->setChecked(true);
                qWarning("could not disconnect from gantry");
            }
        }
    }

    //enable/disable buttons according to gantry connection status
    ui->enableAxesButton->setEnabled(mMotionHandler->gantryConnected);
    ui->disableAxesButton->setEnabled(mMotionHandler->gantryConnected);
    ui->resetErrorButton->setEnabled(mMotionHandler->gantryConnected);

    return;
}

//------------------------------------------
//enable axes
//proxy function to handle the EnableAxes and DisableAxes functions from MotionHandler
void Magrathea::enableAxesClicked()
{
    if (sender() == ui->enableAxesButton)
        mMotionHandler->enableAxes(true);
    else if (sender() == ui->disableAxesButton)
        mMotionHandler->enableAxes(false);

    updatePosition();
    return;
}

//******************************************
//motion

//------------------------------------------
//enable joystick free run
//re-assign joystick buttons to free runnning
void Magrathea::enableJoystickFreeRun(bool checked)
{
    if (checked)
    {
        //disconnect signals from slots
        ui->positiveXButton->disconnect();
        ui->negativeXButton->disconnect();
        ui->positiveYButton->disconnect();
        ui->negativeYButton->disconnect();
        ui->positiveZButton->disconnect();
        ui->negativeZButton->disconnect();
        ui->positiveZ_2_Button->disconnect();
        ui->negativeZ_2_Button->disconnect();
        ui->positiveUButton->disconnect();
        ui->negativeUButton->disconnect();

        //connect signals to slots
        //NOTE free run requires parameters
        connect(ui->positiveXButton, SIGNAL(pressed()), this, SLOT(freeRun()));
        connect(ui->negativeXButton, SIGNAL(pressed()), this, SLOT(freeRun()));
        connect(ui->positiveYButton, SIGNAL(pressed()), this, SLOT(freeRun()));
        connect(ui->negativeYButton, SIGNAL(pressed()), this, SLOT(freeRun()));
        connect(ui->positiveZButton, SIGNAL(pressed()), this, SLOT(freeRun()));
        connect(ui->negativeZButton, SIGNAL(pressed()), this, SLOT(freeRun()));
        connect(ui->positiveZ_2_Button, SIGNAL(pressed()), this, SLOT(freeRun()));
        connect(ui->negativeZ_2_Button, SIGNAL(pressed()), this, SLOT(freeRun()));
        connect(ui->positiveUButton, SIGNAL(pressed()), this, SLOT(freeRun()));
        connect(ui->negativeUButton, SIGNAL(pressed()), this, SLOT(freeRun()));

        //NOTE end run does *not* require parameters
        connect(ui->positiveXButton, &QPushButton::released, mMotionHandler, &MotionHandler::endRunX);
        connect(ui->negativeXButton, &QPushButton::released, mMotionHandler, &MotionHandler::endRunX);
        connect(ui->positiveYButton, &QPushButton::released, mMotionHandler, &MotionHandler::endRunY);
        connect(ui->negativeYButton, &QPushButton::released, mMotionHandler, &MotionHandler::endRunY);
        connect(ui->positiveZButton, &QPushButton::released, mMotionHandler, &MotionHandler::endRunZ);
        connect(ui->negativeZButton, &QPushButton::released, mMotionHandler, &MotionHandler::endRunZ);
        connect(ui->positiveZ_2_Button, &QPushButton::released, mMotionHandler, &MotionHandler::endRunZ_2);
        connect(ui->negativeZ_2_Button, &QPushButton::released, mMotionHandler, &MotionHandler::endRunZ_2);
        connect(ui->positiveUButton, &QPushButton::released, mMotionHandler, &MotionHandler::endRunU);
        connect(ui->negativeUButton, &QPushButton::released, mMotionHandler, &MotionHandler::endRunU);
    }
}

//------------------------------------------
//free run proxy function to pass arguments to the MotionHandler
void Magrathea::freeRun()
{
    if (sender() == ui->positiveXButton)
        mMotionHandler->runX(+1, ui->xAxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->negativeXButton)
        mMotionHandler->runX(-1, ui->xAxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->positiveYButton)
        mMotionHandler->runY(+1, ui->yAxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->negativeYButton)
        mMotionHandler->runY(-1, ui->yAxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->positiveZButton)
        mMotionHandler->runZ(+1, ui->zAxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->negativeZButton)
        mMotionHandler->runZ(-1, ui->zAxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->positiveZ_2_Button)
        mMotionHandler->runZ_2(+1, ui->z_2_AxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->negativeZ_2_Button)
        mMotionHandler->runZ_2(-1, ui->z_2_AxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->positiveUButton)
        mMotionHandler->runU(+1, ui->uAxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->negativeUButton)
        mMotionHandler->runU(-1, ui->uAxisSpeedDoubleSpinBox->value());
    return;
}

//------------------------------------------
//enable joystick step motion
//re assign joystick buttons to step motion
void Magrathea::enableJoystickStepMotion(bool checked)
{
    if (checked)
    {
        //disconnect signals from slots
        ui->positiveXButton->disconnect();
        ui->negativeXButton->disconnect();
        ui->positiveYButton->disconnect();
        ui->negativeYButton->disconnect();
        ui->positiveZButton->disconnect();
        ui->negativeZButton->disconnect();
        ui->positiveZ_2_Button->disconnect();
        ui->negativeZ_2_Button->disconnect();
        ui->positiveUButton->disconnect();
        ui->negativeUButton->disconnect();

        //connect signals to slots
        //NOTE step motion requires parameters
        connect(ui->positiveXButton, SIGNAL(clicked(bool)), this, SLOT(stepMotion()));
        connect(ui->negativeXButton, SIGNAL(clicked(bool)), this, SLOT(stepMotion()));
        connect(ui->positiveYButton, SIGNAL(clicked(bool)), this, SLOT(stepMotion()));
        connect(ui->negativeYButton, SIGNAL(clicked(bool)), this, SLOT(stepMotion()));
        connect(ui->positiveZButton, SIGNAL(clicked(bool)), this, SLOT(stepMotion()));
        connect(ui->negativeZButton, SIGNAL(clicked(bool)), this, SLOT(stepMotion()));
        connect(ui->positiveZ_2_Button, SIGNAL(clicked(bool)), this, SLOT(stepMotion()));
        connect(ui->negativeZ_2_Button, SIGNAL(clicked(bool)), this, SLOT(stepMotion()));
        connect(ui->positiveUButton, SIGNAL(clicked(bool)), this, SLOT(stepMotion()));
        connect(ui->negativeUButton, SIGNAL(clicked(bool)), this, SLOT(stepMotion()));

        //set repeat delay
        ui->xAxisStepMoveButton->setAutoRepeatDelay(autoRepeatDelay);
        ui->positiveXButton->setAutoRepeatDelay(autoRepeatDelay);
        ui->negativeXButton->setAutoRepeatDelay(autoRepeatDelay);
        ui->yAxisStepMoveButton->setAutoRepeatDelay(autoRepeatDelay);
        ui->positiveYButton->setAutoRepeatDelay(autoRepeatDelay);
        ui->negativeYButton->setAutoRepeatDelay(autoRepeatDelay);
        ui->zAxisStepMoveButton->setAutoRepeatDelay(autoRepeatDelay);
        ui->positiveZButton->setAutoRepeatDelay(autoRepeatDelay);
        ui->negativeZButton->setAutoRepeatDelay(autoRepeatDelay);
        ui->z_2_AxisStepMoveButton->setAutoRepeatDelay(autoRepeatDelay);
        ui->positiveZ_2_Button->setAutoRepeatDelay(autoRepeatDelay);
        ui->negativeZ_2_Button->setAutoRepeatDelay(autoRepeatDelay);
        ui->uAxisStepMoveButton->setAutoRepeatDelay(autoRepeatDelay);
        ui->positiveUButton->setAutoRepeatDelay(autoRepeatDelay);
        ui->negativeUButton->setAutoRepeatDelay(autoRepeatDelay);

        //set repeat interval
        ui->xAxisStepMoveButton->setAutoRepeatInterval(autoRepeatInterval);
        ui->positiveXButton->setAutoRepeatInterval(autoRepeatInterval);
        ui->negativeXButton->setAutoRepeatInterval(autoRepeatInterval);
        ui->yAxisStepMoveButton->setAutoRepeatInterval(autoRepeatInterval);
        ui->positiveYButton->setAutoRepeatInterval(autoRepeatInterval);
        ui->negativeYButton->setAutoRepeatInterval(autoRepeatInterval);
        ui->zAxisStepMoveButton->setAutoRepeatInterval(autoRepeatInterval);
        ui->positiveZButton->setAutoRepeatInterval(autoRepeatInterval);
        ui->negativeZButton->setAutoRepeatInterval(autoRepeatInterval);
        ui->z_2_AxisStepMoveButton->setAutoRepeatInterval(autoRepeatInterval);
        ui->positiveZ_2_Button->setAutoRepeatInterval(autoRepeatInterval);
        ui->negativeZ_2_Button->setAutoRepeatInterval(autoRepeatInterval);
        ui->uAxisStepMoveButton->setAutoRepeatInterval(autoRepeatInterval);
        ui->positiveUButton->setAutoRepeatInterval(autoRepeatInterval);
        ui->negativeUButton->setAutoRepeatInterval(autoRepeatInterval);
    }
}

//------------------------------------------
//step motion proxy function to pass arguments to the MotionHandler
void Magrathea::stepMotion()
{
    //joystick
    if (sender() == ui->positiveXButton)
        mMotionHandler->moveXBy(+1*std::abs(ui->xAxisStepDoubleSpinBox->value()), ui->xAxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->negativeXButton)
        mMotionHandler->moveXBy(-1*std::abs(ui->xAxisStepDoubleSpinBox->value()), ui->xAxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->positiveYButton)
        mMotionHandler->moveYBy(+1*std::abs(ui->yAxisStepDoubleSpinBox->value()), ui->yAxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->negativeYButton)
        mMotionHandler->moveYBy(-1*std::abs(ui->yAxisStepDoubleSpinBox->value()), ui->yAxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->positiveZButton)
        mMotionHandler->moveZBy(+1*std::abs(ui->zAxisStepDoubleSpinBox->value()), ui->zAxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->negativeZButton)
        mMotionHandler->moveZBy(-1*std::abs(ui->zAxisStepDoubleSpinBox->value()), ui->zAxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->positiveZ_2_Button)
        mMotionHandler->moveZ_2_By(+1*std::abs(ui->z_2_AxisStepDoubleSpinBox->value()), ui->z_2_AxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->negativeZ_2_Button)
        mMotionHandler->moveZ_2_By(-1*std::abs(ui->z_2_AxisStepDoubleSpinBox->value()), ui->z_2_AxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->positiveUButton)
        mMotionHandler->moveUBy(+1*std::abs(ui->uAxisStepDoubleSpinBox->value()), ui->uAxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->negativeUButton)
        mMotionHandler->moveUBy(-1*std::abs(ui->uAxisStepDoubleSpinBox->value()), ui->uAxisSpeedDoubleSpinBox->value());

    //naviagtion panel
    else if  (sender() == ui->xAxisStepMoveButton)
        mMotionHandler->moveXBy(ui->xAxisStepDoubleSpinBox->value(), ui->xAxisSpeedDoubleSpinBox->value());
    else if  (sender() == ui->yAxisStepMoveButton)
        mMotionHandler->moveYBy(ui->yAxisStepDoubleSpinBox->value(), ui->yAxisSpeedDoubleSpinBox->value());
    else if  (sender() == ui->zAxisStepMoveButton)
        mMotionHandler->moveZBy(ui->zAxisStepDoubleSpinBox->value(), ui->zAxisSpeedDoubleSpinBox->value());
    else if  (sender() == ui->z_2_AxisStepMoveButton)
        mMotionHandler->moveZBy(ui->z_2_AxisStepDoubleSpinBox->value(), ui->z_2_AxisSpeedDoubleSpinBox->value());
    else if  (sender() == ui->uAxisStepMoveButton)
        mMotionHandler->moveUBy(ui->uAxisStepDoubleSpinBox->value(), ui->uAxisSpeedDoubleSpinBox->value());
    return;
}

//------------------------------------------
//position move proxy function to pass arguments to the MotionHandler
void Magrathea::positionMove()
{
    if (sender() == ui->xAxisPositionMoveButton)
        mMotionHandler->moveXTo(ui->xAxisPositionMoveDoubleSpinBox->value(), ui->xAxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->yAxisPositionMoveButton)
        mMotionHandler->moveYTo(ui->yAxisPositionMoveDoubleSpinBox->value(), ui->yAxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->zAxisPositionMoveButton)
        mMotionHandler->moveZTo(ui->zAxisPositionMoveDoubleSpinBox->value(), ui->zAxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->z_2_AxisPositionMoveButton)
        mMotionHandler->moveZ_2_To(ui->z_2_AxisPositionMoveDoubleSpinBox->value(), ui->z_2_AxisSpeedDoubleSpinBox->value());
    else if (sender() == ui->uAxisPositionMoveButton)
        mMotionHandler->moveUTo(ui->uAxisPositionMoveDoubleSpinBox->value(), ui->uAxisSpeedDoubleSpinBox->value());
    return;
}

//------------------------------------------
//step motion autorepeat
void Magrathea::axisStepRepeatBoxClicked(bool checked)
{
    if (sender() == ui->xAxisStepRepeatBox) {
        ui->xAxisStepMoveButton->setAutoRepeat(checked);
        ui->positiveXButton->setAutoRepeat(checked);
        ui->negativeXButton->setAutoRepeat(checked);
    }
    else if (sender() == ui->yAxisStepRepeatBox) {
        ui->yAxisStepMoveButton->setAutoRepeat(checked);
        ui->positiveYButton->setAutoRepeat(checked);
        ui->negativeYButton->setAutoRepeat(checked);
    }
    else if (sender() == ui->zAxisStepRepeatBox) {
        ui->zAxisStepMoveButton->setAutoRepeat(checked);
        ui->positiveZButton->setAutoRepeat(checked);
        ui->negativeZButton->setAutoRepeat(checked);
    }
    else if (sender() == ui->z_2_AxisStepRepeatBox) {
        ui->z_2_AxisStepMoveButton->setAutoRepeat(checked);
        ui->positiveZ_2_Button->setAutoRepeat(checked);
        ui->negativeZ_2_Button->setAutoRepeat(checked);
    }
    else if (sender() == ui->uAxisStepRepeatBox) {
        ui->uAxisStepMoveButton->setAutoRepeat(checked);
        ui->positiveUButton->setAutoRepeat(checked);
        ui->negativeUButton->setAutoRepeat(checked);
    }
    return;
}

void Magrathea::AxisEnableDisableButton(){
    bool current = false;
    if(sender() == ui->EnableButton_X){
        current = mMotionHandler->getXAxisState();
        mMotionHandler->enableXAxis(!current);
        ui->EnableButton_X->setText((current ? "Enable" : "Disable"));
    }else if (sender() == ui->EnableButton_Y){
        current = mMotionHandler->getYAxisState();
        mMotionHandler->enableYAxis(!current);
        ui->EnableButton_Y->setText((current ? "Enable" : "Disable"));
    }else if (sender() == ui->EnableButton_Z){
        current = mMotionHandler->getZAxisState();
        mMotionHandler->enableZAxis(!current);
        ui->EnableButton_Z->setText((current ? "Enable" : "Disable"));
    }else if (sender() == ui->EnableButton_Z_2){
        current = mMotionHandler->getZ_2_AxisState();
        mMotionHandler->enableZ_2_Axis(!current);
        ui->EnableButton_Z_2->setText((current ? "Enable" : "Disable"));
    }else if (sender() == ui->EnableButton_U){
        current = mMotionHandler->getUAxisState();
        mMotionHandler->enableUAxis(!current);
        ui->EnableButton_U->setText((current ? "Enable" : "Disable"));
    }else
        qWarning("Warning! Improper use of function AxisEnableDisableButton.");
}


void Magrathea::led_label(QLabel *label, bool value){
    if(value){
        label->setStyleSheet("QLabel { background-color : green; color : black; }");
        label->setText("ON");
    }else{
        label->setStyleSheet("QLabel { background-color : red; color : white; }");
        label->setText("OFF");
    }
}

void Magrathea::color_test(){
    std::cout<<"here"<<std::endl;
//    std::string Images[15] = {"C:/Users/Silicio/WORK/Full_Size/R0_W80_gantry_7.3/color/001.jpg",//0
//                              "C:/Users/Silicio/WORK/Full_Size/R0_W80_gantry_7.3/color/002.jpg",
//                              "C:/Users/Silicio/WORK/Full_Size/R0_W80_gantry_7.3/color/004.jpg",
//                              "C:/Users/Silicio/WORK/Full_Size/R0_W80_gantry_7.3/color/005.jpg",
//                              "C:/Users/Silicio/WORK/Full_Size/R0_W80_gantry_7.3/color/006.jpg",
//                              "C:/Users/Silicio/WORK/Full_Size/R0_W80_gantry_7.3/color/007.jpg"//5
//                             };
//    cv::Mat input = cv::imread(Images[ui->spinBox_input->value()],CV_LOAD_IMAGE_COLOR);
    //cv::imshow("input",input);

    cv::destroyAllWindows();
    mCamera->stop(); //closing QCamera
    cv::VideoCapture cap(ui->spinBox_dummy->value()); // open the video camera no. 0

    if (!cap.isOpened()){
        //Opening opencv-camera, needed for easier image manipulation
        QMessageBox::critical(this, tr("Error"), tr("Could not open camera"));
        return;}

    cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', 'U', 'Y', 'V'));
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 3856);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 2764);
    cap.set(CV_CAP_PROP_FPS, 4.0);
    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
    qInfo("Frame size : %6.0f x %6.0f",dWidth,dHeight);

    cv::Mat input;
    bool bSuccess = cap.read(input);
    if (!bSuccess){ //if not success
        qInfo("Cannot read a frame from video stream");
        return;
    }

    cv::Mat bgr[3];   //destination array
    cv::split(input,bgr);//split source

    //Note: OpenCV uses BGR color order
    cv::imshow("blue",bgr[0]);  //blue channel
    cv::imshow("green",bgr[1]); //green channel
    cv::imshow("red",bgr[2]);   //red channel


    cv::imwrite("blue.png",bgr[0]); //blue channel
    cv::imwrite("green.png",bgr[1]); //green channel
    cv::imwrite("red.png",bgr[2]); //red channel
}

void Magrathea::destroy_all(){
    cv::destroyAllWindows();
}

void Magrathea::loop_test(){

    for(int i=0;i<440;i++){
        Sleeper::msleep(500);
        std::cout<<"It "<<i<<std::endl;
        ui->spinBox_input->setValue(i);
        FiducialFinderCaller(2);
    }
}








