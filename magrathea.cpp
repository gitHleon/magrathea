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

std::string type2str(int type);

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
    connect(ui->captureButton,   SIGNAL(clicked(bool)), this, SLOT(captureButtonClicked()));
    connect(ui->Calib_button,    SIGNAL(clicked(bool)), this, SLOT(Calibration_ButtonClicked()));
    connect(ui->pushButton_dummy,SIGNAL(clicked(bool)), this, SLOT(Camera_test()));
    connect(ui->focusButton     ,SIGNAL(clicked(bool)), this, SLOT(focusButtonClicked()));
    connect(ui->std_dev_button  ,SIGNAL(clicked(bool)), this, SLOT(focusButtonClicked()));
    connect(ui->std_dev_many_button,SIGNAL(clicked(bool)), this, SLOT(focusButtonClicked()));
    connect(ui->Fiducial_finder_button,SIGNAL(clicked(bool)), this, SLOT(Fiducial_finder_button_Clicked()));
    connect(ui->Circles_button,SIGNAL(clicked(bool)), this, SLOT(Circles_button_Clicked()));
    connect(ui->VignetteButton,SIGNAL(clicked(bool)),this,SLOT(VignetteButton_clicked()));
    connect(ui->ArucoButton,SIGNAL(clicked(bool)),this,SLOT(Aruco_test()));
    connect(ui->F_fid_gen_button,SIGNAL(clicked(bool)),this,SLOT(createTemplate_F()));
    //connect(ui->cap_and_move_button,SIGNAL(clicked(bool)),this,SLOT(capture_fid_and_move()));
    connect(ui->focusalgotest_pushButton,SIGNAL(clicked(bool)),this,SLOT(FocusAlgoTest_Func()));

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
    connect(ui->f_loop_button,SIGNAL(clicked(bool)), this, SLOT(loop_fid_finder()));
    connect(ui->DelLogButton,SIGNAL(clicked(bool)),outputLogTextEdit,SLOT(clear()));
    connect(ui->Run_calib_plate_button,SIGNAL(clicked(bool)),this,SLOT(calibration_plate_measure()));
    connect(ui->FitTestButton,SIGNAL(clicked(bool)),this,SLOT(FitTestButtonClick()));
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
    std::vector <double> pos_t = mMotionHandler->whereAmI(1);

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
    //reading fault state for each axis

    return;

    unsigned int mask1 = ((1 << 1) - 1 ) << 5;//Mask for Software Right Limit
    unsigned int mask2 = ((1 << 1) - 1 ) << 6;//Mask for Software Left  Limit
    //GETMASK(index, size) (((1 << (size)) - 1) << (index))
    //READFROM(data, index, size) (((data) & GETMASK((index), (size))) >> (index))
    unsigned int fault_state = mMotionHandler->GetfaultSateXAxis();
    unsigned int temp1 = (fault_state & mask1) >> 5;//Software Right Limit
    unsigned int temp2 = (fault_state & mask2) >> 6;//Software Left  Limit
    bool axis_fault = (temp1 || temp2 );
    ui->label_9->setText((axis_fault ? "Out of Env" : "Good"));
    if(axis_fault)
        ui->label_9->setStyleSheet("QLabel { background-color : red; color : black; }");
    else
        ui->label_9->setStyleSheet("QLabel { background-color : green; color : white; }");

    fault_state = mMotionHandler->GetfaultSateYAxis();
    temp1 = (fault_state & mask1) >> 5;//Software Right Limit
    temp2 = (fault_state & mask2) >> 6;//Software Left  Limit
     axis_fault = (temp1 == 1 || temp2 == 1);
    ui->label_11->setText((axis_fault ? "Out of Env" : "Good"));
    if(axis_fault)
        ui->label_11->setStyleSheet("QLabel { background-color : red; color : black; }");
    else
        ui->label_11->setStyleSheet("QLabel { background-color : green; color : white; }");

    fault_state = mMotionHandler->GetfaultSateZ1Axis();
    temp1 = (fault_state & mask1) >> 5;//Software Right Limit
    temp2 = (fault_state & mask2) >> 6;//Software Left  Limit
     axis_fault = (temp1 == 1 || temp2 == 1);
    ui->label_13->setText((axis_fault ? "Out of Env" : "Good"));
    if(axis_fault)
        ui->label_13->setStyleSheet("QLabel { background-color : red; color : black; }");
    else
        ui->label_13->setStyleSheet("QLabel { background-color : green; color : white; }");

    fault_state = mMotionHandler->GetfaultSateZ2Axis();
    temp1 = (fault_state & mask1) >> 5;//Software Right Limit
    temp2 = (fault_state & mask2) >> 6;//Software Left  Limit
     axis_fault = (temp1 == 1 || temp2 == 1);
    ui->label_15->setText((axis_fault ? "Out of Env" : "Good"));
    if(axis_fault)
        ui->label_15->setStyleSheet("QLabel { background-color : red; color : black; }");
    else
        ui->label_15->setStyleSheet("QLabel { background-color : green; color : white; }");

    ui->label_17->setText("Always good");
    ui->label_17->setStyleSheet("QLabel { background-color : yellow; color : black; }");

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

void Magrathea::FocusAlgoTest_Func(){
  Focus_finder * FocusFinder = new Focus_finder(this);
  cv::Mat mat_mat;
  std::string file_name = "";
  std::string address = "C:/Users/Silicio/cernbox/Gantry_2018/Focus_test_mitutoyo/";
  std::string Images[] ={

          "0.1_pos",
          "0.09_pos",
          "0.08_pos",
          "0.07_pos",
          "0.06_pos",
          "0.05_pos",
          "0.04_pos",
          "0.03_pos",
          "0.02_pos",
          "0.01_pos",
          "Focused",
          "0.01_neg",
          "0.02_neg",
          "0.03_neg",
          "0.04_neg",
          "0.05_neg",
          "0.06_neg",
          "0.07_neg",
          "0.08_neg",
          "0.09_neg",
          "0.1_neg"
  };
  std::vector<double> std_dev_value;
  for(int i=0;i<21;i++){
    file_name = address + Images[i] + ".tif";
    mat_mat = cv::imread( file_name, CV_LOAD_IMAGE_COLOR);
    const int kernel_size = ( (mat_mat.rows > 1000 && mat_mat.cols > 1000) ? 11 : 5);
    FocusFinder->Set_ksize(kernel_size);
    FocusFinder->Set_color_int(ui->ColorBox->value());
    cv::Rect region(0,0,mat_mat.cols,mat_mat.rows-30);
    cv::Mat RoI = mat_mat(region);
    std::vector<double> figures_of_merit;
    FocusFinder->eval_stddev(RoI,figures_of_merit);
    if(figures_of_merit.size() != 0){
        qInfo("%s  %5.1f  %5.5f  %5.1f  %5.1f",Images[i].c_str(),figures_of_merit[0],figures_of_merit[1],figures_of_merit[2],figures_of_merit[3]);
        std_dev_value.push_back(figures_of_merit[0]);
        std::string file_name = "focus_m.txt";
        std::ofstream ofs (file_name, std::ofstream::app);
        ofs << i <<" "<<Images[i]<<" "<<  figures_of_merit[0]<<" "<<figures_of_merit[1]<<" "<<figures_of_merit[2]<<" "<<figures_of_merit[3]<<std::endl;
        ofs.close();

    }
    ///draw histogram
    /// Establish the number of bins
    cv::Mat mat_mat_g = cv::imread( file_name, CV_LOAD_IMAGE_GRAYSCALE);
    int histSize = 256;

    /// Set the ranges ( for B,G,R) )
    float range[] = { 0, 256 } ;
    const float* histRange = { range };

    bool uniform = true; bool accumulate = false;

    cv::Mat g_hist;

    /// Compute the histograms:
    calcHist( &mat_mat_g, 1, nullptr, cv::Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
    // Draw the histograms for B, G and R
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );
    cv::Mat histImage( hist_h, hist_w, CV_8UC1, cv::Scalar(0) );
    normalize(g_hist, g_hist, 0, histImage.rows, CV_MINMAX, -1, cv::Mat() );

    /// Draw for each channel
    for( int i = 1; i < histSize; i++ )
    {
        line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
              cv::Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
              cv::Scalar(255), 2, 8, 0  );
    }

    /// Display
    cv::namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
    cv::imshow("calcHist Demo", histImage );
    //cv::imwrite("EXPORT/"+Images[i]+"_hist.jpg",histImage);
  }

  double numerator   = 0.0;
  double denominator = 0.0;
  for(unsigned int j=0;j< std_dev_value.size();j++){
      numerator   += j*std_dev_value[j];
      denominator += std_dev_value[j];
  }
  qInfo("%5.1f",numerator/denominator);
  delete FocusFinder;
}

bool Magrathea::focusButtonClicked()
{
    qInfo(" > camera focus ... ");
    QElapsedTimer timer;
    timer.start();
    Focus_finder * FocusFinder = new Focus_finder(this);
    mCamera->stop(); //closing QCamera

    cv::VideoCapture cap(ui->spinBox_dummy->value()); // open the video camera no. 0
    if (!cap.isOpened()){
        //    if(!cap.open(0)){     //Opening opencv-camera, needed for easier image manipulation
        QMessageBox::critical(this, tr("Error"), tr("Could not open camera"));
        return false;}
    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    //veryfing that the setting of the camera is optimal
    cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', 'U', 'Y', 'V'));
    //cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', '8', '0', '0'));
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 3856);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 2764);
    cap.set(CV_CAP_PROP_FPS, 4.0);
    cap.set(CV_CAP_PROP_GAIN, 4.0);
    dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
    qInfo("Frame size : %6.0f x %6.0f",dWidth,dHeight);

    FocusFinder->Set_camera(cap);
    FocusFinder->Set_gantry(mMotionHandler);
    FocusFinder->Set_log(outputLogTextEdit);
    FocusFinder->Set_color_int(ui->ColorBox->value());
    const int kernel_size = ( (dWidth > 2000 && dHeight > 2000) ? 11 : 5);
    FocusFinder->Set_ksize(kernel_size);
    double focus_position = -1.;
    if (sender() == ui->std_dev_button){
        cv::Mat mat_from_camera;
        bool bSuccess = cap.read(mat_from_camera);
        if (!bSuccess){ //if not success
            qInfo("Cannot read a frame from video stream");
            return false;
        }
        std::vector<double> figures_of_merit;
        FocusFinder->eval_stddev_ROI(mat_from_camera,figures_of_merit);
        if(figures_of_merit.size() != 0)
            qInfo(" Lap : %5.5f;  StdDev : %5.5f;  1st der : %5.5f;  canny edge : %5.5f; ",figures_of_merit[0],figures_of_merit[1],figures_of_merit[2],figures_of_merit[3]);
    } else if(sender() == ui->std_dev_many_button){
        FocusFinder->Eval_syst_scan();
    } else {
        FocusFinder->find_focus(focus_position);
    }
    qInfo(" > camera focus : %3.5f",focus_position);
    delete FocusFinder;
    std::cout<<">>> The FOCUS operation took "<< timer.elapsed() <<" milliseconds"<<std::endl;
    cap.release();

    //Going back to QCameraa
    //mCamera->start();

    return true;
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
    qInfo("cap.get(CV_CAP_PROP_GAMMA);         : %5.5f",cap.get(CV_CAP_PROP_GAMMA));
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
    //this function is to evaluate perpendicularity of the camera wrt to the table
    //It is still in prototype fase and it has not been tested yet.
    //Use with caution!!
    //Written from BNL function described here: https://github.com/sciollalab/BNL_ThermomechanicalStave/wiki/Calibration-Methods
    //"Checking optical axis alignment"
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

void Magrathea::Fiducial_finder_button_Clicked()
{   std::vector <double> dummy;
    FiducialFinderCaller(0,dummy); }

void Magrathea::Circles_button_Clicked()
{   std::vector <double> dummy;
    FiducialFinderCaller(2,dummy); }

bool Magrathea::FiducialFinderCaller(const int &input, std::vector <double> & F_point){

    bool debug = false;
    cv::destroyAllWindows();
    mCamera->stop(); //closing QCamera

    //opening camera with opencv
    cv::Mat mat_from_camera;
    cv::VideoCapture cap(ui->spinBox_dummy->value()); // open the video camera no. 0
    if (!cap.isOpened()){
        //Opening opencv-camera, needed for easier image manipulation
        QMessageBox::critical(this, tr("Error"), tr("Could not open camera"));
        return false;}
    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    if(debug)
        qInfo("Frame size : %6.0f x %6.0f",dWidth,dHeight);
    cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', 'U', 'Y', 'V'));
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 3856);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 2764);
    cap.set(CV_CAP_PROP_FPS, 4.0);
    dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
    if(debug)
        qInfo("Frame size : %6.0f x %6.0f",dWidth,dHeight);

    FiducialFinder * Ffinder = new FiducialFinder(this);

    bool from_file = ui->calib_from_file_Box->isChecked();
    Ffinder->Set_log(outputLogTextEdit);

    double distance_x = 0;
    double distance_y = 0;
    std::string timestamp = "";
    std::string address = "D:/Images/Templates_mytutoyo/";
    //std::string address = "C:/Users/Silicio/WORK/MODULE_ON_CORE/medidas_fiduciales_CNM/Imagenes_fiduciales/mag_15X/Sensor_estandar/Todas/templates/";
    std::string Images[] = {
        address + "atlasE_fiducial_chip_1_1_pos_1.TIF",  //0
        address + "Fiducial_chip_1_1_pos_1.TIF",
        address + "atlas_g_chip_1_1_pos_1.TIF",
        address + "atlas_h_chip_1_1_pos_1.TIF",
        address + "aruco_f_chip_1_1_pos_1.TIF",
        address + "aruco_l_chip_1_1_pos_1.TIF", //5
        address + "aruco_f_chip_1_1_pos_1.TIF",
        address + "aruco_h_chip_1_1_pos_1.TIF",
        address + "aruco_o_chip_1_1_pos_2.TIF",
        address + "aruco_g_chip_1_1_pos_1.TIF",
        address + "aruco_M_chip_1_1_pos_1.TIF", //10
        address + "fiducialE.png",
        address + "fiducialF.png",
        address + "e_perfect_4_5.png",
        address + "f_perfect_4_5.png",
        address + "e_fid.jpg",                           //15
        address + "f_fid.jpg",
        address + "ar_m_fid.jpg",
        address + "fid_test_1.jpg",
        address + "fid_test_2.jpg",
        address + "fid_test_3.jpg", //20
        address + "placa_fid_73_F.jpg",
        address + "placa_fid_F.jpg",
        address + "placa_fid_test3_dotsandcrosses.jpg",
        address + "003_F.jpg",
        address + "003_F3.jpg"
    };

    Ffinder->SetImageFiducial(Images[ui->spinBox_input_F->value()]
            ,CV_LOAD_IMAGE_COLOR);

    std::string tmp_filename = "";
    bool success = false;

    if(from_file){
        //std::string address = "C:/Users/Silicio/cernbox/Gantry_2018/Camera_tests/sctcamera_20190111/";
        std::string address = "C:/Users/Silicio/cernbox/Gantry_2018/Camera_tests/Calibration_plate_measures/pos_5/";
        std::string Images[] = {
//            "003.jpg",
//            "004.jpg",
//            "005.jpg",
//            "006.jpg",
//            "007.jpg",
//            "008.jpg",
//            "009.jpg",
//            "010.jpg",
//            "011.jpg",
//            "012.jpg",
//            "013.jpg",
//            "014.jpg",
//            "015.jpg",
            "Circles_0_2_0.jpg",
            //"017.jpg"
            //"chip_1_1_pos_1.TIF"
        };

        tmp_filename = Images[ui->spinBox_input->value()];
        Ffinder->SetImage(address + Images[ui->spinBox_input->value()]
                ,CV_LOAD_IMAGE_COLOR);
        Ffinder->Set_calibration(mCalibration); //get calibration from a private variable
    }else{
        bool bSuccess = cap.read(mat_from_camera);
        if (!bSuccess){ //if not success
            qInfo("Cannot read a frame from video stream");
            return false;
        }
        Ffinder->Set_calibration(mCalibration); //get calibration from a private variable
        Ffinder->SetImage(mat_from_camera);
    }
    qInfo("Calibration value is : %5.3f [px/um]",mCalibration);

    if(input == 1 || input == 0){
        bool invalid_match = true;
        //int ii = 0;
        //here you can apply condition on the found match to evaluate if it is good or bad
        //while(invalid_match){
            cv::Mat output_H;
            success = Ffinder->Find_F(ui->algorithm_box->value(),distance_x,distance_y,ui->spinBox_input->value(),
                                      ui->chip_number_spinBox->value(),timestamp,ui->filter_spinBox->value()/*dummy_temp*/,output_H);
            double H_1_1 = cv::Scalar(output_H.at<double>(0,0)).val[0];
            double H_1_2 = cv::Scalar(output_H.at<double>(0,1)).val[0];
//            if( ( fabs(H_1_1/H_1_2) < 0.26 ) && (sqrt(H_1_1*H_1_1 + H_1_2*H_1_2) < 1.05 && sqrt(H_1_1*H_1_1 + H_1_2*H_1_2) > 0.95) )
//                invalid_match = false;
            //ii++;
            //if(ii> 1)
            //    invalid_match = false;
            std::cout<<" invalid_match "<<invalid_match <<" ;tan(theta) "<< fabs(H_1_1/H_1_2)<<" ;s "<< sqrt(H_1_1*H_1_1 + H_1_2*H_1_2)<<std::endl;
        //}//while invalid match
    } else if(input == 2){
        success = Ffinder->Find_circles(distance_x,distance_y,ui->spinBox_input->value(),ui->chip_number_spinBox->value(),true);
        QTime now = QTime::currentTime();
        QString time_now = now.toString("hhmmss");
        timestamp = time_now.toLocal8Bit().constData();
    }

    if(success)
        qInfo("Displacement from expected position is: %5.2f um along X, %5.2f um along Y",distance_x,distance_y);
    else {
        qInfo("Fiducial fail");
        return false;
    }
    std::vector <double> pos_t = mMotionHandler->whereAmI(1); //get gantry position
    std::string file_name = ((input==0) ? "output_temp.txt" : "output_good.txt");
    std::ofstream ofs (file_name, std::ofstream::app);
    ///////////////////////////////////////////////////////////////////
    //WARNING!!! x,y of camera may be different of x,y of gantry!!!  //
    ///////////////////////////////////////////////////////////////////
    ofs << timestamp<<" "<<ui->spinBox_input->value()<<" "<<ui->chip_number_spinBox->value()<<" "<<tmp_filename;//<<" "<<
    delete Ffinder;
    cap.release();         //Going back to QCameraa
    //mCamera->start();
    ///////////////////////////////////////////////////////////////////
    //WARNING!!! x,y of camera may be different of x,y of gantry!!!  //
    ///////////////////////////////////////////////////////////////////

#if VALENCIA
    double camera_angle = 0.886;
    double target_x_short = distance_x*0.001*cos(camera_angle) + distance_y*0.001*sin(camera_angle);
    double target_y_short = distance_x*0.001*sin(camera_angle) - distance_y*0.001*cos(camera_angle);
    ofs<<" "<<pos_t[0]-target_x_short<<" "<<pos_t[1]-target_y_short<<" "<<pos_t[4]<<std::endl;
    ofs.close();
#else
    ofs << std::endl;
    mMotionHandler->moveXBy(distance_x*0.001,1);
    mMotionHandler->moveYBy(distance_y*0.001,1);
#endif
    F_point.clear();
    F_point.push_back(distance_x*0.001);
    F_point.push_back(distance_y*0.001);

    return true;
    }

//------------------------------------------
//calibrate

void Magrathea::Calibration_ButtonClicked()
{    calibrationCaller(0); }

void Magrathea::calibrationCaller(int input){
    //Eventually add command to move the gantry to the place where the
    //calibration area is.
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
    cap.set(CV_CAP_PROP_GAIN, 363.0);
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
    calibrator->Calibration_strips(calibration_value,calibration_value_err, true);
    QString unit = " px/um";
    QString output = "C: "+QString::number(calibration_value,'g',3)+ " +- "+QString::number(calibration_value_err,'g',3)+ unit;
    std::cout<<"calib = "<<calibration_value<<" err : "<<calibration_value_err<<std::endl;
    ui->Calib_value_lineEdit->setText(output);
    mCalibration = calibration_value;
    delete calibrator;
    cap.release();         //Going back to QCamera
    //mCamera->start();
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
        mMotionHandler->moveZ_2_By(ui->z_2_AxisStepDoubleSpinBox->value(), ui->z_2_AxisSpeedDoubleSpinBox->value());
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

bool Magrathea::loop_test(){
    //mMotionHandler->SetLimitsController();
    //run fiducial finding algo automatically
    //and move to the fiducial position
    for(int j=0;j<70;j++){//set appropriate value of the loop limit
        ui->chip_number_spinBox->setValue(j);
        if(!mMotionHandler->moveXBy(0.070,1.))//dispalacement added for systematic testing of the algorithm
            return false;
        if(!mMotionHandler->moveYBy(0.070,1.))
            return false;
        for(int i=0;i<5;i++){//set appropriate value of the loop limit
            std::cout<<"j "<<j<<" ; i "<<i<<std::endl;
            ui->spinBox_input->setValue(i);
            std::vector <double> distances;
            //if(!focusButtonClicked())
            //    return;
            if(!FiducialFinderCaller(2,distances))
            {
                std::cout<<"FAIL!!"<<std::endl;
                return false;
            }
            double camera_angle = 0.886;
            double target_x_short = distances[0]*cos(camera_angle) + distances[1]*sin(camera_angle);
            double target_y_short = distances[0]*sin(camera_angle) - distances[1]*cos(camera_angle);
            //ATTENTION! distances[0] is cols, distances[1] is rows of the image
            //std::vector <double> pos_t_1 = mMotionHandler->whereAmI(1);
            if(!mMotionHandler->moveXBy(-target_x_short,1.))
                return false;
            if(!mMotionHandler->moveYBy(-target_y_short,1.))
                return false;
        }
    }
    return true;
}

bool Magrathea::loop_fid_finder(){
    //mMotionHandler->SetLimitsController();
    //run fiducial finding algo automatically
    //and move to the fiducial position
    for(int i=0;i<3;i++){//set appropriate value of the loop limit
        std::cout<<"It "<<i<<std::endl;
        std::vector <double> distances;
        if(!FiducialFinderCaller(2,distances))
        {
            std::cout<<"FAIL!!"<<std::endl;
            return false;
        }
        double camera_angle = 0.886;
        double target_x_short = distances[0]*cos(camera_angle) + distances[1]*sin(camera_angle);
        double target_y_short = distances[0]*sin(camera_angle) - distances[1]*cos(camera_angle);
        //ATTENTION! distances[0] is cols, distances[1] is rows of the image
        if(!mMotionHandler->moveXBy(-target_x_short,1.))
            return false;
        if(!mMotionHandler->moveYBy(-target_y_short,1.))
            return false;
    }
    return true;
}

void Magrathea::Aruco_test(){
    //visualize the different aruco markers
    cv::Mat test_aruco;
    auto dictionary = cv::aruco::generateCustomDictionary(522,3);
    cv::aruco::drawMarker(dictionary, ui->ArucospinBox->value() , 200, test_aruco, 1);
    cv::imshow("aruco",test_aruco);
    ui->ArucospinBox->setValue(ui->ArucospinBox->value()+1);
}

void Magrathea::createTemplate_F(){
    //function to create imges of the ATLAS F and ATLAS E fiducials
    //Need to run only once to create the images. The user save them and put in the appropriate folder for the code to reach them.
    //the factor can be adjusted to have more r less pixels in the image.
    double factor = mCalibration;
    std::cout<<"factor : "<<factor<<std::endl;
    //F fiducial
    cv::Mat fiducial = cv::Mat(cv::Size(200*factor,220*factor),CV_8UC1,cv::Scalar(255));
    cv::Mat ARoi = fiducial(cv::Rect(50*factor, 50*factor, 100*factor, 120*factor));
    ARoi.setTo(cv::Scalar(0));
    cv::Mat BRoi = fiducial(cv::Rect(70*factor, 70*factor, 80*factor, 28*factor));
    BRoi.setTo(cv::Scalar(255));
    cv::Mat CRoi = fiducial(cv::Rect(130*factor, 98*factor, 20*factor, 20*factor));
    CRoi.setTo(cv::Scalar(255));
    cv::Mat DRoi = fiducial(cv::Rect(70*factor, 118*factor, 80*factor, 52*factor));
    DRoi.setTo(cv::Scalar(255));
    cv::imshow("fiducial F",fiducial);

    //E fiducial
    cv::Mat fiducial_2 = cv::Mat(cv::Size(155*factor,185*factor),CV_8UC1,cv::Scalar(255));
    cv::Mat aRoi = fiducial_2(cv::Rect(50*factor, 50*factor, 25*factor, 25*factor));
    aRoi.setTo(cv::Scalar(0));
    cv::Mat bRoi = fiducial_2(cv::Rect(80*factor, 50*factor, 25*factor, 25*factor));
    bRoi.setTo(cv::Scalar(0));
    cv::Mat cRoi = fiducial_2(cv::Rect(50*factor, 110*factor, 25*factor, 25*factor));
    cRoi.setTo(cv::Scalar(0));
    cv::Mat dRoi = fiducial_2(cv::Rect(80*factor, 110*factor, 25*factor, 25*factor));
    dRoi.setTo(cv::Scalar(0));
    cv::circle(fiducial_2, cv::Point(77*factor,92*factor), 12.5*factor, cv::Scalar(0), -1);
    cv::imshow("fiducial E",fiducial_2);
}

bool Magrathea::calibration_plate_measure(){
    //function to calibrate the gantry
    //it is meant to be used with a calibration plate
    //for now to be used only in valencia, it can be taken as starting point for others that would
    //like to perform a similar measure
    //Fpr questions : dmadaffa@cern.ch
    cv::destroyAllWindows();

    std::vector< std::vector<double> > points;
    std::vector<double> temp_v;

    temp_v.push_back(ui->point1_x_box->value());//gantry coord of point 1
    temp_v.push_back(ui->point1_y_box->value());
    points.push_back(temp_v);
    temp_v.clear();

    temp_v.push_back(ui->point2_x_box->value());//gantry coord of point 2
    temp_v.push_back(ui->point2_y_box->value());
    points.push_back(temp_v);
    temp_v.clear();

    double angle = atan((points[1][1]-points[0][1])/(points[1][0]-points[0][0]));
    //these are measured points from which I get the angle of the calibration plate
    //These are also used as starting point (i.e. origin of the frame of reference) for the calibration plate measure

    double step_x = 15.;
    double step_y = 12.;
    double speed = 3.;
    for(int i=0;i<10;i++){//y
        ui->chip_number_spinBox->setValue(i);
        for(int j=0;j<10;j++){//x
            speed = (i!=0 && j==0) ? 6. : 3.;
            ui->spinBox_input->setValue(j);
            double target_x = points[0][0] + step_x*j*cos(angle) - step_y*i*sin(angle);
            double target_y = points[0][1] + step_x*j*sin(angle) + step_y*i*cos(angle);
            std::cout<<j<<" "<<i<<" target_x "<<target_x<<" target_y "<<target_y<<std::endl;
            if(!mMotionHandler->moveXTo(target_x,speed))
                return false;
            if(!mMotionHandler->moveYTo(target_y,3.))
                return false;
            if(!focusButtonClicked())
                return false;
            if(!loop_fid_finder())
                return false;

            auto one = std::to_string(ui->spinBox_plate_position->value());
            QTime now = QTime::currentTime();
            QString time_now = now.toString("hhmmss");
            std::string timestamp = time_now.toLocal8Bit().constData();

            std::vector <double> pos_t_1 = mMotionHandler->whereAmI(1);
            std::string file_name = "Calibration_plate_position_"+one+".txt";
            std::ofstream ofs (file_name, std::ofstream::app);
            ofs<<timestamp<<" "<<j<<" "<<i<<" "<<pos_t_1[0]<<" "<<pos_t_1[1]<<" "<<pos_t_1[4]<<std::endl;
            ofs.close();

            std::vector <double> pos_t_2 = mMotionHandler->whereAmI(0);
            std::string file_name_2 = "Calibration_plate_position_"+one+"_other_var.txt";
            std::ofstream ofs_2 (file_name_2, std::ofstream::app);
            ofs_2<<timestamp<<" "<<j<<" "<<i<<" "<<pos_t_2[0]<<" "<<pos_t_2[1]<<" "<<pos_t_2[4]<<std::endl;
            ofs_2.close();
        }
    }
    return true;
}

int Magrathea::FitTestButtonClick(){

    FiducialFinder * Ffinder = new FiducialFinder(this);
    Ffinder->Set_log(outputLogTextEdit);
    Ffinder->dumb_test();
    return 0;
}


