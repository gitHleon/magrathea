#include <iomanip>
#include <sstream>
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
#include <QMouseEvent>
#include <MotionHandler.h>
#include <cmath>
#include <QMessageBox>
#include "calibrator.h"
#include "focus_finder.h"
#include "Fiducial_finder.h"
#include "verticalalignmenttool.h"
//#include <conio.h>
#include <fstream>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#ifdef VANCOUVER
#include <AerotechMotionhandler.h>
#elif VALENCIA
#include <ACSCMotionHandler.h>
#include <QJoysticks.h>
#endif
#include "CameraView.h"
#include <logger.h>
#include <QPetalLocator.h>

static LoggerStream os;

//function for debug in opencv, defined elsewhere
std::string type2str(int type);

//function to write in appropriate way exadecimal number to ultimusV
QByteArray int_tohexQByteArray_UltimusV(int input){
    auto && oss = std::ostringstream();
    oss << std::hex << std::setw(2) << std::setfill('0')
        << input;
    auto && buf = oss.str();
    //add the stx
    QByteArray writeData = QByteArray(buf.c_str());
    for(int i=0;i<writeData.size();i++){
        switch (writeData[i]) {
        case 'a' : writeData[i] = 'A'; break;
        case 'b' : writeData[i] = 'B'; break;
        case 'c' : writeData[i] = 'C'; break;
        case 'd' : writeData[i] = 'D'; break;
        case 'e' : writeData[i] = 'E'; break;
        case 'f' : writeData[i] = 'F'; break;
        default: break;
        }
    }
    return writeData;
}




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

    log_viewer.set_text_view(outputLogTextEdit);

    //------------------------------------------
#ifdef VANCOUVER
    qInfo("Vancouver, Aerotech A3200 gantry");
    mMotionHandler = new AerotechMotionHandler();
#elif VALENCIA
    qInfo("Valencia, ACSC gantry");
    mMotionHandler = new ACSCMotionHandler();
    QJoysticks* J_instance = QJoysticks::getInstance();
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

    //setup of buttons in the GUI
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
    ui->zAxisPositionMoveDoubleSpinBox->setMinimum(-100.0);
    ui->zAxisPositionMoveDoubleSpinBox->setMaximum(100.0);
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
    ui->uAxisPositionMoveDoubleSpinBox->setMinimum(-360.0);
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
    //timer for update of feedback position
    mPositionTimer = new QTimer(this);
    connect(mPositionTimer, SIGNAL(timeout()), this, SLOT(updatePosition()));
    mPositionTimer->start(100);//ms

    //------------------------------------------
    //camera

    //create camera objects
    mCamera = new QCamera(this);
    mCameraViewfinder = new CameraView(this); //new QCameraViewfinder(this);
    mCameraImageCapture = new QCameraImageCapture(mCamera, this);
    mCameraLayout = new QVBoxLayout;

    mCameraViewfinder->setMouseTracking(true);
    connect(mCameraViewfinder, SIGNAL(mouseMoved(QMouseEvent *)), this, SLOT(camera_mouse_moved(QMouseEvent *)));

    //add the camera to the layout
    mCamera->setViewfinder(mCameraViewfinder);
    mCameraLayout->addWidget(mCameraViewfinder);
    mCameraLayout->setContentsMargins(0,0,0,0);

    //add the layout to the frame area in the GUI
    ui->frame->setLayout(mCameraLayout);

    /**
     * TODO: need to implement a mechanism to provide the petal locator coordinates.
     *       For now this is done when we click the Find Petal button in Loading.
     */
    //petal_locations.set_reference()


    /*
     * Camera to Gantry transform
     * Moves from pixels to um in the gantry
     */
    cameraM.rotate(mCamera_angle);
    cameraM.scale(1.0/mCalibration);

    ///////////////////
    //------------------------------------------
    //Init of Real Joystick - Valencia ONLY
#ifdef  VALENCIA
    int J_number = J_instance->count();
    //std::cout<<" J_numebr :"<<J_number<<std::endl;
    QStringList J_list = J_instance->deviceNames();
    J_number = J_list.size();
    //std::cout<<" J_size :"<<J_number<<std::endl;
    int J_buttons = J_instance->getNumButtons(0);
    //std::cout<<" J_buttons :"<<J_buttons<<std::endl;
    connect(J_instance,SIGNAL(buttonChanged(int,int,bool)), this, SLOT(J_translator(int,int,bool)));
    connect(J_instance,SIGNAL(axisChanged(int,int,double)), this, SLOT(J_axes_translator(int,int,double)));
    connect(this,SIGNAL(Run_focus_signal()), this, SLOT(createTemplate_F()));
    connect(this,SIGNAL(Test_signal()), this, SLOT(TestButtonClick()));
#endif
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
    //Create combobox for module loading tab
    ui->Module_comboBox->addItem("R0");
    ui->Module_comboBox->addItem("R1");
    ui->Module_comboBox->addItem("R2");
    ui->Module_comboBox->addItem("R3 Left");
    ui->Module_comboBox->addItem("R3 Right");
    ui->Module_comboBox->addItem("R4 Left");
    ui->Module_comboBox->addItem("R4 Right");
    ui->Module_comboBox->addItem("R5 Left");
    ui->Module_comboBox->addItem("R5 Right");

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
    connect(ui->F_fid_gen_button,SIGNAL(clicked(bool)),this,SLOT(createTemplate_F()));
    connect(ui->focusalgotest_pushButton,SIGNAL(clicked(bool)),this,SLOT(FocusAlgoTest_Func()));
    connect(ui->button_measure_30,SIGNAL(clicked(bool)),this,SLOT(loop_find_circles()));
    connect(ui->button_measure_1_well,SIGNAL(clicked(bool)),this,SLOT(loop_fid_finder()));

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

    //test
    connect(ui->color_button,SIGNAL(clicked(bool)), this, SLOT(color_test()));
    connect(ui->destroy_Button,SIGNAL(clicked(bool)), this, SLOT(destroy_all()));
    connect(ui->f_loop_button,SIGNAL(clicked(bool)), this, SLOT(loop_test_images()));
    connect(ui->DelLogButton,SIGNAL(clicked(bool)),outputLogTextEdit,SLOT(clear()));
    connect(ui->Run_calib_plate_button,SIGNAL(clicked(bool)),this,SLOT(calibration_plate_measure()));
    //connect(ui->Run_calib_plate_button,SIGNAL(clicked(bool)),this,SLOT(fiducial_chip_measure()));
    connect(ui->TestButton,SIGNAL(clicked(bool)),this,SLOT(TestButtonClick()));
    //connect(ui->TestButton,SIGNAL(clicked(bool)),this,SLOT(loop_test_pressure()));

    /*
     * Petal location
     */
    connect(ui->btn_locate_petal, SIGNAL(clicked(bool)),this,SLOT(set_petal_coordinates()));
}

//******************************************
Magrathea::~Magrathea()
{
    delete ui;
    delete mMotionHandler;
}

//position update
void Magrathea::updatePosition(){

    if (!mMotionHandler->gantryConnected)
        return;

    //In Valencia calibrated feedback positions need to be accessed with whereAmI(1), non calibrated, with whereAmI(0)
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

    double current_t = mMotionHandler->CurrentAmI(1);//reading current in motor of axis Z1
    ui->lineEdit_Z_1_current->setText(QString::number(current_t, 'f', 6));
    current_t = mMotionHandler->CurrentAmI(2);//reading current in motor of axis Z2
    ui->lineEdit_Z_2_current->setText(QString::number(current_t, 'f', 6));

    //axes status update
    std::vector <bool> status_axes;
    std::vector <QString> m_labels{"STOP","MOVING"};
    mMotionHandler->getXAxisState(status_axes);
    led_label(ui->label_8, status_axes[0]);
    led_label(ui->label_75, status_axes[1],m_labels);
    ui->EnableButton_X->setText((status_axes[0] ? "Disable" : "Enable"));

    mMotionHandler->getYAxisState(status_axes);
    led_label(ui->label_10, status_axes[0]);
    led_label(ui->label_76, status_axes[1],m_labels);
    ui->EnableButton_Y->setText((status_axes[0] ? "Disable" : "Enable"));

    mMotionHandler->getZAxisState(status_axes);
    led_label(ui->label_12, status_axes[0]);
    led_label(ui->label_77, status_axes[1],m_labels);
    ui->EnableButton_Z->setText((status_axes[0] ? "Disable" : "Enable"));

    mMotionHandler->getZ_2_AxisState(status_axes);
    led_label(ui->label_14, status_axes[0]);
    led_label(ui->label_78, status_axes[1],m_labels);
    ui->EnableButton_Z_2->setText((status_axes[0] ? "Disable" : "Enable"));

    mMotionHandler->getUAxisState(status_axes);
    led_label(ui->label_16, status_axes[0]);
    led_label(ui->label_79, status_axes[1],m_labels);
    ui->EnableButton_U->setText((status_axes[0] ? "Disable" : "Enable"));

    //RealJoystick status update
    if(J_control_Rotation){
        ui->label_J_control->setStyleSheet("QLabel { background-color : yellow; color : black; }");
        ui->label_J_control->setText("Rotation");
    }else{
        if(J_control_Z_1){
            ui->label_J_control->setStyleSheet("QLabel { background-color : green; color : white; }");
            ui->label_J_control->setText("Z 1");
        }else{
            ui->label_J_control->setStyleSheet("QLabel { background-color : green; color : white; }");
            ui->label_J_control->setText("Z 2");
        }
    }

    //reading fault state for each axis
    led_label(ui->label_9 ,!mMotionHandler->GetfaultSateXAxis(), std::vector<QString>{"Out of Env","Good"});
    led_label(ui->label_11,!mMotionHandler->GetfaultSateYAxis(), std::vector<QString>{"Out of Env","Good"});
    led_label(ui->label_13,!mMotionHandler->GetfaultSateZAxis(), std::vector<QString>{"Out of Env","Good"});
    led_label(ui->label_15,!mMotionHandler->GetfaultSateZ2Axis(),std::vector<QString>{"Out of Env","Good"});
    ui->label_17->setText("Always good");
    ui->label_17->setStyleSheet("QLabel { background-color : yellow; color : black; }");

    return;
}


void Magrathea::camera_mouse_moved(QMouseEvent *event)
{
    std::ostringstream ostr;
    QPoint P = event->pos();
    ostr << P.x() << ", " << P.y() << std::endl;
    ui->imageCoord->setText(QString::fromStdString(ostr.str()));
    //std::cout << "Mouse moved " << P.x() << " " << P.y() << std::endl;
}

//******************************************
//real joystick
void Magrathea::J_axes_translator(int index, int axis, double value)
{
    const double threshold = 0.15;
    if (index != 0)
        return; //I want only the main joystick to work
    if (axis == 0 && (value > threshold))
        mMotionHandler->runX(-1, ui->spinBox_J_speed->value() * value);
    else if (axis == 0 && (value < -threshold))
        mMotionHandler->runX(+1, ui->spinBox_J_speed->value() * value);
    else if (axis == 0 && ((value < threshold) || (value > -threshold)))
        mMotionHandler->endRunX();
    else if (axis == 1 && (value > threshold))
        mMotionHandler->runY(-1, ui->spinBox_J_speed->value() * value);
    else if (axis == 1 && (value < -threshold))
        mMotionHandler->runY(+1, ui->spinBox_J_speed->value() * value);
    else if (axis == 1 && ((value < threshold) || (value > -threshold)))
        mMotionHandler->endRunY();
    else if (axis == 2 && !J_control_Rotation && J_control_Z_1)
    {
        //std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> A"<<std::endl;
        if (value > threshold)
            mMotionHandler->runZ(+1, ui->spinBox_J_speed->value() * value);
        else if (value < -threshold)
            mMotionHandler->runZ(-1, ui->spinBox_J_speed->value() * value);
        else if ((value < threshold) || (value > -threshold))
            mMotionHandler->endRunZ();
    }
    else if (axis == 2 && !J_control_Rotation && !J_control_Z_1)
    {
        //std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> B"<<std::endl;
        if (value > threshold)
            mMotionHandler->runZ_2(+1, ui->spinBox_J_speed->value() * value);
        else if (value < -threshold)
            mMotionHandler->runZ_2(-1, ui->spinBox_J_speed->value() * value);
        else if ((value < threshold) || (value > -threshold))
            mMotionHandler->endRunZ_2();
    }
    else if (axis == 2 && J_control_Rotation)
    {
        //std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> C"<<std::endl;
        if (value > threshold)
            mMotionHandler->runU(+1, ui->spinBox_J_speed->value() * value);
        else if (value < -threshold)
            mMotionHandler->runU(-1, ui->spinBox_J_speed->value() * value);
        else if ((value < threshold) || (value > -threshold))
            mMotionHandler->endRunU();
    }
}

void Magrathea::J_translator(int index, int button, bool pressed)
{
    //need to translate the generic joystick slots to one specific for every function
    //An other way may be to add inputs to other functions, but this may be more complicated and less friendly to other sites
    if (index != 0)
        return; //I want only the main joystick to work
    if (button == 1 && pressed)
        ui->spinBox_J_speed->setValue(ui->spinBox_J_speed->value() + 1);
    if (button == 0 && pressed)
        ui->spinBox_J_speed->setValue(ui->spinBox_J_speed->value() - 1);
    if (button == 3 && pressed)
        ui->spinBox_J_speed->setValue(ui->spinBox_J_speed->value() + 15);
    if (button == 2 && pressed)
        ui->spinBox_J_speed->setValue(ui->spinBox_J_speed->value() - 15);
    if (button == 5 && pressed)
    {
        //std::cout<<"buuton 5"<<std::endl;
        std::vector<double> limits;
        limits.clear();
        limits.push_back(ui->doubleSpinBox_X_min_lim->value());
        limits.push_back(ui->doubleSpinBox_Y_min_lim->value());
        limits.push_back(ui->doubleSpinBox_Z_1_min_lim->value());
        limits.push_back(ui->doubleSpinBox_Z_2_min_lim->value());
        limits.push_back(ui->doubleSpinBox_X_max_lim->value());
        limits.push_back(ui->doubleSpinBox_Y_max_lim->value());
        limits.push_back(ui->doubleSpinBox_Z_1_max_lim->value());
        limits.push_back(ui->doubleSpinBox_Z_2_max_lim->value());
        mMotionHandler->SetLimitsController(limits);
    }
    if (button == 4 && pressed)
    {
        //to be merged in one button
        //std::cout<<"buuton 4"<<std::endl;
        std::vector<double> limits;
        limits.clear();
        mMotionHandler->GetLimitsController(limits);
        ui->lineEdit_X_min_lim->setText(QString::number(limits.at(0), 'f', 3));
        ui->lineEdit_Y_min_lim->setText(QString::number(limits.at(1), 'f', 3));
        ui->lineEdit_Z_1_min_lim->setText(QString::number(limits.at(2), 'f', 3));
        ui->lineEdit_Z_2_min_lim->setText(QString::number(limits.at(3), 'f', 3));
        ui->lineEdit_X_max_lim->setText(QString::number(limits.at(4), 'f', 3));
        ui->lineEdit_Y_max_lim->setText(QString::number(limits.at(5), 'f', 3));
        ui->lineEdit_Z_1_max_lim->setText(QString::number(limits.at(6), 'f', 3));
        ui->lineEdit_Z_2_max_lim->setText(QString::number(limits.at(7), 'f', 3));
    }
    if (button == 10 && pressed)
    {
        J_control_Z_1 = !J_control_Z_1;
        mMotionHandler->endRunZ();
        mMotionHandler->endRunZ_2();
    }
    if (button == 11 && pressed)
    {
        J_control_Rotation = !J_control_Rotation;
        mMotionHandler->endRunZ();
        mMotionHandler->endRunZ_2();
        mMotionHandler->endRunU();
    }
    if (button == 9 && pressed)
        //emit Run_focus_signal();
        emit Test_signal();
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

void Magrathea::FocusAlgoTest_Func(){//test of foucus algorithm with library of images
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
    mat_mat = cv::imread( file_name, cv::IMREAD_COLOR);
    const int kernel_size = ( (mat_mat.rows > 1000 && mat_mat.cols > 1000) ? 11 : 5);
    FocusFinder->Set_ksize(kernel_size);
    FocusFinder->Set_color_int(ui->ColorBox->value());
    cv::Rect region(0,0,mat_mat.cols,mat_mat.rows-30);
    cv::Mat RoI = mat_mat(region);
    std::vector<double> figures_of_merit;
    FocusFinder->eval_stddev(RoI,figures_of_merit);
    if(figures_of_merit.size() != 0){//writing output to file
        qInfo("%s  %5.1f  %5.5f  %5.1f  %5.1f",Images[i].c_str(),figures_of_merit[0],figures_of_merit[1],figures_of_merit[2],figures_of_merit[3]);
        std_dev_value.push_back(figures_of_merit[0]);
        std::string file_name = "focus_m.txt";
        std::ofstream ofs (file_name, std::ofstream::app);
        ofs << i <<" "<<Images[i]<<" "<<  figures_of_merit[0]<<" "<<figures_of_merit[1]<<" "<<figures_of_merit[2]<<" "<<figures_of_merit[3]<<std::endl;
        ofs.close();

    }
    ///draw histogram
    /// Establish the number of bins
    cv::Mat mat_mat_g = cv::imread( file_name, cv::IMREAD_COLOR);
    int histSize = 256;

    /// Set the ranges ( for B,G,R) )
    float range[] = { 0, 256 } ;
    const float* histRange = { range };

    bool uniform = true; bool accumulate = false;

    cv::Mat g_hist;

    /// Compute the histograms of the image:
    calcHist( &mat_mat_g, 1, nullptr, cv::Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
    // Draw the histograms for B, G and R
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );
    cv::Mat histImage( hist_h, hist_w, CV_8UC1, cv::Scalar(0) );
    normalize(g_hist, g_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

    /// Draw for each channel
    for( int i = 1; i < histSize; i++ )
    {
        line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
              cv::Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
              cv::Scalar(255), 2, 8, 0  );
    }
    /// Display
    cv::namedWindow("calcHist Demo", cv::WINDOW_AUTOSIZE );
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

bool Magrathea::CVCaptureButtonClicked(){//function to capture an image using OpenCV functions
    mCamera->stop(); //closing QCamera
    std::cout<<" ok1 "<<std::endl;
    cv::VideoCapture cap(ui->spinBox_dummy->value()); // open the video camera no. 0
    if (!cap.isOpened()){ //Opening opencv-camera, needed for easier image manipulation
        QMessageBox::critical(this, tr("Error"), tr("Could not open camera"));
        return false;}
    std::cout<<" ok2 "<<std::endl;
    //veryfing that the setting of the camera is optimal
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 3856);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 2764);
    cap.set(cv::CAP_PROP_FPS, 4.0);
    //cap.set(cv::CAP_PROP_GAIN, 4.0);
    std::cout<<" ok3 "<<std::endl;
    cv::Mat mat_from_camera;
    if (!cap.read(mat_from_camera)){ //if not success
        qInfo("Cannot read a frame from video stream");
        return false;
    }
    std::cout<<" ok4 "<<std::endl;
    cap.release();
    std::cout<<" ok4.1 "<<std::endl;
    std::string one     = std::to_string(ui->spinBox_plate_position->value());
    std::string two     = std::to_string(ui->chip_number_spinBox->value());
    std::string three   = std::to_string(ui->spinBox_input->value());
    cv::imwrite("EXPORT/Image_"+one+"_"+two+"_"+three+".jpg",mat_from_camera);
    std::cout<<" ok5 "<<std::endl;
    return true;
}

bool Magrathea::focusButtonClicked() //function to perform autofocus
{
    qInfo(" > camera focus ... ");
    QElapsedTimer timer;
    timer.start();

    //initiating focus finder class, where all he algorithms are
    Focus_finder * FocusFinder = new Focus_finder(this);
    mCamera->stop();//closing QCamera

    // open the video camera no. 0
    cv::VideoCapture cap(ui->spinBox_dummy->value());
    if (!cap.isOpened())
    {
        //Opening opencv-camera, needed for easier image manipulation
        QMessageBox::critical(this, tr("Error"), tr("Could not open camera"));
        return false;
    }
    //get the width and height of frames of the video
    double dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

    //veryfing that the setting of the camera is optimal
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    //cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', '8', '0', '0'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 3856);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 2764);
    cap.set(cv::CAP_PROP_FPS, 4.0);

    //get the width and height of frames of the video
    dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    qInfo("Frame size : %6.0f x %6.0f",dWidth,dHeight);

    //giving focus fnder class control over the camera and the gantry
    FocusFinder->Set_camera(cap);
    FocusFinder->Set_gantry(mMotionHandler);
    FocusFinder->Set_log(outputLogTextEdit);

    //selecting wich color is gong to be used for autofocus, in general shuld be green
    //in valencia we have a color camera
    FocusFinder->Set_color_int(ui->ColorBox->value());

    //settng kernel size to evaluate the figure of merit for the focus
    const int kernel_size = ( (dWidth > 2000 && dHeight > 2000) ? 11 : 5);
    FocusFinder->Set_ksize(kernel_size);
    double focus_position = -1.;
    if (sender() == ui->std_dev_button)
    {    //just evaluating the figures of merit, without autofocus
        cv::Mat mat_from_camera;
        bool bSuccess = cap.read(mat_from_camera);
        if (!bSuccess)
        { //if not success
            qInfo("Cannot read a frame from video stream");
            return false;
        }
        std::vector<double> figures_of_merit;
        FocusFinder->eval_stddev_ROI(mat_from_camera,figures_of_merit);
        if(figures_of_merit.size() != 0)
        qInfo(" Lap : %5.5f;  StdDev : %5.5f;  1st der : %5.5f;  canny edge : %5.5f; ",figures_of_merit[0],figures_of_merit[1],figures_of_merit[2],figures_of_merit[3]);
    }
    else if(sender() == ui->std_dev_many_button)
    {
        FocusFinder->Eval_syst_scan(); //evaluating a scan
    }
    else
    {
        FocusFinder->find_focus(focus_position); //running autofocus
    }
    qInfo(" > camera focus : %3.5f",focus_position);
    delete FocusFinder;
    std::cout<<">>> The FOCUS operation took "<< timer.elapsed() <<" milliseconds"<<std::endl;
    cap.release();

    //Going back to QCameraa
    //mCamera->start(); //this is commented because the code crashes when gantry is moving and QCamera is on. Need investigation

    return true;
}

//------------------------------------------
//capture picture with QCamera
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
//https://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html#videocapture-open
void Magrathea::Camera_test(){//function to test camera settings
    cv::VideoCapture cap(0); // open the video camera no. 0

    if (!cap.isOpened())  // if not success, exit program
    {
        qWarning("Cannot open the video cam");
        getchar();
        return;
    }

    double dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    qInfo("Frame size : %6.0f x %6.0f",dWidth,dHeight);

    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    //cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', '8', '0', '0'));
    //cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', 'U', 'Y', '2'));//https://www.fourcc.org/yuv.php
    //cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', 'U', 'Y', 'V'));

    //cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 3856);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 2764);
    cap.set(cv::CAP_PROP_FPS, 4.0);

    dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    qInfo("Frame size : %6.0f x %6.0f",dWidth,dHeight);

    qInfo("cap.get(CV_CAP_PROP_POS_MSEC);      : %5.5f",cap.get(cv::CAP_PROP_POS_MSEC));
    qInfo("cap.get(CV_CAP_PROP_POS_FRAMES );   : %5.5f",cap.get(cv::CAP_PROP_POS_FRAMES ));
    qInfo("cap.get(CV_CAP_PROP_POS_AVI_RATIO); : %5.5f",cap.get(cv::CAP_PROP_POS_AVI_RATIO));
    qInfo("cap.get(CV_CAP_PROP_FRAME_WIDTH );  : %5.5f",cap.get(cv::CAP_PROP_FRAME_WIDTH ));
    qInfo("cap.get(CV_CAP_PROP_FRAME_HEIGHT);  : %5.5f",cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    qInfo("cap.get(CV_CAP_PROP_FPS );          : %5.5f",cap.get(cv::CAP_PROP_FPS ));
    qInfo("cap.get(CV_CAP_PROP_FOURCC);        : %5.5f",cap.get(cv::CAP_PROP_FOURCC));
    qInfo("cap.get(CV_CAP_PROP_FRAME_COUNT );  : %5.5f",cap.get(cv::CAP_PROP_FRAME_COUNT ));
    qInfo("cap.get(CV_CAP_PROP_FORMAT );       : %5.5f",cap.get(cv::CAP_PROP_FORMAT ));
    qInfo("cap.get(CV_CAP_PROP_MODE );         : %5.5f",cap.get(cv::CAP_PROP_MODE ));
    qInfo("cap.get(CV_CAP_PROP_BRIGHTNESS);    : %5.5f",cap.get(cv::CAP_PROP_BRIGHTNESS));
    qInfo("cap.get(CV_CAP_PROP_CONTRAST);      : %5.5f",cap.get(cv::CAP_PROP_CONTRAST));
    qInfo("cap.get(CV_CAP_PROP_SATURATION);    : %5.5f",cap.get(cv::CAP_PROP_SATURATION));
    qInfo("cap.get(CV_CAP_PROP_HUE);           : %5.5f",cap.get(cv::CAP_PROP_HUE));
    qInfo("cap.get(CV_CAP_PROP_GAIN);          : %5.5f",cap.get(cv::CAP_PROP_GAIN));
    qInfo("cap.get(CV_CAP_PROP_GAMMA);         : %5.5f",cap.get(cv::CAP_PROP_GAMMA));
    qInfo("cap.get(CV_CAP_PROP_EXPOSURE);      : %5.5f",cap.get(cv::CAP_PROP_EXPOSURE));
    qInfo("cap.get(CV_CAP_PROP_CONVERT_RGB);   : %5.5f",cap.get(cv::CAP_PROP_CONVERT_RGB));
    qInfo("cap.get(CV_CAP_PROP_RECTIFICATION); : %5.5f",cap.get(cv::CAP_PROP_RECTIFICATION));
    qInfo("cap.get(CV_CAP_PROP_ISO_SPEED );    : %5.5f",cap.get(cv::CAP_PROP_ISO_SPEED ));
    qInfo("cap.get(CV_CAP_PROP_BUFFERSIZE );   : %5.5f",cap.get(cv::CAP_PROP_BUFFERSIZE ));

    cv::namedWindow("MyVideo", cv::WINDOW_AUTOSIZE); //create a window called "MyVideo"
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
    getchar();
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
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 3856);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 2764);
    cap.set(cv::CAP_PROP_FPS, 5.0);
    double dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
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
{
    Point dummy;
    FiducialFinderCaller(0, dummy);
}

void Magrathea::Circles_button_Clicked()
{
    Point dummy;
    FiducialFinderCaller(2, dummy);
}

//function to find fiducial markers
bool Magrathea::FiducialFinderCaller(const int &input, Point& F_point)
{

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
    double dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    if(debug)
        qInfo("Frame size : %6.0f x %6.0f",dWidth,dHeight);
    //setting camera in the appropriate way
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 3856);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 2764);
    cap.set(cv::CAP_PROP_FPS, 4.0);
    dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
    if(debug)
        qInfo("Frame size : %6.0f x %6.0f",dWidth,dHeight);

    FiducialFinder * Ffinder = new FiducialFinder();

    bool from_file = ui->calib_from_file_Box->isChecked();
    // TODO: Ffinder->Set_log(outputLogTextEdit);

    std::string timestamp = "";
    //loading template
    //std::string address = "D:/Images/Templates_mytutoyo/";
    //std::string address = "C:/Users/Silicio/WORK/MODULE_ON_CORE/medidas_fiduciales_CNM/Imagenes_fiduciales/mag_15X/Sensor_estandar/Todas/templates/";
    std::string address = "D:/Gantry/cernbox/GANTRY-IFIC/Pictures_general/Templates/";

    std::string Images[] = {
                            address + "F-cnm.png",
                            address + "E-cnm.png",
//        address + "atlasE_fiducial_chip_1_1_pos_1.TIF",  //0
//        address + "Fiducial_chip_1_1_pos_1.TIF",
//        address + "atlas_g_chip_1_1_pos_1.TIF",
//        address + "atlas_h_chip_1_1_pos_1.TIF",
//        address + "aruco_f_chip_1_1_pos_1.TIF",
//        address + "aruco_l_chip_1_1_pos_1.TIF", //5
//        address + "aruco_f_chip_1_1_pos_1.TIF",
//        address + "aruco_h_chip_1_1_pos_1.TIF",
//        address + "aruco_o_chip_1_1_pos_2.TIF",
//        address + "aruco_g_chip_1_1_pos_1.TIF",
//        address + "aruco_M_chip_1_1_pos_1.TIF", //10
//        address + "fiducialE.png",
//        address + "fiducialF.png",
//        address + "e_perfect_4_5.png",
//        address + "f_perfect_4_5.png",
//        address + "e_fid.jpg",                           //15
//        address + "f_fid.jpg",
//        address + "ar_m_fid.jpg",
//        address + "fid_test_1.jpg",
//        address + "fid_test_2.jpg",
//        address + "fid_test_3.jpg", //20
//        address + "placa_fid_73_F.jpg",
//        address + "placa_fid_F.jpg",
//        address + "placa_fid_test3_dotsandcrosses.jpg",
//        address + "003_F.jpg",
//        address + "003_F3.jpg"
    };
    int indx = ui->spinBox_input_F->value();
    if (indx>1)
        indx=0;

    os << loglevel(Log::debug) << "Searching fiducial " << indx << " - " << Images[indx] << std::endl;
    Ffinder->SetImageFiducial(Images[ui->spinBox_input_F->value()]
            ,cv::IMREAD_COLOR);

    std::string tmp_filename = "";
    bool success = false;

    if (from_file)
    { //loading image if need to run on loaded image instead than image from the camera
        std::string address =
                "C:/Users/Silicio/cernbox/Gantry_2018/Camera_tests/Calibration_plate_measures/pos_5/";
        std::string Images[] = { "Circles_0_2_0.jpg",
        //"chip_1_1_pos_1.TIF"
                };

        tmp_filename = Images[ui->spinBox_input->value()];
        Ffinder->SetImage(address + Images[ui->spinBox_input->value()], cv::IMREAD_COLOR);
    }
    else
    { //running on image from the camera
        bool bSuccess = cap.read(mat_from_camera);
        if (!bSuccess)
        { //if not success
            qInfo("Cannot read a frame from video stream");
            return false;
        }
        Ffinder->SetImage(mat_from_camera);
    }
    qInfo("Calibration value is : %5.3f [px/um]",mCalibration);

    Point position(Point::NaN());
    MatrixTransform outM;
    if(input == 1 || input == 0)
    {//finding generic template, using SURF
        //here you can apply condition on the found match to evaluate if it is good or bad
        int fail_code = 0;

        // TODO: give center of RoI
        position = Ffinder->FindFiducial(outM, fail_code, position, debug);
        success = (fail_code==0);
        if (fail_code)
        {
            os << loglevel(Log::error) << "FiducialFinder: Error finding fiducial: rc=" << fail_code << std::endl;
        }
        else
            success = true;

    }
    else
    {//finding circles
        /*
         * TODO: how to pass this information to the program.
         *       May be we send a radius depending on the fiducial
         *       we want to find
         */
        double expected_R = 150*mCalibration;
        double range=0.25;
        double min_dist=-1.0;
        Point origin(Point::NaN());
        std::vector<Circle> circles;
        int rc = Ffinder->FindCircles(circles, expected_R, range, min_dist, origin, debug);
        if ( circles.size()==0 )
        {
            // May be a G-type
            expected_R = 12.5*mCalibration;
            min_dist = 2*expected_R;
            range = 0.25;
            os << loglevel(Log::info) << " Trying with 4 circles" << std::endl;
            rc = Ffinder->FindCircles(circles, expected_R, range, min_dist, Point(0,0), debug);
        }



        std::cout << "CIRCLES: rc=" << rc << " No. of circles: " << circles.size() << std::endl;
        for (auto c : circles )
        {
            std::cout << c.get_center() << " - R=" << c.get_R() << std::endl;
        }
        // We pick the first one...
        success = (circles.size()>0);
        if (success)
        {
            position =  circles[0].get_center();
        }
        QTime now = QTime::currentTime();
        QString time_now = now.toString("hhmmss");
        timestamp = time_now.toLocal8Bit().constData();
    }

    if(success)
        qInfo("Displacement from expected position is: %5.2f um along X, %5.2f um along Y",position.x(), position.y());
    else
    {
        qInfo("Fiducial fail");
        return false;
    }
    //writing output to file
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

    /*
     * Correct by camera transformation
     */
    os << "FiducialFinderCaller:  Position in image " << position << " px"<< std::endl;
    position.y( - position.y() );
    F_point = (cameraM * position);
	F_point *= 0.001; // from um to mm
	os << "... and returns " << F_point << " mm" << std::endl;


#if VALENCIA
    //taking into account orientation of camera wrt gantry
    //double target_x_short = - distance_x*0.001*cos(mCamera_angle) - distance_y*0.001*sin(mCamera_angle);
    //double target_y_short = distance_x*0.001*sin(mCamera_angle) - distance_y*0.001*cos(mCamera_angle);
    ofs<<" "<<pos_t[0]-F_point.x()<<" "<<pos_t[1]-F_point.y()<<" "<<pos_t[4]<<std::endl;
#else
    ofs << std::endl;
    mMotionHandler->moveXBy(F_point.x(),1);
    mMotionHandler->moveYBy(F_point.y(),1);
#endif
    ofs.close();
    return true;
 }

//------------------------------------------
//calibrate

void Magrathea::Calibration_ButtonClicked()
{    calibrationCaller(0); }
//calibration of px/um of the camera
void Magrathea::calibrationCaller(int input){
    //Eventually add command to move the gantry to the place where the
    //calibration area is.
    cv::destroyAllWindows();//closing all openCV windows. Otherwise the code may crash
    mCamera->stop(); //closing QCamera
    Calibrator * calibrator = new Calibrator(this);

    bool from_file = ui->calib_from_file_Box->isChecked();
    cv::VideoCapture cap(ui->spinBox_dummy->value()); // open the video camera no. 0

    if (!cap.isOpened()){
        //Opening opencv-camera, needed for easier image manipulation
        QMessageBox::critical(this, tr("Error"), tr("Could not open camera"));
        return;}

    double dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    qInfo("Frame size : %6.0f x %6.0f",dWidth,dHeight);

    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    //cap.set(CV_CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', '8', '0', '0'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 3856);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 2764);
    cap.set(cv::CAP_PROP_FPS, 4.0);
    dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    qInfo("Frame size : %6.0f x %6.0f",dWidth,dHeight);
    cv::Mat mat_from_camera;
    if(from_file){//library of images  to test the algorithm
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
                ,cv::IMREAD_COLOR);
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
#ifdef  VALENCIA
        if(!mMotionHandler->SetLimitsController())
            return;
        std::vector<double> limits;
        limits.clear();
        if(!mMotionHandler->GetLimitsController(limits))
            return;
        if(limits.size() != 8){
            qWarning("Error get limit position. wrong limits size");
            return;
        }
        ui->lineEdit_X_min_lim->setText(QString::number(     limits.at(0), 'f', 3));
        ui->lineEdit_Y_min_lim->setText(QString::number(    limits.at(1), 'f', 3));
        ui->lineEdit_Z_1_min_lim->setText(QString::number(    limits.at(2), 'f', 3));
        ui->lineEdit_Z_2_min_lim->setText(QString::number(    limits.at(3), 'f', 3));
        ui->lineEdit_X_max_lim->setText(QString::number(    limits.at(4), 'f', 3));
        ui->lineEdit_Y_max_lim->setText(QString::number(    limits.at(5), 'f', 3));
        ui->lineEdit_Z_1_max_lim->setText(QString::number(    limits.at(6), 'f', 3));
        ui->lineEdit_Z_2_max_lim->setText(QString::number(    limits.at(7), 'f', 3));
#endif
    } else {
        std::vector<bool> x_status;
        mMotionHandler->getXAxisState(x_status);
        std::vector<bool> y_status;
        mMotionHandler->getYAxisState(y_status);
        std::vector<bool> z_status;
        mMotionHandler->getZAxisState(z_status);
        std::vector<bool> z2_status;
        mMotionHandler->getZ_2_AxisState(z2_status);
        std::vector<bool> u_status;
        mMotionHandler->getUAxisState(u_status);
        if (    x_status[0]  ||
                y_status[0]  ||
                z_status[0]  ||
                z2_status[0] ||
                u_status[0]    ) {
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
    std::vector <QString> m_labels{"STOP","MOVING"};
    if (sender() == ui->xAxisPositionMoveButton){
        led_label(ui->label_75,true,m_labels);
        mMotionHandler->moveXTo(ui->xAxisPositionMoveDoubleSpinBox->value(), ui->xAxisSpeedDoubleSpinBox->value());
    }else if (sender() == ui->yAxisPositionMoveButton){
        led_label(ui->label_76,true,m_labels);
        mMotionHandler->moveYTo(ui->yAxisPositionMoveDoubleSpinBox->value(), ui->yAxisSpeedDoubleSpinBox->value());
    }else if (sender() == ui->zAxisPositionMoveButton){
        led_label(ui->label_77,true,m_labels);
        mMotionHandler->moveZTo(ui->zAxisPositionMoveDoubleSpinBox->value(), ui->zAxisSpeedDoubleSpinBox->value());
    }else if (sender() == ui->z_2_AxisPositionMoveButton){
        led_label(ui->label_78,true,m_labels);
        mMotionHandler->moveZ_2_To(ui->z_2_AxisPositionMoveDoubleSpinBox->value(), ui->z_2_AxisSpeedDoubleSpinBox->value());
    }else if (sender() == ui->uAxisPositionMoveButton){
        led_label(ui->label_79,true,m_labels);
        mMotionHandler->moveUTo(ui->uAxisPositionMoveDoubleSpinBox->value(), ui->uAxisSpeedDoubleSpinBox->value());
    }
    return;
}

void Magrathea::AxisEnableDisableButton(){
    std::vector <bool> status_axes;
    if(sender() == ui->EnableButton_X){
        mMotionHandler->getXAxisState(status_axes);
        mMotionHandler->enableXAxis(!status_axes[0]);
    }else if (sender() == ui->EnableButton_Y){
        mMotionHandler->getYAxisState(status_axes);
        mMotionHandler->enableYAxis(!status_axes[0]);
    }else if (sender() == ui->EnableButton_Z){
        mMotionHandler->getZAxisState(status_axes);
        mMotionHandler->enableZAxis(!status_axes[0]);
    }else if (sender() == ui->EnableButton_Z_2){
        mMotionHandler->getZ_2_AxisState(status_axes);
        mMotionHandler->enableZ_2_Axis(!status_axes[0]);
    }else if (sender() == ui->EnableButton_U){
        mMotionHandler->getUAxisState(status_axes);
        mMotionHandler->enableUAxis(!status_axes[0]);
    }else
        qWarning("Warning! Improper use of function AxisEnableDisableButton.");
}

//functions to change color of labels
void Magrathea::led_label(QLabel *label, bool value){
    if(value){
        label->setStyleSheet("QLabel { background-color : green; color : black; }");
        label->setText("ON");
    }else{
        label->setStyleSheet("QLabel { background-color : red; color : white; }");
        label->setText("OFF");
    }
}

void Magrathea::led_label(QLabel *label, bool value, const std::vector <QString> &input){
    if(input.size() ==2){
        if(value){
            label->setStyleSheet("QLabel { background-color : green; color : black; }");
            label->setText(input[1]);
        }else{
            label->setStyleSheet("QLabel { background-color : red; color : white; }");
            label->setText(input[0]);
        }
    } else
        led_label(label,value);

}

void Magrathea::color_test(){//test of opencv camera selecting one color channel
    std::cout<<"here"<<std::endl;

    cv::destroyAllWindows();
    mCamera->stop(); //closing QCamera
    cv::VideoCapture cap(ui->spinBox_dummy->value()); // open the video camera no. 0

    if (!cap.isOpened()){
        //Opening opencv-camera, needed for easier image manipulation
        QMessageBox::critical(this, tr("Error"), tr("Could not open camera"));
        return;}

    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 3856);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 2764);
    cap.set(cv::CAP_PROP_FPS, 4.0);
    double dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
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

bool Magrathea::loop_test_pressure(){
    //function to test the touchdown function systematically
    if(!mMotionHandler->moveZTo(-47.100,1.))
        return false;
    if(!mMotionHandler->WaitZ())
        return false;
    std::cout<<"start!! "<<std::endl;
    for(int j=0;j<100;j++){//set appropriate value of the loop limit
        std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> iteration : "<<j<<std::endl;
        if(!touchDown(0.018))
            return false;
        if(!mMotionHandler->moveZTo(-47.100,1.))
            return false;
        if(!mMotionHandler->WaitZ())
            return false;
    }
    return true;
}
bool Magrathea::loop_test(){
    //run fiducial finding algo automatically
    //and move to the fiducial position
    for(int j=0;j<70;j++){//set appropriate value of the loop limit
        ui->chip_number_spinBox->setValue(j);
        if(!mMotionHandler->moveXBy(0.070,1.))//dispalacement added for systematic testing of the algorithm
            return false;
        if(!mMotionHandler->WaitX())
            return false;
        if(!mMotionHandler->moveYBy(0.070,1.))
            return false;
        if(!mMotionHandler->WaitY())
            return false;
        for(int i=0;i<5;i++){//set appropriate value of the loop limit
            std::cout<<"j "<<j<<" ; i "<<i<<std::endl;
            ui->spinBox_input->setValue(i);
            Point target;
            //if(!focusButtonClicked())
            //    return;
            if(!FiducialFinderCaller(2,target))
            {
                std::cout<<"FAIL!!"<<std::endl;
                return false;
            }

            // Already done in FiducialFinderCaller
            //double target_x_short = - distances[0]*cos(mCamera_angle) - distances[1]*sin(mCamera_angle);
            //double target_y_short = distances[0]*sin(mCamera_angle) - distances[1]*cos(mCamera_angle);
            //ATTENTION! distances[0] is cols, distances[1] is rows of the image
            //std::vector <double> pos_t_1 = mMotionHandler->whereAmI(1);
            if(!mMotionHandler->moveXBy(-target.x(),1.))
                return false;
            if(!mMotionHandler->moveYBy(-target.y(),1.))
                return false;
        }
    }
    return true;
}

bool Magrathea::loop_find_circles()
{
    //run fiducial finding algo automatically
    for (int j = 0; j < 30; j++)
    {            //set appropriate value of the loop limit
        ui->chip_number_spinBox->setValue(j);
        std::cout << "j " << j << std::endl;
        Point target;
        if (!FiducialFinderCaller(2, target))
        {
            std::cout << "FAIL!!" << std::endl;
            return false;
        }
    }
    return true;
}

bool Magrathea::loop_test_images(){
    //run fiducial finding algo automatically on CNM chip
    cv::destroyAllWindows();
    std::string timestamp = "";
    std::string address_images = "C:/Users/Silicio/cernbox/Gantry_2018_BIG/Fiducial_chip_images_NewOptics_20190306/";
    std::string address_fiducial = "C:/Users/Silicio/cernbox/Gantry_2018_BIG/Templates_NewOptics/";
    std::string Images_fiducial[] = {
        address_fiducial + "ATLAS_E.jpg",
        address_fiducial + "ATLAS_F.jpg",
        address_fiducial + "ATLAS_G.jpg",
        address_fiducial + "ATLAS_H.jpg",
        address_fiducial + "ATLAS_I.jpg"
    };
    for(int i=0;i<34;i++){//chip number
        for(int j=0;j<5;j++){//chip row (i.e. fiducial type)
            for(int m=0;m<8;m++){//chip column
                std::cout<<"i "<<i<<" ; j "<<j<<" ; m "<<m<<std::endl;
                cv::destroyAllWindows();
                FiducialFinder * Ffinder = new FiducialFinder();

                // TODO - Ffinder->Set_log(outputLogTextEdit);
                Ffinder->SetImageFiducial(Images_fiducial[j]
                        ,cv::IMREAD_COLOR);
                std::string one     = std::to_string(i);
                std::string two     = std::to_string(j);
                std::string three   = std::to_string(m);
                std::string image_address = "Image_"+one+"_"+two+"_"+three+".jpg";
                Ffinder->SetImage(address_images + image_address,cv::IMREAD_COLOR);

                Point position;
                MatrixTransform outM;
                int fail_code = 0;
                position = Ffinder->FindFiducial(outM, fail_code);
                bool success = (fail_code==0);
                std::string file_name = (success ? "output_success" : "output_fail");
                file_name += ("_" +two+".txt");
                std::ofstream ofs (file_name, std::ofstream::app);
                ofs <<i<<" "<<j<<" "<<m<<" "<<timestamp<<" "<<fail_code<<" "<<position.x()<<" "<<position.y()<<std::endl;
                ofs.close();
                delete Ffinder;
                //ATTENTION! distances[0] is cols, distances[1] is rows of the image
            }
        }
    }
    return true;
}

bool Magrathea::loop_fid_finder()
{
    //run fiducial finding algo automatically
    //and move to the fiducial position
    for (int i = 0; i < 4; i++)
    {    //set appropriate value of the loop limit
        QApplication::processEvents();
        std::cout << "It " << i << std::endl;
        Point target;
        int input = ((i == 3) ? 3 : 2);
        if (!FiducialFinderCaller(input, target))
        {
            std::cout << "FAIL!!" << std::endl;
            return false;
        }
        // already done in FiducialFinderCaller
        //double target_x_short = - distances[0]*cos(mCamera_angle) - distances[1]*sin(mCamera_angle);
        //double target_y_short = distances[0]*sin(mCamera_angle) - distances[1]*cos(mCamera_angle);
        //ATTENTION! distances[0] is cols, distances[1] is rows of the image
        if (!mMotionHandler->moveXBy(-target.x(), 1.))
            return false;
        if (!mMotionHandler->WaitX(-1))
            return false;
        if (!mMotionHandler->moveYBy(-target.y(), 1.))
            return false;
        if (!mMotionHandler->WaitY(-1))
            return false;
    }
    if (sender() == ui->button_measure_1_well)
    {
        auto one = std::to_string(ui->spinBox_plate_position->value());
        QTime now = QTime::currentTime();
        QString time_now = now.toString("hhmmss");
        std::string timestamp = time_now.toLocal8Bit().constData();

        std::vector<double> pos_t_1 = mMotionHandler->whereAmI(1);
        std::string file_name = "Calibration_plate_position_" + one + ".txt";
        std::ofstream ofs(file_name, std::ofstream::app);
        ofs << timestamp << " " << ui->chip_number_spinBox->value() << " " << pos_t_1[0]
                << " " << pos_t_1[1] << " " << pos_t_1[4] << std::endl;
        ofs.close();
        ui->chip_number_spinBox->setValue(ui->chip_number_spinBox->value() + 1);
    }
    return true;
}

bool Magrathea::loop_fid_finder(int input)
{
    LoggerStream os;
    //run fiducial finding algo automatically
    //and move to the fiducial position
    for (int i = 0; i < 3; i++)
    {    //set appropriate value of the loop limit
        QApplication::processEvents();
        Point target;
        if (!FiducialFinderCaller(input, target))
        {
            std::cout << "FAIL!!" << std::endl;
            return false;
        }
        os << "Iteration " << i << " find pos " << target << std::endl;
        // Already done in FiducialFinderCaller
        //double target_x_short = - distances[0]*cos(mCamera_angle) - distances[1]*sin(mCamera_angle);
        //double target_y_short = distances[0]*sin(mCamera_angle) - distances[1]*cos(mCamera_angle);
        //ATTENTION! distances[0] is cols, distances[1] is rows of the image
        if (!mMotionHandler->moveXBy(-target.x(), 1.))
            return false;
        if (!mMotionHandler->WaitX(-1))
            return false;
        if (!mMotionHandler->moveYBy(-target.y(), 1.))
            return false;
        if (!mMotionHandler->WaitY(-1))
            return false;

        Point new_pos( mMotionHandler->whereAmI(1));
        os << "... moved to " << new_pos << " at end of iteration" << std::endl;
    }
    return true;
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
            QApplication::processEvents();
            speed = (i!=0 && j==0) ? 6. : 3.;
            ui->spinBox_input->setValue(j);
            double target_x = points[0][0] + step_x*j*cos(angle) - step_y*i*sin(angle);
            double target_y = points[0][1] + step_x*j*sin(angle) + step_y*i*cos(angle);
            std::cout<<j<<" "<<i<<" target_x "<<target_x<<" target_y "<<target_y<<std::endl;
            if(!mMotionHandler->moveXTo(target_x,speed))
                return false;
            if(!mMotionHandler->WaitX(-1))
                return false;
            if(!mMotionHandler->moveYTo(target_y,3.))
                return false;
            if(!mMotionHandler->WaitY(-1))
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

bool Magrathea::fiducial_chip_measure(){
    //Function to take pictures of image from CNM
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

    double step_x = 0.5;
    double step_y = 0.3;
    double big_step_x = 4.72; //distance between the first fiducial of two adjiacent chips
    double big_step_y = 4.5029;  //vertical distance between the first fiducial of two adjiacent chips
    double speed  = 3.;
    for(int m = 0; m<34;m++){//chip ID from 0 to 34
        ui->spinBox_plate_position->setValue(m);
        int working_m  = (m < 17) ? m : (m-17);
        int second_row = (m < 17) ? 0 : 1;
        double big_target_x = points[0][0] + big_step_x*working_m*cos(angle) - big_step_y*second_row*sin(angle);
        double big_target_y = points[0][1] + big_step_x*working_m*sin(angle) + big_step_y*second_row*cos(angle);
        std::cout<<m<<" BIG : target_x "<<big_target_x<<" target_y "<<big_target_y<<std::endl;
        if(!mMotionHandler->moveXTo(big_target_x,speed))
            return false;
        if(!mMotionHandler->WaitX(-1))
            return false;
        if(!mMotionHandler->moveYTo(big_target_y,3.))
            return false;
        if(!mMotionHandler->WaitY(-1))
            return false;
        for(int i=0;i<12;i++){
            ui->chip_number_spinBox->setValue(i);
            for(int j=0;j<8;j++){
                cv::destroyAllWindows();
                ui->spinBox_input->setValue(j);
                double target_x = big_target_x + step_x*j*cos(angle) - step_y*i*sin(angle);
                double target_y = big_target_y + step_x*j*sin(angle) + step_y*i*cos(angle);
                std::cout<<i<<" "<<j<<" target_x "<<target_x<<" target_y "<<target_y<<std::endl;
                if(!mMotionHandler->moveXTo(target_x,speed))
                    return false;
                if(!mMotionHandler->WaitX(-1))
                    return false;
                if(!mMotionHandler->moveYTo(target_y,3.))
                    return false;
                if(!mMotionHandler->WaitY(-1))
                    return false;
                if(!focusButtonClicked())
                    return false;
                std::cout<<" >> ok "<<i<<" "<<j<<std::endl;
                if(!CVCaptureButtonClicked())
                    return false;

                auto one = std::to_string(ui->spinBox_plate_position->value());

                std::vector <double> pos_t_1 = mMotionHandler->whereAmI(1);
                std::string file_name = "Image_position_"+one+".txt";
                std::ofstream ofs (file_name, std::ofstream::app);
                ofs<<m<<" "<<i<<" "<<j<<" "<<pos_t_1[0]<<" "<<pos_t_1[1]<<" "<<pos_t_1[4]<<std::endl;
                ofs.close();

                std::vector <double> pos_t_2 = mMotionHandler->whereAmI(0);
                std::string file_name_2 = "Image_position_"+one+"_other_var.txt";
                std::ofstream ofs_2 (file_name_2, std::ofstream::app);
                ofs_2<<m<<" "<<i<<" "<<j<<" "<<pos_t_2[0]<<" "<<pos_t_2[1]<<" "<<pos_t_2[4]<<std::endl;
                ofs_2.close();
            }
        }
    }
    return true;
}

int Magrathea::TestButtonClick(){//dummy function to perform simple tests

    //touchDown(0.018);
    touchDown(ui->doubleSpinBox_thresholdTouch->value());

    return 0;
    std::vector<std::string> arguments;
    arguments.push_back("DI  ");
    //arguments.push_back("PS  ");
    //arguments.push_back("0500");
    TalkSR232(arguments);
    //    Sleeper::msleep(500);
//    if(!mMotionHandler->moveYBy(30,5.))
//        return 1;


    return 0;

    //try to add three buttons to mimic the
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Warning", "Module placement error is greater than 20 um! Would you like to adjust the module?",
                                  QMessageBox::Yes|QMessageBox::No);
    if(reply == QMessageBox::Yes){
        std::cout<<" Yes!!!"<<std::endl;
    }else if(reply == QMessageBox::No){
        std::cout<<" No!!!"<<std::endl;
    }
    std::cout<<" Something something!"<<std::endl;

    QMessageBox::information(this,
                             "Step to be taken by user 2",
                             "Please turn OFF the vacuum of the gantry. Push ok to continue.",QMessageBox::Ok);
    std::cout<<" Something something!"<<std::endl;

    //    FiducialFinder * Ffinder = new FiducialFinder(this);
    //    Ffinder->Set_log(outputLogTextEdit);
    //    Ffinder->dumb_test();

    //touchDown(2,0.5,0.2);

    return 0;
}


Point Magrathea::find_coordinates_at_position(const Point &estimated_point, int fiducial_type, double speed)
    throw(MagratheaException)
{
    LoggerStream os;

    if(!mMotionHandler->moveXTo(estimated_point.x(), speed))
        throw(MagratheaException("Cannot move to given X postion"));

    if(!mMotionHandler->WaitX(-1))
        throw(MagratheaException("Error waiting to X movement to finish"));;

    if(!mMotionHandler->moveYTo(estimated_point.y(), speed))
        throw(MagratheaException("Cannot move to given Y postion"));

    if(!mMotionHandler->WaitY(-1))
        throw(MagratheaException("Error waiting to Y movement to finish"));

    //turn on the light (if needed in setup) and autofocus
    if(!focusButtonClicked())
        throw(MagratheaException("Error while focusing in Z"));

    //find a circle of 300 um diameter
    if(!loop_fid_finder(fiducial_type))
        throw(MagratheaException("Error while Finding fiducial"));

    //Get the current positon
    std::vector <double> the_position = mMotionHandler->whereAmI(1);
    Point position(the_position[0], the_position[1], the_position[2]);

    os << loglevel(Log::info) << "Fiducial position: " << position << std::endl;
    return position;
}

/**
 * This is called when the "Locate Petal" button is clicked.
 * The idea is that this opens a dialog that allows to get, via joystick, a first estimate of the petal locators.
 */
bool Magrathea::set_petal_coordinates()
{
    os << loglevel(Log::info) << "set petal coordinates" << std::endl;
    QPetalLocator P(mMotionHandler);
    P.show();
    if ( P.exec() == QDialog::Accepted )
    {
        Point Pup = P.get_top_position();
        Point Pbot = P.get_bottom_position();
        os << loglevel(Log::info) << Pup << Pbot << std::endl;
        if (Pup.is_nan() || Pbot.is_nan())
        {
            std::ostringstream ostr;
            ostr << "Invalid Locator coorditates:" << std::endl
                    << "+ Top: " << Pup << std::endl
                    << "+ Bot: " << Pbot << std::endl;

            QMessageBox msgBox;

            msgBox.setText(ostr.str().c_str());
            msgBox.exec();
        }
        else
        {
            FindPetal(Pup, Pbot);
        }
    }
    os << loglevel(Log::info) << "####" << std::endl;
    /*
     * Say we have two petal coordinates we got by inspecting with the joystick.
     */
    return true;
}



//--------------------------------------------------------
// Porting from Scott code.
//Function L744 FindPetal()
int Magrathea::FindPetal(Point &top_locator, Point &bottom_locator )
{
    LoggerStream os;
    cv::destroyAllWindows();

    /*
     * Get current position
     */
    std::vector<double> current_pos( mMotionHandler->whereAmI(1) );
    Point here(current_pos[0], current_pos[1], current_pos[3]);
    Point top, bottom;

    /*
     * Find the locators. Start with the closer one.
     */
    try
    {
        if ( here.distance(top_locator) < here.distance(bottom_locator) )
        {
            os << "Moving to top " << std::endl;
            top = find_coordinates_at_position(top_locator, 4, 20.0);
            os << "..." << top << std::endl
               << "Moving to bottom" << std::endl;
            bottom = find_coordinates_at_position(bottom_locator, 4, 20.0);
            os << "..." << bottom << std::endl;
        }
        else
        {
            os << "Moving to bottom " << std::endl;
            bottom = find_coordinates_at_position(bottom_locator, 4, 20.0);
            os << "..." << bottom << std::endl
                        << "Moving to top" << std::endl;
            top = find_coordinates_at_position(top_locator, 4, 20.0);
            os << "..." << top << std::endl;
        }
    }
    catch ( MagratheaException &e )
    {
        return 1;
    }

    /*
     * Compute the new reference system
     */
    petal_locations.set_reference(top, bottom);
    return 0;
}

// Porting from Scott code.
//Function L1204 Place()
int Magrathea::PickAndPlaceModule(const double &PetalAngle,const std::vector<cv::Point3d> &Coordinates ){
    cv::destroyAllWindows();
    double safe_Z_height = -20;
    double safe_Z_ModulePickUp_height = -40;
    double Camera_offset_X = 4321.0;
    double Camera_offset_Y = 4321.0;
    //This has to be evaluated as function of the position in Z
    //of the petal. i.e. using Z_coordinates from FindPetal function
    double safe_Z_ModulePlace_height = -50;
    double PickUpTool_angle = 0.;
    double module_angle = 0.;
    std::vector <double> pos_t_1(4);
    std::vector <double> pos_t_2(4);
    QMessageBox::StandardButton reply;
    QMessageBox::StandardButton info_step;
    enum Step {AllSteps, LocateModule, PickUpTool, PickUpModule, PlaceModule};//enum to run segment of code without splitting the code.
    Step m_step = AllSteps;
    if (sender() == ui->PickUp_Button)
        m_step = PickUpTool;
    else if (sender() == ui->PickUp_Module_Button)
        m_step = PickUpModule;
    else if(sender() == ui->PlaceModule_Button)
        m_step = PlaceModule;

    //Think of how we want the drawing of the petal to be made
    //Maybe load several pictures showing the different steps of the loading.

    //need to load expected positions of the module on the jig and on the petal
    //then need to adjust expected module position by the mesured rotation of the petal

    //Add lockdown of petal location button

    //Move to first (lower-left) 'F' fiducial on sensor (sensor is still in the jig)
    //Add motion with the joystick. May avoid problems with small misalignement of the sensors

    std::vector<cv::Point3d> R0_jig_coordinates (5);
    std::vector<cv::Point3d> R1_jig_coordinates (5);
    std::vector<cv::Point3d> R2_jig_coordinates (5);
    std::vector<cv::Point3d> R3_LEFT_jig_coordinates (5);
    std::vector<cv::Point3d> R3_RIGHT_jig_coordinates (5);
    std::vector<cv::Point3d> R4_LEFT_jig_coordinates (5);
    std::vector<cv::Point3d> R4_RIGHT_jig_coordinates (5);
    std::vector<cv::Point3d> R5_LEFT_jig_coordinates (5);
    std::vector<cv::Point3d> R5_RIGHT_jig_coordinates (5);

    std::vector<std::vector< cv::Point3d> > Jig_coordinates(9);
    Jig_coordinates.at(0)= R0_jig_coordinates;
    Jig_coordinates.at(1)= R1_jig_coordinates;
    Jig_coordinates.at(2)= R2_jig_coordinates;
    Jig_coordinates.at(3)= R3_LEFT_jig_coordinates;
    Jig_coordinates.at(4)= R3_RIGHT_jig_coordinates;
    Jig_coordinates.at(5)= R4_LEFT_jig_coordinates;
    Jig_coordinates.at(6)= R4_RIGHT_jig_coordinates;
    Jig_coordinates.at(7)= R5_LEFT_jig_coordinates;
    Jig_coordinates.at(8)= R5_RIGHT_jig_coordinates;

    std::vector<cv::Point3d> R0_Petal_coordinates (5);
    std::vector<cv::Point3d> R1_Petal_coordinates (5);
    std::vector<cv::Point3d> R2_Petal_coordinates (5);
    std::vector<cv::Point3d> R3_LEFT_Petal_coordinates (5);
    std::vector<cv::Point3d> R3_RIGHT_Petal_coordinates (5);
    std::vector<cv::Point3d> R4_LEFT_Petal_coordinates (5);
    std::vector<cv::Point3d> R4_RIGHT_Petal_coordinates (5);
    std::vector<cv::Point3d> R5_LEFT_Petal_coordinates (5);
    std::vector<cv::Point3d> R5_RIGHT_Petal_coordinates (5);

    std::vector<double> Module_Petal_angles (9);
    std::vector<std::vector< cv::Point3d> > Petal_coordinates(9);
    Petal_coordinates.at(0) = R0_Petal_coordinates;
    Petal_coordinates.at(1) = R1_Petal_coordinates;
    Petal_coordinates.at(2) = R2_Petal_coordinates;
    Petal_coordinates.at(3) = R3_LEFT_Petal_coordinates;
    Petal_coordinates.at(4) = R3_RIGHT_Petal_coordinates;
    Petal_coordinates.at(5) = R4_LEFT_Petal_coordinates;
    Petal_coordinates.at(6) = R4_RIGHT_Petal_coordinates;
    Petal_coordinates.at(7) = R5_LEFT_Petal_coordinates;
    Petal_coordinates.at(8) = R5_RIGHT_Petal_coordinates;

    std::vector< cv::Point3d > PickUpTool_coordinates(9);

    int selected_module_index = ui->Module_comboBox->currentIndex();

    if(m_step == AllSteps || m_step == LocateModule){//<<<<<<<
    reply = QMessageBox::question(this, "Warning", "Is module on jig and vacuum available?",
                                  QMessageBox::Yes|QMessageBox::No);
    if (reply == QMessageBox::No) {
        return 1;
    }

    if(!mMotionHandler->moveXTo(Jig_coordinates[selected_module_index][0].x,12))
        return false;
    if(!mMotionHandler->moveYTo(Jig_coordinates[selected_module_index][0].y,12))
        return false;
    if(!mMotionHandler->moveZTo(Jig_coordinates[selected_module_index][0].z,3))
        return false;
    mMotionHandler->WaitX();
    mMotionHandler->WaitY();
    mMotionHandler->WaitZ();
    if(!focusButtonClicked())
        return false;
    if(!loop_fid_finder(0))
        return false;

    //Store real position of fiducial
    pos_t_1 = mMotionHandler->whereAmI(1);

    //Move to second (upper-right) 'F' fiducial on sensor
    if(!mMotionHandler->moveXTo(Jig_coordinates[selected_module_index][1].x,12))
        return false;
    if(!mMotionHandler->moveYTo(Jig_coordinates[selected_module_index][1].y,12))
        return false;
    if(!mMotionHandler->moveZTo(Jig_coordinates[selected_module_index][1].z,3))
        return false;
    mMotionHandler->WaitX();
    mMotionHandler->WaitY();
    mMotionHandler->WaitZ();
    if(!focusButtonClicked())
        return false;
    if(!loop_fid_finder(0))
        return false;

    //Store real position of fiducial
    pos_t_2 = mMotionHandler->whereAmI(1);

    QMessageBox::information(this, "Information", "Module orientation determined.  \n Moving to pick-up tool...");
    //verify calculation after deciding how the setup is placed on the gantry table
    //take into account properly the position of the fiducials
    module_angle = atan((pos_t_2[1]-pos_t_1[1])/(pos_t_2[0]-pos_t_1[0]));
    } //<<<<<<<

    //setting ifs to allow partial run of the code, without splitting the function
    if (m_step == AllSteps || m_step == PickUpTool){ //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        //Move to correct pick-up tool
        //Fix axis according to final camera setup
        if(!mMotionHandler->moveZTo(safe_Z_height,3))
            return false;
        mMotionHandler->WaitZ();
        if(!mMotionHandler->moveXTo(PickUpTool_coordinates[selected_module_index].x,12))
            return false;
        mMotionHandler->WaitX();
        if(!mMotionHandler->moveYTo(PickUpTool_coordinates[selected_module_index].y,12))
            return false;
        mMotionHandler->WaitY();
        //understand what the velocity means for the U axis
        if(!mMotionHandler->moveUTo(PickUpTool_angle,5))
            return false;
        mMotionHandler->WaitU();

        //Fix axis according to final camera setup
        if(!mMotionHandler->moveZTo(PickUpTool_coordinates[selected_module_index].z,5))
            return false;
        mMotionHandler->WaitU();
        //Slowly touch the pickup tool
        //        if(!mMotionHandler->moveZBy(-2,0.75))
        //            return false;
        touchDown(0.018);
        //turn ON gantry vacuum
        info_step = QMessageBox::information(this,
                                             "Step to be taken by user",
                                             "Please turn ON the vacuum of the gantry. Push ok to continue.",QMessageBox::Ok);

        qInfo("Pick-up tool obtained. \n Moving to module...");
        //Lift pick-up tool up
        //Fix axis according to final camera setup
        if(!mMotionHandler->moveZTo(safe_Z_height,3))
            return false;
        mMotionHandler->WaitZ();

    } // end if sender pickup_button<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    if (m_step == AllSteps || m_step == PickUpModule){ //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        reply = QMessageBox::question(this, "Warning", "Is pick-up tool properly attached to the loading head?",
                                      QMessageBox::Yes|QMessageBox::No);
        if (reply == QMessageBox::No)
            return 1;

        // Calculate x,y coordinates of sensor location
    //rotating by module_angle the fift element of the array, which should be an offset wrt the first fiducial
    //verify the calculation after the setup is designed
        //make position T1 and T2 a defaualt in case of testing
    double target_x = pos_t_1[0] + Jig_coordinates[selected_module_index][5].x*cos(module_angle) - Jig_coordinates[selected_module_index][5].y*sin(module_angle) - Camera_offset_X;
    double target_y = pos_t_1[1] + Jig_coordinates[selected_module_index][5].x*sin(module_angle) - Jig_coordinates[selected_module_index][5].y*cos(module_angle) - Camera_offset_Y;

    //Move to correct location over sensor
    if(!mMotionHandler->moveXTo(target_x,12))
        return false;
    if(!mMotionHandler->moveYTo(target_y,12))
        return false;
    mMotionHandler->WaitX();
    mMotionHandler->WaitY();
    //rotate pickup tool to align with the sensor
    //understand what the velocity means for the U axis
    //verify the calculation after the setup is designed
    if(!mMotionHandler->moveUBy(module_angle,5))
        return false;
    mMotionHandler->WaitU();

    //Pick up module
    if(!mMotionHandler->moveZTo(safe_Z_ModulePickUp_height,3))
        return false;
    mMotionHandler->WaitZ();

    touchDown(0.018);

    //turn OFF gantry vacuum
    info_step = QMessageBox::information(this,
                                         "Step to be taken by user 2",
                                         "Please turn OFF the vacuum of the gantry. Push ok to continue.",QMessageBox::Ok);

    //Lift up and rotate gantry-vacuum over holes
    if(!mMotionHandler->moveZBy(5,1))
        return false;
    mMotionHandler->WaitZ();

    if(!mMotionHandler->moveUBy(45,5))
        return false;
    mMotionHandler->WaitU();

    if(!mMotionHandler->moveZBy(-5,1))//touchdown again?
        return false;
    mMotionHandler->WaitZ();

    //turn ON gantry vacuum
    info_step = QMessageBox::information(this,
                                         "Step to be taken by user 3",
                                         "Please turn ON the vacuum of the gantry. Push ok to continue.",QMessageBox::Ok);

    // Move over petal and place sensor
    if(!mMotionHandler->moveZTo(safe_Z_height,3))
        return false;
    mMotionHandler->WaitZ();
    }//End if PickUp module button<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    if(m_step == AllSteps || m_step == PlaceModule){//<<<<<<<
        reply = QMessageBox::question(this, "Warning", "Is module properly attached to the loading head?",
                                      QMessageBox::Yes|QMessageBox::No);
        if (reply == QMessageBox::No)
            return 1;

    // Calculate x,y coordinates of sensor location
    //rotating by module_angle the fift coordinate, which should be an offset wrt the first fiducial
    //verify the calculation after the setup is designed
    double target_x = Petal_coordinates[selected_module_index][5].x*cos(PetalAngle) - Petal_coordinates[selected_module_index][5].y*sin(PetalAngle) - Camera_offset_X;
    double target_y = Petal_coordinates[selected_module_index][5].y*sin(PetalAngle) + Petal_coordinates[selected_module_index][5].y*cos(PetalAngle) - Camera_offset_Y;

    //Move to correct location over petal
    if(!mMotionHandler->moveXTo(target_x,5))
        return false;
    if(!mMotionHandler->moveYTo(target_y,5))
        return false;
    mMotionHandler->WaitX();
    mMotionHandler->WaitY();


    //align module with petal
    if(!mMotionHandler->moveUTo(PetalAngle + Module_Petal_angles[selected_module_index],5))
        return false;
    mMotionHandler->WaitU();

    // Move over petal and place sensor
    if(!mMotionHandler->moveZTo(safe_Z_ModulePlace_height,3))
        return false;
    mMotionHandler->WaitZ();

    touchDown(0.018);

    // Once petal has been found, slowly move down 50 um more to ensure contact
    //if(!mMotionHandler->moveZBy(-0.05,0.1))//necessary in our case?
    //    return false;
    //mMotionHandler->WaitZ();

    } //end if place module

    if(m_step != AllSteps)
        return true; //finish here if running testing

    //Store position for module adjustment
    std::vector <double> pos_t_3 = mMotionHandler->whereAmI(1);
    cv::Point3d module_bridge_coordinates(pos_t_3[0],pos_t_3[1],pos_t_3[2]);

    //Module in place; waiting 1 minute for glue to settle...
    //morph it into a window with a timer
    Sleeper::sleep(60);

    //turn OFF gantry vacuum
    info_step = QMessageBox::information(this,
                                         "Step to be taken by user 3",
                                         "Please turn OFF the vacuum of the gantry. Push ok to continue.",QMessageBox::Ok);

    // Lift gantry head up (slowly)
    if(!mMotionHandler->moveZBy(2,0.2))
        return false;
    mMotionHandler->WaitZ();

    qInfo("Proceeding to survey placement...");
    std::vector <cv::Point3d> Module_offsets;
    Survey(selected_module_index, PetalAngle, Module_offsets);
    // Check if any of the errors are greater than 20 um
    bool big_errors = false;
    for(unsigned int i=0;i<Module_offsets.size();i++){
        double offset = sqrt(Module_offsets[i].x*Module_offsets[i].x + Module_offsets[i].y*Module_offsets[i].y);
        if(offset > 0.02){
            big_errors = true;
            qInfo("Fiducial %d is out of range : %5.5f",i,offset);
        }
    }
    if(big_errors){
        //try to add three buttons to mimic the vancouve 3-option messagebox
        //https://doc.qt.io/qt-5/qmessagebox.html
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(this, "Warning", "Module placement error is greater than 20 um! Would you like to adjust the module?",
                                      QMessageBox::Yes|QMessageBox::No);
        if(reply == QMessageBox::Yes){
            //If 'Yes', move to adjust and resurvey the placement (below)
            //call adjust function
            //Function inVancouver code does not make sense, rewrite it (or try to understand it...)
            Adjust_module(module_bridge_coordinates,Module_offsets);
        }else if(reply == QMessageBox::No){
            //If 'No', just wait for glue to dry
            qInfo("Module adjustment skipped by user.");
        }
    }

    if(!mMotionHandler->moveZTo(safe_Z_height,3))
        return false;
    mMotionHandler->WaitZ();


    //Turn lamp over module on petal to green, indicating module has been successfully placed
    //PetalUpdate(app,app.ModuleDropDown.Value, 2);

    //Putting back the pick up tool

    if(!mMotionHandler->moveUBy(45,5))
        return false;
    mMotionHandler->WaitU();
    if(!mMotionHandler->moveXTo(module_bridge_coordinates.x,5))
        return false;
    mMotionHandler->WaitX();
    if(!mMotionHandler->moveYTo(module_bridge_coordinates.y,5))
        return false;
    mMotionHandler->WaitY();
    if(!mMotionHandler->moveZTo(module_bridge_coordinates.z+1,5))
        return false;
    mMotionHandler->WaitZ();
    if(!mMotionHandler->moveZBy(-1,0.25))
        return false;
    mMotionHandler->WaitZ();

    //turn ON gantry vacuum
    QMessageBox::information(this,
                             "Step to be taken by user - 4",
                             "Please turn ON the vacuum of the gantry. Push ok to continue.",QMessageBox::Ok);
    //Slowly lift tool off of module
    if(!mMotionHandler->moveZBy(0.5,0.1))
        return false;
    mMotionHandler->WaitZ();
    if(!mMotionHandler->moveZBy(2,0.5))
        return false;
    mMotionHandler->WaitZ();
    if(!mMotionHandler->moveZTo(safe_Z_height,5))
        return false;
    mMotionHandler->WaitZ();

    //Move tool back to tool holder
    if(!mMotionHandler->moveXTo(PickUpTool_coordinates[selected_module_index].x,12))
        return false;
    mMotionHandler->WaitX();
    if(!mMotionHandler->moveYTo(PickUpTool_coordinates[selected_module_index].y,12))
        return false;
    mMotionHandler->WaitY();
    if(!mMotionHandler->moveZTo(PickUpTool_coordinates[selected_module_index].z+2,5))
        return false;
    mMotionHandler->WaitZ();
    //Slowly drop the pickup tool
    if(!mMotionHandler->moveZBy(-2,0.75))
        return false;
    mMotionHandler->WaitZ();

    //turn OFF gantry vacuum
    info_step = QMessageBox::information(this,
                                         "Step to be taken by user - 5",
                                         "Please turn OFF the vacuum of the gantry. Push ok to continue.",QMessageBox::Ok);
    qInfo("Module has been successfully placed!");

    return 0;
}

//L2383
// Function for surveying placement results
// *** When calling function, be sure to check 'motion' in calling
//     function/callback if motion was complete or stopped
//function [aveXYZ, motion, cals] = Survey(app, value)
bool Magrathea::Survey(const int &selected_module_index, const double &PetalAngle, std::vector <cv::Point3d> &Module_offsets){
    //Load module survey data
    cv::destroyAllWindows();
    double safe_Z_height = -20;
    double safe_Z_height_To_autofocus = -30;
    std::vector<cv::Point3d> R0_Petal_coordinates (5);
    std::vector<cv::Point3d> R1_Petal_coordinates (5);
    std::vector<cv::Point3d> R2_Petal_coordinates (5);
    std::vector<cv::Point3d> R3_LEFT_Petal_coordinates (5);
    std::vector<cv::Point3d> R3_RIGHT_Petal_coordinates (5);
    std::vector<cv::Point3d> R4_LEFT_Petal_coordinates (5);
    std::vector<cv::Point3d> R4_RIGHT_Petal_coordinates (5);
    std::vector<cv::Point3d> R5_LEFT_Petal_coordinates (5);
    std::vector<cv::Point3d> R5_RIGHT_Petal_coordinates (5);

    std::vector<double> Module_Petal_angles (9);
    std::vector<std::vector< cv::Point3d> > Petal_coordinates(9);
    Petal_coordinates.at(0)= R0_Petal_coordinates;
    Petal_coordinates.at(1)= R1_Petal_coordinates;
    Petal_coordinates.at(2)= R2_Petal_coordinates;
    Petal_coordinates.at(3)= R3_LEFT_Petal_coordinates;
    Petal_coordinates.at(4)= R3_RIGHT_Petal_coordinates;
    Petal_coordinates.at(5)= R4_LEFT_Petal_coordinates;
    Petal_coordinates.at(6)= R4_RIGHT_Petal_coordinates;
    Petal_coordinates.at(7)= R5_LEFT_Petal_coordinates;
    Petal_coordinates.at(8)= R5_RIGHT_Petal_coordinates;

    //Fix axis according to final camera setup
    if(!mMotionHandler->moveZTo(safe_Z_height,10))
        return false;

    //rotating the expected coordinate of the module on the petal
    //by the measured angle of the petal: PetalAngle

    //////////////////////////////////////////////////////////////////////////
    // Calculate x,y coordinates of sensor location
    //rotating by Petal_angle
    //verify the calculation after the setup is designed

    //Start survey
    //Add an "helper" that allows to move the gantry manually
    //to help the positioning in case of failure
    std::vector < std::vector <double> > Real_Module_On_Petal_Positions (4);
    std::vector <cv::Point3d> Petal_Rotated_coordinates(4) ;
    Module_offsets.clear();
    for(unsigned int i=0;i<4;i++){//fifth point is the center of the module

        Petal_Rotated_coordinates.at(i).x = Petal_coordinates[selected_module_index][i].x*cos(PetalAngle) - Petal_coordinates[selected_module_index][i].y*sin(PetalAngle) ;
        Petal_Rotated_coordinates.at(i).y = Petal_coordinates[selected_module_index][i].x*sin(PetalAngle) + Petal_coordinates[selected_module_index][i].y*cos(PetalAngle) ;

        if(!mMotionHandler->moveZTo(safe_Z_height,3))
            return false;
        mMotionHandler->WaitZ();

        //Move to correct location over petal
        if(!mMotionHandler->moveXTo(Petal_Rotated_coordinates.at(i).x,5))
            return false;
        mMotionHandler->WaitX();
        if(!mMotionHandler->moveYTo(Petal_Rotated_coordinates.at(i).y,5))
            return false;
        mMotionHandler->WaitY();

        if(!mMotionHandler->moveZTo(safe_Z_height_To_autofocus,3))
            return false;
        mMotionHandler->WaitZ();

        //autofocus and find fiducial
        if(!focusButtonClicked())
            return false;
        if(!loop_fid_finder(0))
            return false;

        //Store position somewhere...
        Real_Module_On_Petal_Positions[i] = mMotionHandler->whereAmI(1);

        //Calculate offsets
        //Check axis index after setup is complete
        cv::Point3d temp_point (Real_Module_On_Petal_Positions[i][0],Real_Module_On_Petal_Positions[i][1],Real_Module_On_Petal_Positions[i][2]);
        Module_offsets.push_back(Petal_Rotated_coordinates.at(i) - temp_point);
    }
    //Do something with the values?
    //Print them and decide if going on or not?
    return true;
}

//L2520
// Function for adjusting placed module and waiting for glue ot dry
// *** When calling function, be sure to check 'motion' in calling
//     function/callback if motion was complete or stopped
//function motion = Adjust(app, startX,startY,startZ, offsetX,offsetY, adj
//function [aveXYZ, motion, cals] = Survey(app, value)
bool Magrathea::Adjust_module(const cv::Point3d &module_bridge_coordinates, const std::vector <cv::Point3d> &Module_offsets){
    cv::destroyAllWindows();
    double safe_Z_height = -20;
    if(!mMotionHandler->moveZTo(safe_Z_height,3))
        return false;
    mMotionHandler->WaitZ();

    //Functio does not make sense. Rewrite it!!!

    //    if(!mMotionHandler->moveXTo(module_bridge_coordinates.x,5))
    //        return false;
    //    if(!mMotionHandler->moveYTo(module_bridge_coordinates.y,5))
    //        return false;
    //    if(!mMotionHandler->moveZTo(module_bridge_coordinates.z+1,3))
    //        return false;
    //    if(!mMotionHandler->moveZBy(-1,0.2))
    //        return false;

    //    //turn ON gantry vacuum
    //    QMessageBox::StandardButton info_step;
    //    info_step = QMessageBox::information(this,
    //                                         "Step to be taken by user - adjustement",
    //                                         "Please turn ON the vacuum of the gantry. Push ok to continue.",QMessageBox::Ok);

    //    // Lift module 1/3 thickness of glue (150/3 = 50 um) at 5 um/s
    //    if(!mMotionHandler->moveZBy(0.05,0.005))
    //        return false;

    //    // Wait for Z-height to be established before moving in x/y
    //    Sleeper::sleep(12);

    //    if(!mMotionHandler->moveXTo(Module_offsets[0].x,5))
    //        return false;
    //    if(!mMotionHandler->moveYTo(Module_offsets[0].y,5))
    //        return false;
    //    Sleeper::sleep(5);
    return true;
}

//Function for "force-sensing"
//L1680
bool Magrathea::touchDown(const double &threshold){
    mMotionHandler->endRunZ(); //need to ensure all motion has stopped!!
    //Add asking for axis status
    const double velocity         = 0.1; //[mm/s]
    const double maximum_distance = 1.5; //[mm]
    const int millisec_wait       = 100;
    std::cout<<"Start, T: "<<threshold<<std::endl;
    if(!mMotionHandler->moveZBy(-maximum_distance,velocity))
        return false;
    Sleeper::msleep(1500); //need to wait for inductance of the engine to charge, corresponds to 150 um of travel
    double current0 = mMotionHandler->CurrentAmI(1);
    Sleeper::msleep(millisec_wait);
    double current1 = mMotionHandler->CurrentAmI(1);
    //VALENCIA ONLY: current is returned in % of maximum motor capability
    int flag = 1;
    int iterations =0;
    while (flag > 0){
        Sleeper::msleep(millisec_wait);
        //QApplication::processEvents(); ???
        iterations++;
        double current2 = mMotionHandler->CurrentAmI(1);
        //Take the difference of two consecutive current measurements
        //(spaced 100 ms apart) average time of the current in the motor in Valencia, may be different in other sites
        double compare0 = current1 - current0;
        double compare1 = current2 - current1;
        std::cout<<" Comp 0 : "<<compare0<<" ; Comp 1 : "<<compare1<<
                   " Curr 0 : "<<current0<<" ; Curr 1 : "<<current1<<" ; Curr 2 : "<<current2<<std::endl;
        //Check to see if the differences are both consistent
        // --> if they are, break from the loop and stop motion
        if (compare0 > threshold){
            if (compare1 > threshold){
                mMotionHandler->endRunZ();
                flag = -1;
            }
        }
        if(iterations>140){
            flag = -1;
            std::cout<<"NO touch!!!!!"<<std::endl;
        }
        current0 = current1;
        current1 = current2;
        if(flag < 0){
            Sleeper::msleep(100);
            std::string file_name = "touchDown_good.txt";
            std::ofstream ofs (file_name, std::ofstream::app);
            ofs <<"T: "<<threshold<<" : "<<iterations<<"  "<<mMotionHandler->whereAmI(1).at(2)<<"  "<<current2<<" : "<<compare0<<" : "<<compare1<<std::endl;
            ofs.close();
            if(iterations<=140)
                std::cout<<"Touch down!!!!!"<<std::endl;
        }
        std::string file_name = "touchDown.txt";
        std::ofstream ofs (file_name, std::ofstream::app);
        ofs <<iterations<<"  "<<mMotionHandler->whereAmI(1).at(2)<<"  "<<current2<<" : "<<compare0<<" : "<<compare1<<std::endl;
        ofs.close();
    }
    //////////////////////////////////////
    //    //Stop motion and move 50 um away to reduce pressure
    //    if(!mMotionHandler->moveZ_2_By(0.05,velocity)){ //velocity shuld be ~0.2 mm/sec
    //        return false;
    //    }
    return true;
}

//Function for gluing (lines)
//L1014
bool Magrathea::GlueLines( const std::vector<cv::Point3d> &line_points){
    if(line_points.size()!=2)
        return false;

    const double safe_gluing_Z_height = -20.0; //add correction for petal real positioning
    const double glue_speed = 3.0; //[mm/s]
    //    const std::vector<cv::Point3d> Petal_nominal_coordinates(2);
    //    double Petal_offset_X = Coordinates[0].x-Petal_nominal_coordinates[0].x;
    //    double Petal_offset_Y = Coordinates[0].y-Petal_nominal_coordinates[0].y;
    //evaluate distance between two points
    //    double distance = sqrt(pow((line_points[0].x-line_points[1].x),2)+
    //            pow((line_points[0].y-line_points[1].y),2));
    //double time = distance / glue_speed; //[s]

    //Move to correct location over petal
    if(!mMotionHandler->moveXTo(line_points[0].x,5))
        return false;
    mMotionHandler->WaitX();
    if(!mMotionHandler->moveYTo(line_points[0].y,5))
        return false;
    mMotionHandler->WaitY();
    if(!mMotionHandler->moveZTo(safe_gluing_Z_height,5))
        return false;
    mMotionHandler->WaitZ();

    //send start dispensing to dispeser
    //(Ultimus needs to be in steady "mode", see sec 2.2.27 of manual )
    std::vector<std::string> arguments;
    arguments.push_back("DI  ");
    TalkSR232(arguments);
    //move symultaneously the two axis (verify if this works,
    //or use the appropriate function (that need to be implemented in ACSCMotionhandler) to move two axis at a time)
    mMotionHandler->moveXTo(line_points[0].x,glue_speed);
    mMotionHandler->moveYTo(line_points[0].y,glue_speed);

    //send end dispensing to dispeser
    mMotionHandler->WaitX();
    mMotionHandler->WaitY();

    TalkSR232(arguments);

    return true;
}

bool Magrathea::TalkSR232( const std::vector<std::string> &arguments){
    //    int stx = 2;
    //    int etx = 3;
    //    int eot = 4;
    //    int enq = 5;
    //    int ack = 6;
    //    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
    //            std::cout << "Name : " << info.portName().toLocal8Bit().constData()<<std::endl;
    //            std::cout << "Description : " << info.description().toLocal8Bit().constData()<<std::endl;
    //            std::cout << "Manufacturer: " << info.manufacturer().toLocal8Bit().constData()<<std::endl;
    //            // Example use QSerialPort
    //            QSerialPort serial;
    //            serial.setPort(info);
    //            if (serial.open(QIODevice::ReadWrite))
    //                serial.close();
    //        }
    bool debug = true;
    QByteArray readData;
    QByteArray writeData;
    QSerialPort serialPort;
    const QString serialPortName = "COM1"; //to modify according to the serial port used
    serialPort.setPortName(serialPortName);
    serialPort.setBaudRate(QSerialPort::Baud115200); // set BaudRate to 115200
    serialPort.setParity(QSerialPort::NoParity); //set Parity Bit to None
    serialPort.setStopBits(QSerialPort::OneStop); //set
    serialPort.setDataBits(QSerialPort::Data8); //DataBits to 8
    serialPort.setFlowControl(QSerialPort::NoFlowControl);
    serialPort.close();
    if (!serialPort.open(QIODevice::ReadWrite)) {
        std::cout<<"FAIL!!!!!"<<std::endl;
        qWarning("Failed to open port %s, error: %s",serialPortName.toLocal8Bit().constData(),serialPort.errorString().toLocal8Bit().constData());
        return false;
    }else {
        if (debug)
            std::cout<<"Port opened successfully"<<std::endl;
    }

    writeData = QByteArrayLiteral("\x05"); //sending enquiry command
    long long int output = 0;
    output = serialPort.write(writeData);
    if (debug)
        std::cout<<"Log >> bytes written   : "<<output<<" : operation : "<<writeData.toStdString()<<std::endl;
    if(output == -1){
        std::cout<<"Error write operation : "<<writeData.toStdString()
                << " => " << serialPort.errorString().toStdString()<<std::endl;
        return false;
    }

    readData.clear();
    int control = 0;
    while(serialPort.isOpen()){ // READING BYTES FROM SERIAL PORT
        control += 1;
        //https://stackoverflow.com/questions/42576537/qt-serial-port-reading
        if(!serialPort.waitForReadyRead(100)) //block until new data arrives, dangerous, need a fix
            std::cout << "Read error: " << serialPort.errorString().toStdString()<<std::endl;
        else{
            if (debug)
                std::cout << "New data available: " << serialPort.bytesAvailable()<<std::endl;
            readData = serialPort.readAll();
            if (debug)
                std::cout << readData.toStdString()<<std::endl;
            break;
        }
        if (control > 10){
            std::cout << "Time out read error"<<std::endl;
            return false;
        }

    }// END READING BYTES FROM SERIAL PORT

    if(readData.size() != 0){
        if (debug)
            std::cout<<"Read operation ok : "<<readData.toStdString()<<std::endl;
        if(readData.at(0) != 6){ //expecting acknowledge command (0x06)
            std::cout<<"Wrong read : "<<readData.toStdString()<<std::endl;
            return false;
        }
    }
    ///////////////////////////////////////////////////////////////////////////////
    // Composing message in an appropriate way for the Ultimis V (Sec1 of appB of manual)
    int checksum = 0;
    int N_bytes = 4*arguments.size();
    writeData = QByteArrayLiteral("\x02"); //https://stackoverflow.com/questions/36327327/is-there-a-shorter-way-to-initialize-a-qbytearray
    QByteArray temp_writeData = int_tohexQByteArray_UltimusV(N_bytes);

    for(unsigned int i=0;i<arguments.size();i++)
        temp_writeData.append(QByteArray(arguments[i].c_str()));

    for(int i=0;i<temp_writeData.size();i++)// evauating checksum quantity
        checksum -= temp_writeData[i];

    writeData.append(temp_writeData);

    //take tha least significant byte of checksum, i.e. checksum & 0x000000ff
    temp_writeData.clear();
    temp_writeData = int_tohexQByteArray_UltimusV(checksum & 0x000000ff);
    QByteArray qb_checksum;
    qb_checksum.clear();
    if(temp_writeData.size() > 2){
        if (debug)
            std::cout<<"here : "<<temp_writeData.size()<<"  :  "<<temp_writeData.toStdString();
        qb_checksum = temp_writeData.remove(0,(temp_writeData.size()-2));
        if (debug)
            std::cout<<"CS  :  "<<qb_checksum.toStdString()<<std::endl;
    } else {
        qb_checksum = temp_writeData;
        if (debug)
            std::cout<<"CS  :  "<<qb_checksum.toStdString()<<std::endl;
    }

    writeData.append(qb_checksum);
    writeData.append(QByteArrayLiteral("\x03"));
    writeData.append(QByteArrayLiteral("\x04"));
    //// END OF COMMAND CONSTRUCTION

    output = serialPort.write(writeData);// SENDING MESSAGE TO ULTIMUS V
    if (debug)
        std::cout<<"Log >> bytes written   : "<<output<<" : "<<writeData.toStdString()<<std::endl;
    if(output == -1){
        std::cout<<"Error write operation : "<<writeData.toStdString()<<std::endl;
        std::cout << "error: " << serialPort.errorString().toStdString()<<std::endl;
        return false;
    }
    //////////// End sending UltimusV command
    Sleeper::msleep(200);

    readData.clear();
    control = 0;
    while(serialPort.isOpen()){ //dangerous, may freez the GUI
        control += 1;
        if(!serialPort.waitForReadyRead(100)) //block until new data arrives
            std::cout << "error: " << serialPort.errorString().toStdString()<<std::endl;
        else{
            if (debug)
                std::cout << "2 New data available: " << serialPort.bytesAvailable()<<std::endl;
            readData.append(serialPort.readAll());
            if (debug)
                std::cout << readData.toStdString()<<std::endl;
            if(readData.at(0) == 2 && readData.at(readData.size()-1) == 3) //expectin A0 command, may add controls on checksum in future
                break;
        }
        if (control > 10){
            std::cout << "Time out read error"<<std::endl;
            return false;
        }
    }
    //////////////////////////////////
    serialPort.close(); //closing serial port comunication
    return true;
}




































