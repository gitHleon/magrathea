#pragma once
#include <QWidget>

class MotionHandler : public QWidget
{
    Q_GADGET

public:

    MotionHandler();
    ~MotionHandler();

    bool gantryConnected;
    bool xAxisEnabled;
    bool yAxisEnabled;
    bool zAxisEnabled;
    bool z_2_AxisEnabled;
    bool uAxisEnabled;

public slots:

    //******************************************
    // connect to the gantry
    virtual bool connectGantry(bool flag=true);
    virtual bool disconnectGantry();

    //******************************************
    //stop gantry
    virtual bool stop();

    //******************************************
    // acknowledge gantry motion errors
    virtual bool acknowledgeMotionFaultGantry();

    //******************************************
    // enable axes before any movement
    virtual bool enableAxes(bool flag=true);
    virtual bool enableXAxis(bool flag=true);
    virtual bool enableYAxis(bool flag=true);
    virtual bool enableZAxis(bool flag=true);
    virtual bool enableZ_2_Axis(bool flag=true);
    virtual bool enableUAxis(bool flag=true);
    virtual bool disableAxes();
    virtual bool disableXAxis();
    virtual bool disableYAxis();
    virtual bool disableZAxis();
    virtual bool disableZ_2_Axis();
    virtual bool disableUAxis();

    //******************************************
    // gantry position
    virtual std::vector<double> whereAmI(int ific_value = 0);
    virtual double CurrentAmI(int ific_value = 0);
    // gantry axis status
    virtual void getXAxisState(std::vector <bool> &state);
    virtual void getYAxisState(std::vector <bool> &state);
    virtual void getZAxisState(std::vector <bool> &state);
    virtual void getZ_2_AxisState(std::vector <bool> &state);
    virtual void getUAxisState(std::vector <bool> &state);

    //******************************************
    // home axes
    virtual bool home();
    virtual bool homeX();
    virtual bool homeY();
    virtual bool homeZ();
    virtual bool homeZ_2();
    virtual bool homeU();

    //******************************************
    // absolute motion
    // NOTE units in mm, mm/s and deg/s
    virtual bool moveTo(double x, double y, double z, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool moveXTo(double x, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool moveYTo(double y, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool moveZTo(double z, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool moveZ_2_To(double z, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool moveUTo(double u, double speed=std::numeric_limits<double>::quiet_NaN());

    //******************************************
    // motion relative to current location
    // NOTE units in mm, mm/s and deg/s
    virtual bool moveBy(double x=0, double y=0, double z=0, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool moveXBy(double x=0, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool moveYBy(double y=0, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool moveZBy(double z=0, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool moveZ_2_By(double z=0, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool moveUBy(double u=0, double speed=std::numeric_limits<double>::quiet_NaN());

    //******************************************
    // free run
    virtual bool runX(double direction, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool endRunX();
    virtual bool runY(double direction, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool endRunY();
    virtual bool runZ(double direction, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool endRunZ();
    virtual bool runZ_2(double direction, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool endRunZ_2();
    virtual bool runU(double direction, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool endRunU();

    //******************************************
    // Safety limits for movement
    virtual bool validate_target_pos(double x, double y, double z, double z_2=0);
    virtual bool validate_target_pos_x(double val);
    virtual bool validate_target_pos_y(double val);
    virtual bool validate_target_pos_z_1(double val);
    virtual bool validate_target_pos_z_2(double val);
    virtual bool SetLimitsController();
    virtual bool SetLimitsController(std::vector <double> & limits);
    virtual bool GetLimitsController(std::vector <double> & limits);
    virtual int  GetfaultSateXAxis();
    virtual int  GetfaultSateYAxis();
    virtual int  GetfaultSateZ1Axis();
    virtual int  GetfaultSateZ2Axis();
    //******************************************
    // default speeds
    // NOTE default unit is mm/s
    /*
    virtual void SetSpeedDefaults(double speedX=std::numeric_limits<double>::quiet_NaN(),
                                  double speedY=std::numeric_limits<double>::quiet_NaN(),
                                  double speedZ=std::numeric_limits<double>::quiet_NaN(),
                                  double speedU=std::numeric_limits<double>::quiet_NaN());
    virtual void SetSpeedX(double speed=std::numeric_limits<double>::quiet_NaN());
    virtual void SetSpeedY(double speed=std::numeric_limits<double>::quiet_NaN());
    virtual void SetSpeedZ(double speed=std::numeric_limits<double>::quiet_NaN());
    virtual void SetSpeedU(double speed=std::numeric_limits<double>::quiet_NaN());
    */
private :
    double x_min = -200.0;
    double x_max =  200.0;
    double y_min = -200.0;
    double y_max =  200.0;
    double z_1_min =  -20.0;
    double z_1_max =  90.0;
    double z_2_min =  -20.0;
    double z_2_max =  90.0;

};
