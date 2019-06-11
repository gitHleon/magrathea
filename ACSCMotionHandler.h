#ifndef ACSCMOTIONHANDLER_H
#define ACSCMOTIONHANDLER_H

#include "MotionHandler.h"
#include "ACSC/C_CPP/ACSC.h"
#include <QWidget>

class ACSCMotionHandler : public MotionHandler
{
    Q_OBJECT

public:

    ACSCMotionHandler();
    ~ACSCMotionHandler();

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
    // gantry current position
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
    virtual bool moveTo(double positions[4], double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool moveXTo(double x, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool moveYTo(double y, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool moveZTo(double z, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool moveZ_2_To(double z, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool moveUTo(double u, double speed=std::numeric_limits<double>::quiet_NaN());

    //******************************************
    // motion relative to current location
    // NOTE units in mm, mm/s and deg/s
    virtual bool moveBy(double x=0, double y=0, double z=0, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool moveBy(double positions[4], double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool moveXBy(double x=0, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool moveYBy(double y=0, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool moveZBy(double z=0, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool moveZ_2_By(double z=0, double speed=std::numeric_limits<double>::quiet_NaN());
    virtual bool moveUBy(double u=0, double speed=std::numeric_limits<double>::quiet_NaN());

    //******************************************
    // wait motion to end
    virtual bool WaitX  (int timeout = -1); //timeout in milliseconds
    virtual bool WaitY  (int timeout = -1);
    virtual bool WaitZ  (int timeout = -1);
    virtual bool WaitZ_2(int timeout = -1);
    virtual bool WaitU  (int timeout = -1);

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

    // Safety limits for movement
    virtual bool validate_target_pos_x(double val);
    virtual bool validate_target_pos_y(double val);
    virtual bool validate_target_pos_z_1(double val);
    virtual bool validate_target_pos_z_2(double val);
    virtual bool SetLimitsController();
    virtual bool SetLimitsController(std::vector <double> & limits);
    virtual bool GetLimitsController(std::vector <double> & limits);
    virtual bool GetfaultSateXAxis();
    virtual bool GetfaultSateYAxis();
    virtual bool GetfaultSateZAxis();
    virtual bool GetfaultSateZ2Axis();

signals:

    void updatePositions_s();

private:

    HANDLE gantry;
    const int X_axis = ACSC_AXIS_0;
    const int Y_axis = ACSC_AXIS_1;
    //AXIS 2 is y axis yaw, AXIS 3 does not exists
    const int Z_axis = ACSC_AXIS_4;//I need the camera to stay on axis z1
    const int Z_2_axis = ACSC_AXIS_5;
    const int U_axis = ACSC_AXIS_6;

    double Home_coord[5]          = {0.,0.,0.,0.,0.};
    double default_speed         = 15;
    double default_angular_speed = 10;

    double x_min = -450.0;
    double x_max =  500.0;
    double y_min = -350.0;
    double y_max =  500.;
    double z_1_min =  -70.0;
    double z_1_max =  90.0;
    double z_2_min =  -70.0;
    double z_2_max =  90.0;

};

#endif //ACSCMOTIONHANDLER_H
