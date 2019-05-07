#include "MotionHandler.h"
#include <QtMessageHandler>

//******************************************
MotionHandler::MotionHandler() :
    gantryConnected(false),
    xAxisEnabled(false),
    yAxisEnabled(false),
    zAxisEnabled(false),
    uAxisEnabled(false)
    {}

//******************************************
MotionHandler::~MotionHandler() {}

//******************************************
// connect to the gantry

//------------------------------------------
bool MotionHandler::connectGantry(bool flag)
{
    if (flag) {
        qInfo("connecting gantry...");
        if (true) { //connect gantry here
            qInfo("gantry connected");
            gantryConnected=true;
            return true;
        } else {
            qWarning("could not connect gantry");
            return false;
        }
    } else {
        qInfo("disconnecting gantry...");
        if (true) { //disconnect gantry here
            qInfo("gantry disconnected");
            gantryConnected=false;
            return true;
        }else {
            qWarning("could not disconnect gantry");
            return false;
        }
    }
    return true;
}

//------------------------------------------
bool MotionHandler::disconnectGantry()
{
    return connectGantry(false);
}

//------------------------------------------
bool MotionHandler::stop(){
    qInfo("stopping...");
    if (true) { //stop here
        qInfo("stop");
        return true;
    } else {
        qWarning("could not stop");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::acknowledgeMotionFaultGantry()
{
    qInfo("resetting errors...");
    if (true) { //acknowledge and clear axes faults
        qInfo("errors reset");
        return true;
    } else {
        qWarning("could not reset errors");
        return false;
    }
    return true;}

//------------------------------------------
bool MotionHandler::enableAxes(bool flag)
{
    if (flag) {
        qInfo("enabling axes...");
        if (true) { //enable all axes here
            qInfo("axes enabled");
            return true;
        } else {
            qWarning("could not enable axes");
            return false;
        }
    } else {
        qInfo("disabling axes...");
        if (true) { //disable all axes here
            qInfo("axes disabled");
            return true;
        } else {
            qWarning("could not disable axes");
            return false;
        }
    }
    return true;
}

//------------------------------------------
bool MotionHandler::enableXAxis(bool flag)
{
    if (flag) {
        qInfo("enabling x axis...");
        if (true) { //enable x axis here
            qInfo("x axis enabled");
            xAxisEnabled=true;
            return true;
        } else {
            qInfo("could not enable x axis");
            return false;
        }
    } else {
        qInfo("disabling x axis...");
        if (true) { //disable x axis here
            qInfo("x axis disabled");
            xAxisEnabled=false;
            return true;
        } else {
            qInfo("could not disable x axis");
            return false;
        }
    }
    return true;
}

//------------------------------------------
bool MotionHandler::enableYAxis(bool flag)
{
    if (flag) {
        qInfo("enabling y axis...");
        if (true) { //enable y axis here
            qInfo("y axis enabled");
            yAxisEnabled=true;
            return true;
        } else {
            qInfo("could not enable y axis");
            return false;
        }
    } else {
        qInfo("disabling y axis...");
        if (true) { //disable y axis here
            qInfo("y axis disabled");
            yAxisEnabled=false;
            return true;
        } else {
            qInfo("could not disable y axis");
            return false;
        }
    }
    return true;
}

//------------------------------------------
bool MotionHandler::enableZAxis(bool flag)
{
    if (flag) {
        qInfo("enabling z axis...");
        if (true) { //enable z axis here
            qInfo("z axis enabled");
            zAxisEnabled=true;
            return true;
        } else {
            qInfo("could not enable z axis");
            return false;
        }
    } else {
        qInfo("disabling z axis...");
        if (true) { //disable z axis here
            qInfo("z axis disabled");
            zAxisEnabled=false;
            return true;
        } else {
            qInfo("could not disable z axis");
            return false;
        }
    }
    return true;
}

//------------------------------------------
bool MotionHandler::enableZ_2_Axis(bool flag)
{
    if (flag) {
        qInfo("enabling z 2 axis...");
        if (true) { //enable z axis here
            qInfo("z 2 axis enabled");
            zAxisEnabled=true;
            return true;
        } else {
            qInfo("could not enable z 2 axis");
            return false;
        }
    } else {
        qInfo("disabling z 2 axis...");
        if (true) { //disable z axis here
            qInfo("z 2 axis disabled");
            zAxisEnabled=false;
            return true;
        } else {
            qInfo("could not disable z 2 axis");
            return false;
        }
    }
    return true;
}

//------------------------------------------
bool MotionHandler::enableUAxis(bool flag)
{
    if (flag) {
        qInfo("enabling u axis...");
        if (true) { //enable u axis here
            qInfo("u axis enabled");
            uAxisEnabled=true;
            return true;
        } else {
            qInfo("could not enable u axis");
            return false;
        }
    } else {
        qInfo("disabling u axis...");
        if (true) { //disable u axis here
            qInfo("u axis disabled");
            uAxisEnabled=false;
            return true;
        } else {
            qInfo("could not disable u axis");
            return false;
        }
    }
    return true;
}

//------------------------------------------
bool MotionHandler::disableAxes()
{
    return enableAxes(false);
}

//------------------------------------------
bool MotionHandler::disableXAxis()
{
    return enableXAxis(false);
}

//------------------------------------------
bool MotionHandler::disableYAxis()
{
    return enableYAxis(false);
}

//------------------------------------------
bool MotionHandler::disableZAxis()
{
    return enableZAxis(false);
}

//------------------------------------------
bool MotionHandler::disableZ_2_Axis()
{
    return enableZ_2_Axis(false);
}

//------------------------------------------
bool MotionHandler::disableUAxis()
{
    return enableUAxis(false);
}

//******************************************
// Set Limits
bool MotionHandler::SetLimitsController()
{
    qInfo("setting limits in the controller, the machine should stop if it gets outside of these boundries");
    return true;
}

// Set Limits
bool MotionHandler::SetLimitsController(std::vector <double> & limits)
{
    qInfo("setting limits in the controller, the machine should stop if it gets outside of these boundries");
    return true;
}

bool MotionHandler::GetLimitsController(std::vector <double> & limits){
    std::vector <double> output;
    output.clear();
    output.push_back(4321.9);
    limits = output;
    return true;
};


int  MotionHandler::GetfaultSateXAxis(){
 qInfo("Info on X axis.");
 return 1;
};

int  MotionHandler::GetfaultSateYAxis(){
 qInfo("Info on Y axis.");
 return 1;
};

int  MotionHandler::GetfaultSateZ1Axis(){
 qInfo("Info on Z1 axis.");
 return 1;
};

int  MotionHandler::GetfaultSateZ2Axis(){
 qInfo("Info on Z2 axis.");
 return 1;
};

//******************************************
//Get Axis states
//------------------------------------------
bool MotionHandler::getXAxisState(){
    return xAxisEnabled;
}

//------------------------------------------
bool MotionHandler::getYAxisState(){
    return yAxisEnabled;
}

//------------------------------------------
bool MotionHandler::getZAxisState(){
    return zAxisEnabled;
}

//------------------------------------------
bool MotionHandler::getZ_2_AxisState(){
    return z_2_AxisEnabled;
}

//------------------------------------------
bool MotionHandler::getUAxisState(){
    return uAxisEnabled;
}

//******************************************
// home axes

//------------------------------------------
bool MotionHandler::home() {
    qInfo("homing axes...");
    if (true) { //home all axes here
        qInfo("axes homed");
        return true;
    } else {
        qWarning("could not home axes");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::homeX() {
    qInfo("homing x axis...");
    if (true) { //home x axis here
        qInfo("x axis homed");
        return true;
    } else {
        qWarning("could not home x axis");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::homeY() {
    qInfo("homing y axis...");
    if (true) { //home y axis here
        qInfo("y axis homed");
        return true;
    } else {
        qWarning("could not home y axis");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::homeZ() {
    qInfo("homing z axis...");
    if (true) { //home z axis here
        qInfo("z axis homed");
        return true;
    } else {
        qWarning("could not home z axis");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::homeZ_2() {
    qInfo("homing z 2 axis...");
    if (true) { //home z axis here
        qInfo("z 2 axis homed");
        return true;
    } else {
        qWarning("could not home z 2 axis");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::homeU() {
    qInfo("homing u axis...");
    if (true) { //home u axis here
        qInfo("u axis homed");
        return true;
    } else {
        qWarning("could not home u axis");
        return false;
    }
    return true;
}

//******************************************
// absolute motion
// NOTE units in mm, mm/s and deg/s

//------------------------------------------
bool MotionHandler::moveTo(double x, double y, double z, double speed)
{
    qInfo("moving to (%.3f mm, %.3f mm, %.3f mm) at %.3f mm/s speed...", x, y, z, speed);
    if (true) { //move to destination here
        qInfo("moved to destination");
        return true;
    } else {
        qWarning("could not move to destination");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::moveXTo(double x, double speed) {
    qInfo("moving x axis to %.3f mm at %.3f mm/s speed", x, speed);
    if (true) { //move to destination here
        qInfo("moved x axis to destination");
        return true;
    } else {
        qWarning("could not move x axis to destination");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::moveYTo(double y, double speed) {
    qInfo("moving y axis to %.3f mm at %.3f mm/s speed", y, speed);
    if (true) { //move to destination here
        qInfo("moved y axis to destination");
        return true;
    } else {
        qWarning("could not move y axis to destination");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::moveZTo(double z, double speed) {
    qInfo("moving z axis to %.3f mm at %.3f mm/s speed", z, speed);
    if (true) { //move to destination here
        qInfo("moved z axis to destination");
        return true;
    } else {
        qWarning("could not move z axis to destination");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::moveZ_2_To(double z, double speed) {
    qInfo("moving z 2 axis to %.3f mm at %.3f mm/s speed", z, speed);
    if (true) { //move to destination here
        qInfo("moved z 2 axis to destination");
        return true;
    } else {
        qWarning("could not move z 2 axis to destination");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::moveUTo(double u, double speed) {
    qInfo("moving u axis to %.3f deg at %.3f deg/s speed", u, speed);
    if (true) { //move to destination here
        qInfo("moved u axis to destination");
        return true;
    } else {
        qWarning("could not move u axis to destination");
        return false;
    }
    return true;
}

//******************************************
// step  motion
// NOTE units in mm, mm/s and deg/s

//------------------------------------------
bool MotionHandler::moveBy(double x, double y, double z, double speed)
{
    qInfo("moving by (%.3f mm, %.3f mm, %.3f mm) at %.3f mm/s speed", x, y, z, speed);
    if (true) { //move by step here
        qInfo("moved by step");
        return true;
    } else {
        qWarning("could not move by step");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::moveXBy(double x, double speed) {
    qInfo("moving x axis by %.3f mm at %.3f mm/s", x, speed);
    if (true) { //move by step here
        qInfo("moved x axis by step");
        return true;
    } else {
        qWarning("could not move x axis by step");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::moveYBy(double y, double speed) {
    qInfo("moving y axis by %.3f mm at %.3f mm/s", y, speed);
    if (true) { //move by step here
        qInfo("moved y axis by step");
        return true;
    } else {
        qWarning("could not move y axis by step");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::moveZBy(double z, double speed) {
    qInfo("moving z axis by %.3f mm at %.3f mm/s", z, speed);
    if (true) { //move by step here
        qInfo("moved z axis by step");
        return true;
    } else {
        qWarning("could not move z axis by step");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::moveZ_2_By(double z, double speed) {
    qInfo("moving z 2 axis by %.3f mm at %.3f mm/s", z, speed);
    if (true) { //move by step here
        qInfo("moved z 2 axis by step");
        return true;
    } else {
        qWarning("could not move z 2 axis by step");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::moveUBy(double u, double speed) {
    qInfo("moving u axis by %.3f deg at %.3f deg/s", u, speed);
    if (true) { //move by step here
        qInfo("moved u axis by step");
        return true;
    } else {
        qWarning("could not move u axis by step");
        return false;
    }
    return true;
}


//******************************************
// free run
// NOTE units in mm, mm/s and deg/s

//------------------------------------------
bool MotionHandler::runX(double direction, double speed)
{
    qInfo("free running %sx axis at %.3f mm/s", direction<0?"-":"+", speed);
    if (true) { //free run here
        return true;
    } else {
        qWarning("could not free run along x axis");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::endRunX()
{
    qInfo("stop free running along x axis");
    if (true) { //stop free run here
        return true;
    } else {
        qWarning("could not stop free run along x axis");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::runY(double direction, double speed)
{
    qInfo("free running %sy axis at %.3f mm/s", direction<0?"-":"+", speed);
    if (true) { //free run here
        return true;
    } else {
        qWarning("could not free run along y axis");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::endRunY()
{
    qInfo("stop free running along y axis");
    if (true) { //stop free run here
        return true;
    } else {
        qWarning("could not stop free run along y axis");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::runZ(double direction, double speed)
{
    qInfo("free running %sz axis at %.3f mm/s", direction<0?"-":"+", speed);
    if (true) { //free run here
        return true;
    } else {
        qWarning("could not free run along z axis");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::endRunZ()
{
    qInfo("stop free running along z axis");
    if (true) { //stop free run here
        return true;
    } else {
        qWarning("could not stop free run along z axis");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::runZ_2(double direction, double speed)
{
    qInfo("free running %s z 2 axis at %.3f mm/s", direction<0?"-":"+", speed);
    if (true) { //free run here
        return true;
    } else {
        qWarning("could not free run along z 2 axis");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::endRunZ_2()
{
    qInfo("stop free running along z 2 axis");
    if (true) { //stop free run here
        return true;
    } else {
        qWarning("could not stop free run along z 2 axis");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::runU(double direction, double speed)
{
    qInfo("free running %su axis at %.3f deg/s", direction<0?"-":"+", speed);
    if (true) { //free run here
        return true;
    } else {
        qWarning("could not free run along u axis");
        return false;
    }
    return true;
}

//------------------------------------------
bool MotionHandler::endRunU()
{
    qInfo("stop free running along u axis");
    if (true) { //stop free run here
        return true;
    } else {
        qWarning("could not stop free run along u axis");
        return false;
    }
    return true;
}

//******************************************
//gantry position
std::vector<double> MotionHandler::whereAmI(int ific_value) {
    std::vector<double> position = {qQNaN(), qQNaN(), qQNaN(), qQNaN()};
    if (gantryConnected) {
        position = {qQNaN(), qQNaN(), qQNaN(), qQNaN()};
    }
    return position;
}

//******************************************
//gantry current position
double MotionHandler::CurrentAmI(int ific_value) {
    double current = qQNaN();
    return current;
}

//******************************************
// Safety limits for movement
//******************************************
// Safety limits for movement
bool MotionHandler::validate_target_pos_x(double val){
    if(val < x_min || val > x_max){
        qWarning("ERROR!! Target X position is NOT valid, aborting motion.");
        return false;
    } else {
        return true;
    }
}

bool MotionHandler::validate_target_pos_y(double val){
    if(val < y_min || val > y_max){
        qWarning("ERROR!! Target Y position is NOT valid, aborting motion.");
        return false;
    } else {
        //add control on step
        return true;
    }
}

bool MotionHandler::validate_target_pos_z_1(double val){
    if(val < z_1_min || val > z_1_max){
        qWarning("ERROR!! Target Z1 position is NOT valid, aborting motion.");
        return false;
    } else {
        //add control on step
        return true;
    }
}

bool MotionHandler::validate_target_pos_z_2(double val){
    if(val < z_2_min || val > z_2_max){
        qWarning("ERROR!! Target Z2 position is NOT valid, aborting motion.");
        return false;
    } else {
        //add control on step
        return true;
    }
}

bool MotionHandler::validate_target_pos(double x, double y, double z, double z_2)
{
    qInfo("checking if target position is safe");
    if (true) { // condition is true, position is NOT valid, abort motion.
        return true;
    } else { // conditions are not met, position is valid, carry on...
        qWarning("ERROR!! Target position is NOT valid, aborting motion.");
        return false;
    }
    return true;
}
