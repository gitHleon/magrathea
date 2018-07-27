#include "focus_finder.h"
#include "interpolation.h"

#include <opencv2/opencv.hpp>
#include "stdafx.h"
#include <QThread>
#include <math.h>
#include <QApplication>
#include <QMessageBox>
#ifdef VANCOUVER
#include <AerotechMotionhandler.h>
#elif VALENCIA
#include <ACSCMotionHandler.h>
#endif

//add thresholds and indeces as Valencia and Vancouver variables

//from exaple http://www.pyimagesearch.com/2015/09/07/blur-detection-with-opencv/

Focus_finder::Focus_finder(QObject *parent) : QObject(parent)
{}

Focus_finder::~Focus_finder()
{}

void Focus_finder::Set_gantry(MotionHandler *m_gantry){
    gantry = m_gantry;
}

void Focus_finder::Set_log(QTextEdit *m_log){
    log = m_log;
}

void Focus_finder::Set_camera(const cv::VideoCapture &m_cap){
    cap = m_cap;
}

double Focus_finder::eval_stddev(const cv::Mat &input_image)
{
    cv::Scalar  mean_t;
    cv::Scalar  stddev_t;
    cv::meanStdDev(input_image,mean_t,stddev_t);
    return  stddev_t[0];
}

void Focus_finder::find_focus(double &focus_height)
{
    //Function that return the focus z coordinate
    //https://en.wikipedia.org/wiki/Parabola
    //https://rechneronline.de/function-graphs/
    //http://doc.qt.io/qt-4.8/signalsandslots.html
    if (!cap.isOpened()){
        qWarning("Error : Not able to open camera.");
        return;
    }
    if(!gantry->gantryConnected){
        QMessageBox::critical(nullptr, tr("Error"), tr("Gantry not connected!! Can not perform autofocus."));
        return;
    }
    qInfo("Auto-focus start");
    cv::Mat mat_from_outside;
    cap.read(mat_from_outside);
    int center_rows = mat_from_outside.rows/2.0; //Defining the center of the image
    int center_cols = mat_from_outside.cols/2.0;
    cv::Rect regione_interessante(center_cols-(window_size*0.5),center_rows-(window_size*0.5),window_size,window_size); //Rectangle that will be the RegionOfInterest (ROI)

    double z_step = 0.5;// mm //to be changed according the units of your gantry and shape of focus-height distribution
    double z_from_outside;

    double StdDev_MAX = 1.1;
    double Z_MAX     = 1.;

    int numb_steps = 10;
    double z_temp = gantry->whereAmI(1).at(z_pos_index);
    qInfo("Performing scan around the position : %5.5f",z_temp);
    gantry->moveZBy(-z_step*numb_steps*0.6,1.);

    for(int i=0; i<numb_steps;i++){//large scan to find the position of the focus
        gantry->moveZBy(z_step,1.);//1 mm/s
        cap.read(mat_from_outside);
        Sleeper::msleep(10);
        cap.read(mat_from_outside);
        cv::Mat RoiImage = mat_from_outside(regione_interessante);
        double StdDev_t = eval_stddev(RoiImage);
        z_from_outside = gantry->whereAmI(1).at(z_pos_index);
        qInfo("i : %i ; z : %5.5f ; Std. dev. : %5.5f",i,z_from_outside,StdDev_t);
        if(StdDev_t > StdDev_MAX){
            StdDev_MAX = StdDev_t;
            Z_MAX = z_from_outside;
        }
    }
    if(StdDev_MAX < 30){
        qInfo("Too far from focus position. Camera needs to be < 3 mm far from focus.");
    return;}
    gantry->moveZTo(Z_MAX,1.);//add safety control

    int Iterations = 3;
    z_step = 0.33 * z_step;
    for(int j=0; j<Iterations;j++){//fine scan to find the position of the focus
        gantry->moveZBy(-z_step*ceil(measure_points*0.6),1.);
        for(int i=0; i<measure_points;i++){
            gantry->moveZBy(z_step,1.);
            cap.read(mat_from_outside);
            Sleeper::msleep(10);
            cap.read(mat_from_outside);
            z_from_outside = gantry->whereAmI(1).at(z_pos_index);
            double StdDev_t = eval_stddev(mat_from_outside);
            if(StdDev_t > StdDev_MAX){
                StdDev_MAX = StdDev_t;
                Z_MAX = z_from_outside;
            }
            qInfo("i : %i ; z_step : %3.4f ; z : %3.4f ; stddev : %5.5f ;; z max : %3.4f ; stddev_MAX: %5.5f",
                  i,z_step,z_from_outside,StdDev_t,Z_MAX,StdDev_MAX);
            if(Iterations==(j-1)){
                //this has to be the last iteration
                x[i] = z_from_outside;
                y[i] = StdDev_t;
            }
        }// for 6
        if(StdDev_MAX < 40){
            qInfo("Autofocus failed.");
        return;}
        z_step = 0.33 * z_step; //if step start as 0.5 it should reach 0.02 in 3 iterations
        gantry->moveZTo(Z_MAX,1.);//add safety control
    }

    qInfo("Focus max z : %3.4f ;  stddev_MAX : %5.5f ",Z_MAX,StdDev_MAX);
    double Z_out = 0.;
    perform_fit(Z_out);
    qInfo("Focus fit z : %3.4f",Z_out);
    focus_height = Z_out;//outputvalue
    //z_temp = Z_out - gantry->whereAmI(1).at(z_pos_index); // relative distance from current position
    gantry->moveZTo(Z_MAX,1.);//add safety control
}

void Focus_finder::perform_fit(double &z_output)
{
    //WARNING: these array need to be of size == measure_points
    alglib::real_1d_array al_x = "[0,0,0,0,0,0]";
    alglib::real_1d_array al_y = "[0,0,0,0,0,0]";
    for(int i=0;i<measure_points;i++){
        al_x[i] = x[i];
        al_y[i] = y[i];
    }

    alglib::ae_int_t m = 3;
    alglib::ae_int_t info;
    alglib::barycentricinterpolant p;
    alglib::polynomialfitreport rep;

    // NOTE: result is returned as barycentricinterpolant structure.
    //       if you want to get representation in the power basis,
    //       you can use barycentricbar2pow() function to convert
    //       from barycentric to power representation (see docs for
    //       POLINT subpackage for more info).
    //
    alglib::polynomialfit(al_x, al_y, m, info, p, rep);

    alglib::real_1d_array a2;
    alglib::polynomialbar2pow(p, a2);
    log->append("Coefficients p_1 = a[0]: "+QString::number(a2[0])+" ;a[1]: "+QString::number(a2[1])+" ; a[2]: "+QString::number(a2[2]));
    log->append("Vertex : ( "+QString::number(EvalVertex_x(a2[2],a2[1],a2[0]))+" , "+QString::number(EvalVertex_y(a2[2],a2[1],a2[0]))+" )");
    z_output = EvalVertex_x(a2[2],a2[1],a2[0]);
}

double Focus_finder::EvalVertex_x(double a,double b, double c){
    //y = ax^2 + bx + c
    return -b/(2*a);
}

double Focus_finder::EvalVertex_y(double a,double b, double c){
    //y = ax^2 + bx + c
    double num =0.;
    num = 4*a*c - b*b;
    double den =0.;
    den = 4*a;
    return num/den;
}

void Focus_finder::Eval_syst_scan(){
    int numb_steps = 20;
    double z_temp = gantry->whereAmI(1).at(z_pos_index);
    double z_step = 0.2;// to be changed according the units of your gantry and shape of focus-heught distribution
    qInfo("Performing systematic scan near the focus position : %5.5f",z_temp);
    gantry->moveZBy(-z_step*numb_steps*0.6,1.);
    cv::Mat mat_from_outside;
    for(int i=0; i<numb_steps;i++){
        gantry->moveZBy(z_step,1.);//1 mm/s
        if (cap.isOpened()){
            cap.read(mat_from_outside);
            Sleeper::msleep(10);
            cap.read(mat_from_outside);
        } else {
            qWarning("Error : Not able to open camera.");
            return;
        }
        int center_rows = mat_from_outside.rows/2.0; //Defining the center of the image
        int center_cols = mat_from_outside.cols/2.0;
        cv::Rect regione_interessante(center_cols-(window_size*0.5),center_rows-(window_size*0.5),window_size,window_size); //Rectangle that will be the RegionOfInterest (ROI)
        cv::Mat RoiImage = mat_from_outside(regione_interessante);
        double StdDev_t = eval_stddev(RoiImage);
        cv::imshow("image ROI",RoiImage);
        qInfo("i : %i ; z : %5.5f ; Std. dev. : %5.5f",i,gantry->whereAmI(1).at(z_pos_index),StdDev_t);
    }
}

void Focus_finder::Eval_syst_time(){
    double z_temp = gantry->whereAmI().at(z_pos_index);
    log->append("Performing systematic scan in time (2 sec intervals) at position : "
                    +QString::number(z_temp));
    cv::Mat mat_from_outside;
    for(int i=0; i<7;i++){
        Sleeper::sleep(2);
        if (cap.isOpened()){
            cap >> mat_from_outside;
        } else {
            log->append("Error : Not able to open camera.");
            return;
        }

        double StdDev_t = eval_stddev(mat_from_outside);
        log->append("i : "+QString::number(i)+
                    " ; z : "+QString::number(gantry->whereAmI().at(z_pos_index))+
                    " ; std dev : "+QString::number(StdDev_t));
    }
}

void::Focus_finder::Eval_syst_moving(){
    int numb_steps = 10;
    cv::Mat mat_from_outside;
    double z_temp = gantry->whereAmI().at(z_pos_index);
    log->append("Performing systematic fwd-bkwd stability scan at position : "
                    +QString::number(z_temp));
    double z_step = 1.5; // to be changed according the units of your gantry and shape of focus-height distribution
    for(int i=0; i<numb_steps;i++){
        gantry->moveZBy(-z_step,1.);
        Sleeper::sleep(1);
        if (cap.isOpened()){
            cap >> mat_from_outside;
        } else {
            log->append("Error : Not able to open camera.");
            return;
        }
        double StdDev_t = eval_stddev(mat_from_outside);
        log->append("<< i : "+QString::number(i)+
                    " ; z : "+QString::number(gantry->whereAmI().at(z_pos_index))+
                    " ; std dev : "+QString::number(StdDev_t));
        gantry->moveZBy(z_step,1.);
        Sleeper::sleep(1);
        if (cap.isOpened()){
            cap >> mat_from_outside;
        } else {
            log->append("Error : Not able to open camera.");
            return;
        }
        StdDev_t = eval_stddev(mat_from_outside);
        log->append(">>>> i : "+QString::number(i)+
                    " ; z : "+QString::number(gantry->whereAmI().at(z_pos_index))+
                    " ; std dev : "+QString::number(StdDev_t));
    }
}
