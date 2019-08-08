#include "focus_finder.h"

#include <opencv2/opencv.hpp>
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

void Focus_finder::Set_ksize(int m_ksize){
    ksize = m_ksize;
}
void Focus_finder::Set_camera(const cv::VideoCapture &m_cap){
    cap = m_cap;
}

void Focus_finder::Set_color_int(const int &value){
    color_int = value;
}

cv::Mat  Focus_finder::get_frame_from_camera(){
    cv::Mat output;
    QElapsedTimer timer;
    timer.start();
    //Sleeper::msleep(250);
    for(int i=0;i<2;i++)
        cap.read(output);
    std::cout<<"The slow operation took "<< timer.elapsed() <<" milliseconds"<<std::endl;
    return output;
}

void Focus_finder::addInfo(cv::Mat &image,const std::string &algo_name, int start_x, int start_y,int text_font_size ,int text_thikness,std::string &timestamp, const std::vector<double> &values){
    int baseline = 0;
    //int text_font_size = 2;
    //int text_thikness = 2;

    int window_size = image.rows;
    cv::putText(image,algo_name,cv::Point(start_x,window_size-start_y), cv::FONT_HERSHEY_PLAIN,text_font_size,cv::Scalar(255,255,255),text_thikness);
    cv::Size text_size = cv::getTextSize(algo_name, cv::FONT_HERSHEY_PLAIN,text_font_size,text_thikness,&baseline);
    QTime now = QTime::currentTime();
    QString time_now = now.toString("hhmmss");
    std::string time_now_str = time_now.toLocal8Bit().constData();
    timestamp = time_now_str;
    start_x += text_size.width;
    cv::putText(image,time_now_str,cv::Point(start_x,window_size-start_y), cv::FONT_HERSHEY_PLAIN,text_font_size-1,cv::Scalar(255,255,255),text_thikness);
    cv::Size time_size = cv::getTextSize(time_now_str, cv::FONT_HERSHEY_PLAIN,text_font_size,text_thikness,&baseline);
    start_x += time_size.width;
    for(unsigned int i=0;i<values.size();i++){
        auto s = std::to_string(values.at(i));
        cv::putText(image,s,cv::Point(start_x,window_size-start_y), cv::FONT_HERSHEY_PLAIN,text_font_size-1,cv::Scalar(255,255,255),text_thikness);
        cv::Size s_size = cv::getTextSize(s, cv::FONT_HERSHEY_PLAIN,text_font_size,text_thikness,&baseline);
        start_x += s_size.width;
    }
}

void Focus_finder::eval_stddev(const cv::Mat &input_image,std::vector<double> &output)
{
    output.clear();
    cv::Scalar  mean_t;
    cv::Scalar  stddev_t;
    cv::Mat output_img;
    //
    //cv::GaussianBlur(input_image,input_image,cv::Size(ksize,ksize),ksize/4);
    //
    cv::Laplacian(input_image,output_img,CV_64F,ksize);
    //cv::imshow("Laplacian",output_img);
    //output_img = output_img.mul(output_img);
    cv::meanStdDev(output_img,mean_t,stddev_t);
    output.push_back(stddev_t[0]*stddev_t[0]);// [0]:Variance of Laplacian of pixel values
    //stddev_t = cv::sum(output_img);
    //output.push_back(stddev_t[0]);// [0]:JVvalue (squaresum of all pixel value in an image) of laplacian

    cv::meanStdDev(input_image,mean_t,stddev_t);
    output.push_back(stddev_t[0]);// [1]:std dev of pixel values

    cv::Sobel(input_image,output_img,CV_64F,1,1,ksize);
    //cv::imshow("1st derivative",output_img);
    output_img = output_img.mul(output_img);
    stddev_t = cv::sum(output_img);
    output.push_back(stddev_t[0]);// [2]:JVvalue of 1st derivative

    cv::Canny(input_image,output_img,45000,45050,7);
    //cv::imshow("Canny edge",output_img);
    output_img = output_img.mul(output_img);
    stddev_t = cv::sum(output_img);
    output.push_back(stddev_t[0]);// [3]:JVvalue of found edges
}

cv::Rect Focus_finder::get_rect(const cv::Mat &input_image){
    int center_rows = input_image.rows/2; //Defining the center of the image
    int center_cols = input_image.cols/2;
    cv::Rect regione_interessante(center_cols-(window_size/2),center_rows-(window_size/2),window_size,window_size);
    return regione_interessante;
}

cv::Mat Focus_finder::get_component(const cv::Mat &input_mat,const unsigned int &input){
    cv::Mat bgr[3];   //destination array
    cv::split(input_mat,bgr);//split source
    return bgr[input];
    //Note: OpenCV uses BGR color order
}

void Focus_finder::eval_stddev_ROI(const cv::Mat &input_image, std::vector<double> &output)
{
    //evaluates standard deviation and other quantities of the RoI of the image
    cv::Mat RoiImage  = ( (color_int == -1) ? input_image(get_rect(input_image)) : get_component(input_image(get_rect(input_image)),color_int) );
    eval_stddev(RoiImage,output);
    //    cv::imwrite("EXPORT/Focus_green.png",RoiImage);
    return;
}

bool Focus_finder::find_focus(double &focus_height)
{
    const int figure_index = 1;//Std dev
    //const int figure_index = 0;//Variance of Laplacian

    //Function that return the focus z coordinate
    //https://rechneronline.de/function-graphs/
    //http://doc.qt.io/qt-4.8/signalsandslots.html
    if (!cap.isOpened()){
        qWarning("Error : Not able to open camera.");
        return false;
    }
    if(!gantry->gantryConnected){
        QMessageBox::critical(nullptr, tr("Error"), tr("Gantry not connected!! Can not perform autofocus."));
        return false;
    }
    qInfo("----------------  Auto-focus start -----------------");
    cv::Mat mat_from_outside;
    double z_step[3] = {0.02,0.006,0.002};// mm //to be changed according the units of your gantry and shape of focus-height distribution
    double z_from_outside;

    double StdDev_MAX = 1.1;
    double Z_MAX      = 1.;

    double z_temp = gantry->whereAmI(1).at(z_pos_index);
    qInfo("Performing scan around the position : %5.5f",z_temp);
    //add if() in moveby functions to be sure the code continues when the function returns
    int Iterations = 3;
    for(int j=0; j<Iterations;j++){//fine scan to find the position of the focus
        if(z_pos_index==2){
            if(!gantry->moveZBy(-z_step[j]*ceil(measure_points*0.6),1.))
                return false;
            if(!gantry->WaitZ(-1))
                return false;
        }else if(z_pos_index ==4){
            if(!gantry->moveZ_2_By(-z_step[j]*ceil(measure_points*0.6),1.))
                return false;
            if(!gantry->WaitZ_2(-1))
                return false;
        }
        for(int i=0; i<measure_points;i++){
            QApplication::processEvents();
            if(z_pos_index==2){
                if(!gantry->moveZBy(z_step[j],1.))
                    return false;
                if(!gantry->WaitZ(-1)) //wait for the end of the motion
                    return false;
            }else if(z_pos_index ==4){
                if(!gantry->moveZ_2_By(z_step[j],1.))
                    return false;
                if(!gantry->WaitZ_2(-1))
                    return false;
            }
            mat_from_outside = get_frame_from_camera();
            std::vector<double> figures_of_merit;
            cv::Mat blur_mat;
            //cv::GaussianBlur(mat_from_outside,blur_mat,cv::Size(5,5),2);
            //cv::bilateralFilter(mat_from_outside,blur_mat,ksize,ksize*2,ksize/2);
            //eval_stddev_ROI(blur_mat,figures_of_merit);//gets RoI and proper color component
            eval_stddev_ROI(mat_from_outside,figures_of_merit);//gets RoI and proper color component
            double StdDev_t = figures_of_merit[figure_index];
            z_from_outside = gantry->whereAmI(1).at(z_pos_index);
            std::string file_name = "focus.txt";
            std::ofstream ofs (file_name, std::ofstream::app);
            ofs <<"><> "<< j<<" "<< i<<" "<<gantry->whereAmI(1).at(z_pos_index) <<" "<<
                   figures_of_merit[0]<<" "<<figures_of_merit[1]<<" "<<figures_of_merit[2]<<" "<<figures_of_merit[3]<<std::endl;
            ofs.close();
            if(StdDev_t > StdDev_MAX){
                StdDev_MAX = StdDev_t;
                Z_MAX = z_from_outside;
            }
            //qInfo("i : %i ; z_step : %3.4f ; z : %3.4f ; stddev : %5.5f ; z max : %3.4f ; stddev_MAX: %5.5f",
              //    i,z_step,z_from_outside,StdDev_t,Z_MAX,StdDev_MAX);
            //            if(Iterations==(j-1)){
            //                //this has to be the last iteration
            //                x[i] = z_from_outside;
            //                y[i] = StdDev_t;
            //            }
        }// for 6
        //z_step = 0.3 * z_step;
        if(z_pos_index==2){
            gantry->moveZTo(Z_MAX,1.);
            if(!gantry->WaitZ(-1))
                return false;

        }else if(z_pos_index ==4){
            gantry->moveZ_2_To(Z_MAX,1.);
            if(!gantry->WaitZ_2(-1))
                return false;
        }
        focus_height = Z_MAX;
    }

    qInfo("Focus max z : %3.4f ;  stddev_MAX : %5.5f ",Z_MAX,StdDev_MAX);
    return true;
    /////////////////////
    //evaluation of the focus height ising the fit of a parabola
    //double Z_out = 0.;
    //perform_fit(Z_out);
    //qInfo("Focus fit z : %3.4f",Z_out);
    //focus_height = Z_out;//outputvalue
    //z_temp = Z_out - gantry->whereAmI(1).at(z_pos_index); // relative distance from current position
    //gantry->moveZTo(Z_MAX,1.);//add safety control
    /////////////////////
}

//void Focus_finder::perform_fit(double &z_output)
//{
//    //Currently not used, here for leagcy in case fit is needed in future
//    //WARNING: these array need to be of size == measure_points
//    alglib::real_1d_array al_x = "[0,0,0,0,0,0]";
//    alglib::real_1d_array al_y = "[0,0,0,0,0,0]";
//    for(int i=0;i<measure_points;i++){
//        al_x[i] = x[i];
//        al_y[i] = y[i];
//    }

//    alglib::ae_int_t m = 3;
//    alglib::ae_int_t info;
//    alglib::barycentricinterpolant p;
//    alglib::polynomialfitreport rep;

//    // NOTE: result is returned as barycentricinterpolant structure.
//    //       if you want to get representation in the power basis,
//    //       you can use barycentricbar2pow() function to convert
//    //       from barycentric to power representation (see docs for
//    //       POLINT subpackage for more info).
//    //
//    alglib::polynomialfit(al_x, al_y, m, info, p, rep);

//    alglib::real_1d_array a2;
//    alglib::polynomialbar2pow(p, a2);
//    log->append("Coefficients p_1 = a[0]: "+QString::number(a2[0])+" ;a[1]: "+QString::number(a2[1])+" ; a[2]: "+QString::number(a2[2]));
//    log->append("Vertex : ( "+QString::number(EvalVertex_x(a2[2],a2[1],a2[0]))+" , "+QString::number(EvalVertex_y(a2[2],a2[1],a2[0]))+" )");
//    z_output = EvalVertex_x(a2[2],a2[1],a2[0]);
//}

double Focus_finder::EvalVertex_x(double a,double b, double c){
    //evaluating vertex of parabola. needed for finding focus height when using the fit
    //y = ax^2 + bx + c
    return -b/(2*a);
}

double Focus_finder::EvalVertex_y(double a,double b, double c){
    //evaluating vertex of parabola. needed for finding focus height when using the fit
    //y = ax^2 + bx + c
    double num =0.;
    num = 4*a*c - b*b;
    double den =0.;
    den = 4*a;
    return num/den;
}

bool Focus_finder::Eval_syst_scan(){
    //measure the std-dev several times moving the gantry spanning points around the current position
    int numb_steps = 36;
    double z_focus = gantry->whereAmI(1).at(z_pos_index);
    double z_temp = gantry->whereAmI(1).at(z_pos_index);
    double z_step = 0.003;// to be changed according the units of your gantry and shape of focus-height distribution
    qInfo("Performing systematic scan near the focus position : %5.5f",z_temp);
//    if(z_pos_index==2)
//        gantry->moveZBy(-z_step*numb_steps*0.55,1.);
//    else if(z_pos_index ==4)
//        gantry->moveZ_2_By(-z_step*numb_steps*0.55,1.);
    cv::Mat mat_from_outside;
    for(int side_direction = 0;side_direction<2;side_direction++){
        int direction = (side_direction == 0) ? -1 : 1;
        std::string str_direction = (side_direction == 0) ? "M" : "P";
        for(int i=0; i<numb_steps;i++){
            if(i != 0 && z_pos_index==2){
                if(!gantry->moveZBy(direction*z_step,1.))
                    return false;
                if(!gantry->WaitZ(-1))
                    return false;
            }else if(i != 0 && z_pos_index ==4){
                if(!gantry->moveZ_2_By(direction*z_step,1.))
                    return false;
                if(!gantry->WaitZ_2(-1))
                    return false;
            }
            if (cap.isOpened()){
                mat_from_outside = get_frame_from_camera();
            } else {
                qWarning("Error : Not able to open camera.");
                return false;
            }
            std::vector<double> figures_of_merit;
            eval_stddev_ROI(mat_from_outside,figures_of_merit);
            std::string file_name = "focus.txt";
            std::ofstream ofs (file_name, std::ofstream::app);
            ofs << i<<" "<<gantry->whereAmI(1).at(z_pos_index) <<" "<<
                   figures_of_merit[0]<<" "<<figures_of_merit[1]<<" "<<figures_of_merit[2]<<" "<<figures_of_merit[3]<<std::endl;
            ofs.close();
            std::string time_now_str = "";
            int start_x = 15;
            int start_y = 5;
            std::string s     = std::to_string(i);
            std::vector <double> stuff;
            stuff.clear();
            stuff.push_back(figures_of_merit[1]);
            stuff.push_back(gantry->whereAmI(1).at(z_pos_index));
            cv::Mat RoiImage  = ( (color_int == -1) ? mat_from_outside(get_rect(mat_from_outside)) : get_component(mat_from_outside(get_rect(mat_from_outside)),color_int) );
            addInfo(RoiImage,"F ",start_x,start_y,3,2,time_now_str,stuff);
            cv::imwrite("EXPORT/Focus_"+str_direction+"_"+s+".jpg",RoiImage);
        }
        if(z_pos_index==2){
            if(!gantry->moveZTo(z_focus,2.))
                return false;
            if(!gantry->WaitZ(-1))
                return false;
        }else if(z_pos_index ==4){
            if(!gantry->moveZ_2_To(z_focus,2.))
                return false;
            if(!gantry->WaitZ_2(-1))
                return false;
        }
    }
    return true;
}

void Focus_finder::Eval_syst_time(){
    //measure the std-dev several times without moving the gantry
    double z_temp = gantry->whereAmI().at(z_pos_index);
    log->append("Performing systematic scan in time (2 sec intervals) at position : "
                    +QString::number(z_temp));
    cv::Mat mat_from_outside;
    for(int i=0; i<7;i++){
        Sleeper::sleep(2);
        if (cap.isOpened()){
            mat_from_outside = get_frame_from_camera();
        } else {
            log->append("Error : Not able to open camera.");
            return;
        }
        std::vector<double> figures_of_merit;
        eval_stddev_ROI(mat_from_outside,figures_of_merit);
        std::string file_name = "focus.txt";
        std::ofstream ofs (file_name, std::ofstream::app);
        ofs << i<<" "<<gantry->whereAmI(1).at(z_pos_index) <<" "<<
               figures_of_merit[0]<<" "<<figures_of_merit[1]<<" "<<figures_of_merit[2]<<" "<<figures_of_merit[3]<<std::endl;
        ofs.close();
    }
}


bool Focus_finder::autoFocus(double focus_range)
{

//General function to perform autofocus

bool debug=true;

double currentZ= gantry->whereAmI(1).at(z_pos_index);
double threshold=0.02;
double result=1;
int Nstep=5;
double P0=currentZ-focus_range/2;
double Pn=currentZ+focus_range/2;
double D;
int cont=0;
std::vector<double> FM[];
std::vector<double> Z[];
double Zopt=gantry->whereAmI(1).at(z_pos_index);
double increment=focus_range/m;

if(!gantry->gantryConnected){
    QMessageBox::critical(nullptr, tr("Error"), tr("Gantry not connected!! Can not perform autofocus."));
    return false;
}

if(!gantry->moveZBy(-focus_range/2,1.))
     return false;
if(!gantry->WaitZ(-1))
     return false;

while(result>threshold){
 for(int i=0; i<=Nstep;i++){
  FM[cont]=evalFM();
  Z[cont]=gantry->whereAmI(1).at(z_pos_index);
  cont++;
if(!gantry->moveZBy(increment,1.))
     return false;
if(!gantry->WaitZ(-1))
     return false;
}

FMmax = FM[std::max_element(FM.begin(), FM.end())];
Zmax=Z[std::max_element(FM.begin(), FM.end())];
result=qFabs(Zmax-Zopt);

if(debug) qDebug() << "FM:" << FM << "Z:" << Z << "FMmax:" << FMmax << "Zmax:" << Zmax; 
 
}

Zpeak=evalFit(FM,Z);

if(!gantry->moveZTo(Zpeak,1.))
     return false;
if(!gantry->WaitZ(-1))
     return false;

return true;
}



double Focus_finder::evalFM()
{
bool debug=true;

//evaluating focus parameter 
 
const int figure_index = 1;//Std dev
//const int figure_index = 0;//Variance of Laplacian

cv::Mat image=get_frame_from_camera();
std::vector<double> figures_of_merit;
cv::Mat blur_mat;
eval_stddev_ROI(image,figures_of_merit);//gets RoI and proper color component
double focus_value = figures_of_merit[figure_index];
if(debug) qDebug() << "focus value:" focus value:
return focus_value;
}


double Focus_finder::evalFit(std::vector<double> focus_v,std::vector<double> Z_v)
{

int size=focus_v.size();

alglib::real_1d_array al_x;
alglib::real_1d_array al_y;
for(int i=0;i<size;i++){
    al_x[i] = focus_v[i];
    al_y[i] = Z_v[i];
}
alglib::ae_int_t m = 3;
alglib::ae_int_t info;
alglib::barycentricinterpolant p;
alglib::polynomialfitreport rep;

alglib::polynomialfit(al_x, al_y, m, info, p, rep);

alglib::real_1d_array a2;
alglib::polynomialbar2pow(p, a2);
log->append("Coefficients p_1 = a[0]: "+QString::number(a2[0])+" ;a[1]: "+QString::number(a2[1])+" ; a[2]: "+QString::number(a2[2]));
log->append("Vertex : ( "+QString::number(EvalVertex_x(a2[2],a2[1],a2[0]))+" , "+QString::number(EvalVertex_y(a2[2],a2[1],a2[0]))+" )");
z_output = EvalVertex_x(a2[2],a2[1],a2[0]);

}
