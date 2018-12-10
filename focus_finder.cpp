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
    cv::putText(image,algo_name,cv::Point(start_x,window_size-start_y), CV_FONT_HERSHEY_PLAIN,text_font_size,cv::Scalar(255,255,255),text_thikness);
    cv::Size text_size = cv::getTextSize(algo_name, CV_FONT_HERSHEY_PLAIN,text_font_size,text_thikness,&baseline);
    QTime now = QTime::currentTime();
    QString time_now = now.toString("hhmmss");
    std::string time_now_str = time_now.toLocal8Bit().constData();
    timestamp = time_now_str;
    start_x += text_size.width;
    cv::putText(image,time_now_str,cv::Point(start_x,window_size-start_y), CV_FONT_HERSHEY_PLAIN,text_font_size-1,cv::Scalar(255,255,255),text_thikness);
    cv::Size time_size = cv::getTextSize(time_now_str, CV_FONT_HERSHEY_PLAIN,text_font_size,text_thikness,&baseline);
    start_x += time_size.width;
    for(unsigned int i=0;i<values.size();i++){
        auto s = std::to_string(values.at(i));
        cv::putText(image,s,cv::Point(start_x,window_size-start_y), CV_FONT_HERSHEY_PLAIN,text_font_size-1,cv::Scalar(255,255,255),text_thikness);
        cv::Size s_size = cv::getTextSize(s, CV_FONT_HERSHEY_PLAIN,text_font_size,text_thikness,&baseline);
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
    //cv::GaussianBlur(output_img,output_img,cv::Size(ksize,ksize),ksize/2);
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

void Focus_finder::find_focus(double &focus_height)
{
    const int figure_index = 0;
    //Function that return the focus z coordinate
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
    qInfo("----------------  Auto-focus start -----------------");
    cv::Mat mat_from_outside;
    double z_step = 0.05;// mm //to be changed according the units of your gantry and shape of focus-height distribution
    double z_from_outside;

    double StdDev_MAX = 1.1;
    double Z_MAX      = 1.;

    double z_temp = gantry->whereAmI(1).at(z_pos_index);
    qInfo("Performing scan around the position : %5.5f",z_temp);

    int Iterations = 4;
    for(int j=0; j<Iterations;j++){//fine scan to find the position of the focus
        if(z_pos_index==2)
            gantry->moveZBy(-z_step*ceil(measure_points*0.6),1.);
        else if(z_pos_index ==4)
            gantry->moveZ_2_By(-z_step*ceil(measure_points*0.6),1.);
        for(int i=0; i<measure_points;i++){
            if(z_pos_index==2)
                gantry->moveZBy(z_step,1.);
            else if(z_pos_index ==4)
                gantry->moveZ_2_By(z_step,1.);
            mat_from_outside = get_frame_from_camera();
            std::vector<double> figures_of_merit;
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
            qInfo("i : %i ; z_step : %3.4f ; z : %3.4f ; stddev : %5.5f ; z max : %3.4f ; stddev_MAX: %5.5f",
                  i,z_step,z_from_outside,StdDev_t,Z_MAX,StdDev_MAX);
            //            if(Iterations==(j-1)){
            //                //this has to be the last iteration
            //                x[i] = z_from_outside;
            //                y[i] = StdDev_t;
            //            }
        }// for 6
        z_step = 0.5 * z_step;
        gantry->moveZTo(Z_MAX,1.);//add safety control
        focus_height = Z_MAX;
    }

    qInfo("Focus max z : %3.4f ;  stddev_MAX : %5.5f ",Z_MAX,StdDev_MAX);
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

void Focus_finder::Eval_syst_scan(){
    //measure the std-dev several times moving the gantry spanning points around the current position
    int numb_steps = 30;
    double z_temp = gantry->whereAmI(1).at(z_pos_index);
    double z_step = 0.01;// to be changed according the units of your gantry and shape of focus-heught distribution
    qInfo("Performing systematic scan near the focus position : %5.5f",z_temp);
    if(z_pos_index==2)
        gantry->moveZBy(-z_step*numb_steps*0.6,1.);
    else if(z_pos_index ==4)
        gantry->moveZ_2_By(-z_step*numb_steps*0.6,1.);
    cv::Mat mat_from_outside;
    for(int i=0; i<numb_steps;i++){
        if(z_pos_index==2)
            gantry->moveZBy(z_step,1.);
        else if(z_pos_index ==4)
            gantry->moveZ_2_By(z_step,1.);
        if (cap.isOpened()){
        mat_from_outside = get_frame_from_camera();
        } else {
            qWarning("Error : Not able to open camera.");
            return;
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
        stuff.push_back(figures_of_merit[0]);
        stuff.push_back(gantry->whereAmI(1).at(z_pos_index));
        cv::Mat RoiImage  = ( (color_int == -1) ? mat_from_outside(get_rect(mat_from_outside)) : get_component(mat_from_outside(get_rect(mat_from_outside)),color_int) );
        addInfo(RoiImage,"F ",start_x,start_y,2,2,time_now_str,stuff);
        cv::imwrite("EXPORT/Focus_"+time_now_str+"_"+s+".jpg",RoiImage);
    }
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
