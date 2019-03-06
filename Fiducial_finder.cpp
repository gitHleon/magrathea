#include "Fiducial_finder.h"

void function_ChiSquare(const alglib::real_1d_array &x, double &func, void *ptr)
{
    //
    // this callback calculates chi square function for square of circles
    // We will have 8+4+4 variables, 8 for the points coordinates and 4 for the paremeters we want to fit.
    // Legend of the valiables : x[0], x[1], etc.. => x_1, y_1, x_2, y_2, etc.. coordinates of the points of the image;
    // variables x[8] , ... ,  x[11] are the score of the circles, this are used as weights to give more importance to good circles (i.e. the one with high score)
    // variables x[12], ... ,  x[15] are the fit prameters
    double calibration_value = 10.5; //[px/um]
    //double one_over_sigma_square = 1./(5*calibration_value); //error of 2 um, calibration is 10 px / um and measures are in pixels
    double real_side_size = 50 * calibration_value;
    double real_side_size_err = real_side_size*0.2;
    func =
              pow(x[0]-(x[12]-(x[15]/sqrt(2))*cos(x[14]+M_PI/4)),2)*pow(x[8],2)+pow(x[1]-(x[13]-(x[15]/sqrt(2))*sin(x[14]+M_PI/4)),2)*pow(x[8],2)
            + pow(x[2]-(x[12]-(x[15]/sqrt(2))*cos(M_PI/4-x[14])),2)*pow(x[9],2)+pow(x[3]-(x[13]+(x[15]/sqrt(2))*sin(M_PI/4-x[14])),2)*pow(x[9],2)
            + pow(x[4]-(x[12]+(x[15]/sqrt(2))*cos(M_PI/4+x[14])),2)*pow(x[10],2)+pow(x[5]-(x[13]+(x[15]/sqrt(2))*sin(M_PI/4+x[14])),2)*pow(x[10],2)
            + pow(x[6]-(x[12]+(x[15]/sqrt(2))*cos(M_PI/4-x[14])),2)*pow(x[11],2)+pow(x[7]-(x[13]-(x[15]/sqrt(2))*sin(M_PI/4-x[14])),2)*pow(x[11],2)
            +(pow(x[15]-real_side_size,2)/(real_side_size_err*real_side_size))
            ;
}

bool Distance_sorter(cv::DMatch m_1,cv::DMatch m_2){
    return m_1.distance < m_2.distance;
}

bool Point_sorter(cv::Point2d m_1,cv::Point2d m_2){
    return m_1.x < m_2.x;
}

std::string type2str(int type);

FiducialFinder::FiducialFinder(QWidget *parent) : QWidget(parent)
{}

FiducialFinder::~FiducialFinder()
{}

void FiducialFinder::SetImage(const cv::Mat &input){
    image = input.clone();
}

void FiducialFinder::SetImage(const std::string& filename, int flags){
    image = cv::imread( filename, flags);
}

void FiducialFinder::SetImageFiducial(const cv::Mat &input){
    image_fiducial = input.clone();
}

void FiducialFinder::SetImageFiducial(const std::string& filename, int flags){
    image_fiducial = cv::imread( filename, flags);
}

bool FiducialFinder::IsImageEmpty(){
    return image.empty();
}

void FiducialFinder::Set_log(QTextEdit *m_log){
    log = m_log;
}

void FiducialFinder::Set_calibration(double m_calib){
    Calibration = m_calib;
}

cv::Mat FiducialFinder::get_component(const cv::Mat &input_mat,const unsigned int &input){
    cv::Mat bgr[3];   //destination array
    cv::split(input_mat,bgr);//split source
    return bgr[input];
    //Note: OpenCV uses BGR color order
}

cv::Mat FiducialFinder::change_gamma(const cv::Mat &input_mat, const double &gamma){
    if(gamma < 0 || gamma > 255){
        std::cout<<"enance_contrast: Error in gamma range"<<std::endl;
        return input_mat;
    }
    cv::Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for( int i = 0; i < 256; ++i)
        p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);

    cv::Mat output = input_mat.clone();
    cv::LUT(input_mat, lookUpTable, output);

    return output;
}

cv::Mat FiducialFinder::enance_contrast(const cv::Mat &input_mat, const double &alpha, const double &beta = 0){
    if(alpha < 0 || alpha > 3){
        std::cout<<"enance_contrast: Error in alpha range"<<std::endl;
        return input_mat;
    }
    cv::Mat output;
    cv::Mat betas = cv::Mat::ones(input_mat.rows,input_mat.cols,input_mat.type());
    betas = betas*beta;
    output = input_mat.clone();
    output = output*alpha + betas;
    return output;
}

cv::Mat FiducialFinder::dan_contrast(const cv::Mat &input_mat, const double &max_alpha){
    if(max_alpha < 0 || max_alpha > 3){
        std::cout<<"dan_contrast: Error in alpha range"<<std::endl;
        return input_mat;
    }
    cv::Mat output;
    cv::Scalar  mean_t;
    cv::Scalar  stddev_t;
    cv::meanStdDev(input_mat,mean_t,stddev_t);
    const int steps = 5;
    double alpha_steps = max_alpha/(steps-2);
    bool debug = false;

    std::vector<cv::Mat> matrices;
    std::vector<cv::Mat> matrices_mod;
    std::vector<double> threshold_steps;
    threshold_steps.push_back(0);
    threshold_steps.push_back(mean_t[0]-1*stddev_t[0]);
    threshold_steps.push_back(mean_t[0]+1*stddev_t[0]);
    threshold_steps.push_back(mean_t[0]+2*stddev_t[0]);

    //    for(int i=1;i<(steps-1);i++)
    //        threshold_steps.push_back(i*threshold_step);
    threshold_steps.push_back(255);

    matrices.push_back(input_mat);
    for(unsigned int i=1;i<(steps-1);i++){
        cv::Mat temp_thr;
        cv::threshold(input_mat,temp_thr,threshold_steps.at(i),255,cv::THRESH_TOZERO);
        matrices.push_back(temp_thr);
    }
    matrices.push_back(cv::Mat::zeros(input_mat.rows,input_mat.cols,input_mat.type()));

    for(unsigned int i=0;i<(steps-1);i++){
        cv::Mat temp_thr;
        temp_thr = matrices.at(i) - matrices.at(i+1);
        double square_step = i/(steps-2);
        std::cout<<" value "<< square_step<<" square "<<pow(square_step,2.)<<std::endl;
        matrices_mod.push_back(enance_contrast(temp_thr,alpha_steps*pow(square_step,2)));
        //matrices_mod.push_back(enance_contrast(temp_thr,alpha_steps*sqrt(square_step)));
    }

    if(debug){
        cv::imshow("test 1",matrices_mod.at(0));
        cv::imshow("test 2",matrices_mod.at(1));
        cv::imshow("test 3",matrices_mod.at(2));
        cv::imshow("test 4",matrices_mod.at(3));
    }

    output = matrices_mod.at(0).clone();
    for(unsigned int i=1;i<matrices_mod.size();i++){
        output = output + matrices_mod.at(i);
    }
    return output;
}


void FiducialFinder::addInfo(cv::Mat &image,const std::string &algo_name, int start_x, int start_y,int text_font_size ,int text_thikness,std::string &timestamp){
    int baseline = 0;
    //int text_font_size = 2;
    //int text_thikness = 2;

    int window_size = image.rows;
    std::string um_str = " 50um";
    cv::putText(image,algo_name,cv::Point(start_x,window_size-start_y), cv::FONT_HERSHEY_PLAIN,text_font_size,cv::Scalar(255,255,255),text_thikness);
    cv::Size text_size = cv::getTextSize(algo_name, cv::FONT_HERSHEY_PLAIN,text_font_size,text_thikness,&baseline);
    QTime now = QTime::currentTime();
    QString time_now = now.toString("hhmmss");
    std::string time_now_str = time_now.toLocal8Bit().constData();
    timestamp = time_now_str;
    start_x += text_size.width;
    cv::putText(image,time_now_str,cv::Point(start_x,window_size-start_y), cv::FONT_HERSHEY_PLAIN,text_font_size-1,cv::Scalar(255,255,255),text_thikness);
    cv::Size time_size = cv::getTextSize(time_now_str, cv::FONT_HERSHEY_PLAIN,text_font_size-1,text_thikness,&baseline);
    start_x += time_size.width;
    cv::putText(image,um_str,cv::Point(start_x,window_size-start_y), cv::FONT_HERSHEY_PLAIN,text_font_size-1,cv::Scalar(255,255,255),text_thikness);
    cv::Size um_size = cv::getTextSize(um_str, cv::FONT_HERSHEY_PLAIN,text_font_size-1,text_thikness,&baseline);
    start_x += um_size.width;
    cv::line(image,cv::Point(start_x,window_size-start_y),cv::Point(start_x+(50*Calibration),window_size-start_y),cv::Scalar(255,255,255),text_thikness);
}

bool FiducialFinder::Is_equal(const double &one, const double &two){
    //function needed when searching for fiducial not using SURF
    //Tolerance 3um, the precision of the gantry
    double tolerance = 10*Calibration; //[px]
    return ( fabs(one-two) <= tolerance);
}

bool FiducialFinder::Is_a_triangle(const cv::Point &P_1, const cv::Point &P_2, const cv::Point &P_3){
    //function needed when searching for fiducial not using SURF
    //http://en.cppreference.com/w/cpp/language/range-for
    //https://softwareengineering.stackexchange.com/questions/176938/how-to-check-if-4-points-form-a-square
    //Distances between points are 22 - 20 um
    const double threshold_H = 27*Calibration;
    const double threshold_L = 15*Calibration;
    cv::Vec2i p_1 = {P_1.x,P_1.y};
    cv::Vec2i p_2 = {P_2.x,P_2.y};
    cv::Vec2i p_3 = {P_3.x,P_3.y};

    if(Is_equal(cv::norm(p_1,p_2),cv::norm(p_2,p_3)) &&
            Is_equal(cv::norm(p_2,p_3),cv::norm(p_3,p_1)) &&
            Is_equal(cv::norm(p_3,p_1),cv::norm(p_1,p_2)) &&
            (cv::norm(p_1,p_3) < threshold_H) &&
            (cv::norm(p_1,p_3) > threshold_L))
    {
        return true;
    }else{
        return false;
    }
}

void FiducialFinder::Find_SquareAndTriangles(const std::vector<cv::Vec4f> &Circles, std::vector<std::vector<unsigned int> > &Squares, std::vector<std::vector<unsigned int> > &Triangles){
    //function needed when searching for fiducial not using SURF
    int Iteration = 0;
    Squares.clear();
    Triangles.clear();
    unsigned int num_points = Circles.size();
    for(unsigned int i=0;i<num_points;i++){
        for(unsigned int j=i+1;j<num_points;j++){
            for(unsigned int k=j+1;k<num_points;k++){
                cv::Point point_i = cv::Point(Circles[i][0],Circles[i][1]);
                cv::Point point_j = cv::Point(Circles[j][0],Circles[j][1]);
                cv::Point point_k = cv::Point(Circles[k][0],Circles[k][1]);
                if(Is_a_triangle(point_i,point_j,point_k)){
                    std::cout<<">> Tria True <<"<<std::endl;
                    std::vector <unsigned int> indeces (3);
                    indeces.at(0) = i;
                    indeces.at(1) = j;
                    indeces.at(2) = k;
                    Triangles.push_back(indeces);
                }
                for(unsigned int l=k+1;l<num_points;l++){
                    cv::Point point_l = cv::Point(Circles[l][0],Circles[l][1]);
                    Iteration++;
                    if(Is_a_square(point_i,point_j,point_k,point_l)){
                        std::cout<<">> True <<"<<std::endl;
                        std::vector <unsigned int> indeces (4);
                        indeces.at(0) = i;
                        indeces.at(1) = j;
                        indeces.at(2) = k;
                        indeces.at(3) = l;
                        Squares.push_back(indeces);
                    }
                }
            }
        }
    }
//    std::cout<<"Iterations  : "<<Iteration<<std::endl;
//    std::cout<<"# Squares   : "<<Squares.size()<<std::endl;
//    std::cout<<"# Triangles : "<<Triangles.size()<<std::endl;
}

bool FiducialFinder::Is_a_square(const cv::Point &P_1, const cv::Point &P_2, const cv::Point &P_3, const cv::Point &P_4)
{
    bool debug = false;
    //function needed when searching for fiducial not using SURF
    //http://en.cppreference.com/w/cpp/language/range-for
    //https://softwareengineering.stackexchange.com/questions/176938/how-to-check-if-4-points-form-a-square
    //Distances between points are 50 um. threshold is on the diagonal i.e. 50*1.4
    const int threshold_H = 85*Calibration;
    const int threshold_L = 55*Calibration;
    cv::Vec2i p_1 = {P_1.x,P_1.y};
    cv::Vec2i p_2 = {P_2.x,P_2.y};
    cv::Vec2i p_3 = {P_3.x,P_3.y};
    cv::Vec2i p_4 = {P_4.x,P_4.y};
    enum Diagonals {NONE, P1P2 , P1P3 , P1P4};
    Diagonals diagonal = NONE;
    if(debug){
        std::cout<< "cv::norm(p_1,p_2)" << cv::norm(p_1,p_2)<< std::endl;
        std::cout<< "cv::norm(p_1,p_3)" << cv::norm(p_1,p_3)<< std::endl;
        std::cout<< "cv::norm(p_1,p_4)" << cv::norm(p_1,p_4)<< std::endl;
        std::cout<< "cv::norm(p_2,p_3)" << cv::norm(p_2,p_3)<< std::endl;
        std::cout<< "cv::norm(p_2,p_4)" << cv::norm(p_2,p_4)<< std::endl;
        std::cout<< "cv::norm(p_3,p_4)" << cv::norm(p_3,p_4)<< std::endl;
    }
    //searching for two equal diagonals
    //Diagonal is longer than the sides ;)
    if(cv::norm(p_1,p_2) < cv::norm(p_1,p_3)){
        if(cv::norm(p_1,p_3) < cv::norm(p_1,p_4))
            diagonal = P1P4;
        else
            diagonal = P1P3;
    } else {
        if(cv::norm(p_1,p_2) < cv::norm(p_1,p_4))
            diagonal = P1P4;
        else
            diagonal = P1P2;
    }

    if(diagonal == P1P2 && Is_equal(cv::norm(p_1,p_2),cv::norm(p_3,p_4))){
        //If diagonals are equal, all the sides needs to be equal
        if(
                Is_equal(cv::norm(p_1,p_3),cv::norm(p_3,p_2)) &&
                Is_equal(cv::norm(p_3,p_2),cv::norm(p_2,p_4)) &&
                Is_equal(cv::norm(p_2,p_4),cv::norm(p_4,p_1)) &&
                Is_equal(cv::norm(p_4,p_1),cv::norm(p_1,p_3)) &&
                (cv::norm(p_1,p_2) < threshold_H) &&
                (cv::norm(p_1,p_2) > threshold_L))
                {
            return true;
        }
    }else if(diagonal == P1P3 && Is_equal(cv::norm(p_1,p_3),cv::norm(p_2,p_4))){
        //If diagonals are equal, all the sides needs to be equal
        if(
                Is_equal(cv::norm(p_1,p_2),cv::norm(p_2,p_3)) &&
                Is_equal(cv::norm(p_2,p_3),cv::norm(p_3,p_4)) &&
                Is_equal(cv::norm(p_3,p_4),cv::norm(p_4,p_1)) &&
                Is_equal(cv::norm(p_4,p_1),cv::norm(p_1,p_2)) &&
                (cv::norm(p_1,p_3) < threshold_H) &&
                (cv::norm(p_1,p_3) > threshold_L))
                {
            return true;
        }
    }else if(diagonal == P1P4 && Is_equal(cv::norm(p_1,p_4),cv::norm(p_2,p_3))){
        //If diagonals are equal, all the sides needs to be equal
        if(
                Is_equal(cv::norm(p_1,p_2),cv::norm(p_2,p_4)) &&
                Is_equal(cv::norm(p_2,p_4),cv::norm(p_4,p_3)) &&
                Is_equal(cv::norm(p_4,p_3),cv::norm(p_3,p_1)) &&
                Is_equal(cv::norm(p_3,p_1),cv::norm(p_1,p_2)) &&
                (cv::norm(p_1,p_4) < threshold_H) &&
                (cv::norm(p_1,p_4) > threshold_L))
                {
            return true;
        }
    }else{
        return false;
    }
    //if you reach here, it is not a square...
    return false;
}

cv::Point FiducialFinder::Square_center(const cv::Point &P_1, const cv::Point &P_2, const cv::Point &P_3, const cv::Point &P_4){
    float X_coord = P_1.x + P_2.x + P_3.x + P_4.x;
    X_coord /= 4;
    float Y_coord = P_1.y + P_2.y + P_3.y + P_4.y;
    Y_coord /= 4;
    cv::Point Out;
    Out.x = X_coord;
    Out.y = Y_coord;
    return Out;
}

bool FiducialFinder::Find_circles(double &X_distance, double &Y_distance,const int &input_1, const int &input_2, bool fit){
    //function needed when searching for fiducial not using SURF
    //to find the 4 dot fiducial
    bool debug = false;
    bool print_raw = true;


    if(image.empty()){
        log->append("Error!! Image is empty!!");
        return false;}

        int center_rows = image.rows/2; //Defining the center of the image
        int center_cols = image.cols/2;

        if(debug)
            cv::imshow("f. 0 image",image);
        const int window_size = ( (image.cols > 2600 && image.rows > 2600) ? 2600 : 420);
        const int kernel_size = ( (image.cols > 2000 && image.rows > 2000) ? 15 : 5);
        if(window_size >= image.rows || window_size >= image.cols){
            log->append("Error!! Window size wrongly set!!");
            return false;}

        cv::Rect regione_interessante(center_cols-(window_size/2),center_rows-(window_size/2),window_size,window_size); //Rectangle that will be the RegionOfInterest (ROI)
        cv::Mat RoiImage = image(regione_interessante);
        if(debug)
            cv::imshow("f. 0.1 image ROI",RoiImage);

        cv::Mat output_mat = RoiImage.clone(); // Selecting ROI from the input image
        output_mat = get_component(output_mat,1);

        for(int iterations = 0;;iterations++){
            cv::Mat image_gray = output_mat.clone();
            if(iterations==1){
                //image_gray = enance_contrast(image_gray,1.5);
                image_gray = dan_contrast(image_gray,2);
            }else if(iterations==2){
                image_gray = change_gamma(image_gray,3);
            }else if(iterations==3){
                image_gray = dan_contrast(image_gray,2);
                image_gray = change_gamma(image_gray,3);
            }

            if(debug){
                cv::imshow("contrast",image_gray);
                std::cout<<" iteration : "<<iterations<<std::endl;
            }
            for(int i=0;i<1;i++){
                //https://docs.opencv.org/3.4/d3/d8f/samples_2cpp_2tutorial_code_2ImgProc_2Smoothing_2Smoothing_8cpp-example.html#a12
                cv::medianBlur(image_gray,image_gray,kernel_size);
            }

            if(debug)
                cv::imshow("1 blur",image_gray);

            cv::threshold(image_gray,image_gray,0,255,cv::THRESH_BINARY | cv::THRESH_OTSU );
            if(debug)
                cv::imshow("1.1 blur+thr",image_gray);

            cv::Mat StructElement = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(kernel_size,kernel_size));
            cv::morphologyEx(image_gray,image_gray,cv::MORPH_CLOSE,StructElement);
            if(debug)
                cv::imshow("1.2 blur+thr+close",image_gray);

            cv::adaptiveThreshold(image_gray,image_gray,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY_INV,kernel_size,2); //CV_THRESH_BINARY

            if(debug)
                cv::imshow("2 threshold",image_gray);

            if(debug){
                std::string ty =  type2str( image_gray.type() );
                std::cout<< ty<<std::endl;
            }
            //------
            //return true;
            //Size of dot: diameter = 20 um
            //double Calibration; //[px/um]
            int min_radius = 20*0.5*Calibration; //[px] //14 for SCToptics
            int max_radius = 30*0.5*Calibration; //[px] //30 for SCToptics
            int minDist = min_radius*4; //[px]
            double correction_factor = 0.4; //0.2 for SCToptics
            int hough_threshold = min_radius*correction_factor; //[px]
            if(debug)
                std::cout<<">> calibration "<<Calibration
                        <<" min_radius "<<min_radius<<
                          " max_radius "<<max_radius<<
                          " hough_threshold "<<hough_threshold<<std::endl;
            std::vector<cv::Vec4f> circles;
            circles.clear();

            //    for(int iterations = 0;;iterations++){
            //        if(iterations!=0){
            //            image_gray = enance_contrast(image_gray,2.);
            //            if(debug)
            //                cv::imshow("contrast",image_gray);
            //        }
        cv::HoughCircles(image_gray, circles, cv::HOUGH_GRADIENT, 1, minDist, 150, hough_threshold, min_radius, max_radius);
        if(debug)
            std::cout<<">> circles "<<circles.size()<<std::endl;
        std::vector <cv::Point2f> Centers (circles.size());
        cv::Mat RoiImage_out     = RoiImage.clone();
        cv::Mat RoiImage_out_fit = RoiImage.clone();
        for( size_t i = 0; i < circles.size(); i++ ){
            Centers[i].x = circles[i][0];
            Centers[i].y = circles[i][1];
            cv::Point2f center(circles[i][0], circles[i][1]);
            auto radius = circles[i][2];
            auto vote   = circles[i][3];
            if(debug)
                std::cout<<" radius "<<radius<<" CX "<<Centers[i].x<<" CY "<<Centers[i].y<< " vote : "<<vote<<std::endl;
            // circle center
            cv::circle(RoiImage_out, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
            cv::circle(RoiImage_out_fit, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
            // circle outline
            cv::circle(RoiImage_out, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
            cv::circle(RoiImage_out_fit, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
        }
        if(debug)
            for( size_t i = 0; i < circles.size(); i++ ){
                std::cout<<" CX "<<circles[i][0]<<" CY "<<circles[i][1]<<std::endl;
                cv::Point point_A = cv::Point2f(circles[i][0],circles[i][1]);
                cv::Point point_B = cv::Point2f(circles[(i+1)%Centers.size()][0],circles[(i+1)%Centers.size()][1]);
                std::cout<<" OLD norm "<<cv::norm(Centers.at(i)-Centers.at((i+1)%Centers.size())) <<" CXX "<<Centers[(i+1)%4].x<<" CYY "<<Centers[(i+1)%4].y<<std::endl;
                std::cout<<" NEW norm "<<cv::norm(point_A-point_B) <<" CXX "<<point_B.x<<" CYY "<<point_B.y<<std::endl;
            }

        std::vector <std::vector <unsigned int> > Squares(0);
        std::vector <std::vector <unsigned int> > Triangles(0);

        Find_SquareAndTriangles(circles,Squares,Triangles);
        if(debug)
            std::cout<<"Squares.size() "<<Squares.size()<<std::endl;
        //cv::circle(RoiImage_out, cv::Point(center_cols,center_rows), 3, cv::Scalar(0,0,255), -1, 8, 0 );
        //define square center variables, to be used later in the fit
        double square_center_x = 0.;
        double square_center_y = 0.;
        for( size_t i = 0; i < Squares.size(); i++ ){
            for( unsigned int j = 0; j < 4; j++ ){
                cv::Point point_A = cv::Point2f(circles[Squares[i][j]][0],circles[Squares[i][j]][1]);
                cv::Point point_B = cv::Point2f(circles[Squares[i][(j+1)%4]][0],circles[Squares[i][(j+1)%4]][1]);
                cv::line(RoiImage_out, point_A, point_B, cv::Scalar(0,255,0), 2, 8);
            }

            //            cv::Point square_center = Square_center(Centers.at(Squares.at(i).at(0)),Centers.at(Squares.at(i).at(1)),
            //                                                    Centers.at(Squares.at(i).at(2)),Centers.at(Squares.at(i).at(3)));
            cv::Point square_center = Square_center(cv::Point2f(circles[Squares.at(i).at(0)][0],circles[Squares.at(i).at(0)][1]),
                    cv::Point2f(circles[Squares.at(i).at(1)][0],circles[Squares.at(i).at(1)][1]),
                    cv::Point2f(circles[Squares.at(i).at(2)][0],circles[Squares.at(i).at(2)][1]),
                    cv::Point2f(circles[Squares.at(i).at(3)][0],circles[Squares.at(i).at(3)][1]) );
            cv::circle(RoiImage_out, square_center, 3, cv::Scalar(255,0,0), -1, 8, 0 );
            cv::circle(RoiImage_out, square_center, 50*Calibration, cv::Scalar(255,0,0), 3, 8, 0 );
            if(Squares.size() == 1){
                X_distance = (square_center.x - RoiImage_out.cols/2)*(1./Calibration); //[um]
                Y_distance = (square_center.y - RoiImage_out.rows/2)*(1./Calibration); //[um]
                cv::circle(RoiImage_out, cv::Point(RoiImage_out.cols/2,RoiImage_out.rows/2), 3, cv::Scalar(0,0,255), -1, 8, 0 );
                if(print_raw){
                    std::string file_name = "output_raw.txt";
                    std::ofstream ofs (file_name, std::ofstream::app);
                    ofs << input_1 <<" "<<input_2<<" "<< square_center.x <<" "<<square_center.y<<std::endl;
                    ofs.close();
                }
                std::cout<<"pre fit : "<< square_center.x <<" "<<square_center.y<<std::endl;
                square_center_x = square_center.x;
                square_center_y = square_center.y;
            }
        }

        if(debug)
            cv::imshow("3 Results",RoiImage_out);
        std::string dummy = std::to_string(iterations);
        std::string one   = std::to_string(input_1);
        std::string two   = std::to_string(input_2);
        cv::imwrite("EXPORT/Circles_ORIGINAL_"+one+"_"+two+"_"+dummy+".jpg",image);
        cv::imwrite("EXPORT/Circles_"+one+"_"+two+"_"+dummy+".jpg",RoiImage_out);
        if(!fit && Squares.size() == 1){
            return true;
        }else if(fit && Squares.size() == 1){

            //1.reorder the points of the square
            std::vector<cv::Vec4d > input;
            std::vector<cv::Vec4d> output;
            input.clear();
            for(unsigned int j = 0; j < 4; j++ )
                input.push_back(circles[Squares.at(0).at(j)]);
            output = OrderSquare(input);
            for(unsigned int j = 0; j < 4; j++ )
            {
                std::cout<<output[j][0]<<" "<<output[j][1]<<" "<<output[(j+1)%4][0]<<" "<<output[(j+1)%4][1]<<" "<<std::endl;
            }
            //2.minimize the chi square
            //http://www.alglib.net/translator/man/manual.cpp.html#example_minbleic_d_2
            alglib::real_1d_array starting_value_variables ;//starting point to be set for all 12 variables
            alglib::real_2d_array limiting_conditions ;//= "[[1,0,2],[1,1,6]]";//limiting conditions for all 12 variables
            alglib::integer_1d_array conditions_relation ;//= "[1,1]";//limiting operator for conditions for all 12 var.
            alglib::real_1d_array variables_range ;
            //define theta and L variable to used in the fit
            double theta = 0.01;
            double side = 50*Calibration; //side is approximately 50 um
            double x_1[] = {output[0][0],output[0][1],
                            output[1][0],output[1][1],
                            output[2][0],output[2][1],
                            output[3][0],output[3][1],
                            output[0][3],
                            output[1][3],
                            output[2][3],
                            output[3][3],
                            square_center_x,
                            square_center_y,
                            theta,
                            side};
            starting_value_variables.setcontent(16,x_1);
            printf("%s\n", starting_value_variables.tostring(16).c_str());
            //setting the boundry condition for the fit
            //first 8 conditions are the coordinates of the centers of the circles
            //for sysntaxis look here:
            //http://www.alglib.net/translator/man/manual.cpp.html#example_minbleic_d_2
            double c_1[] = {
                1,0,0,0,0,  0,0,0,0,0, 0,0,0,0,0, 0, output[0][0],
                0,1,0,0,0,  0,0,0,0,0, 0,0,0,0,0, 0, output[0][1],
                0,0,1,0,0,  0,0,0,0,0, 0,0,0,0,0, 0, output[1][0],
                0,0,0,1,0,  0,0,0,0,0, 0,0,0,0,0, 0, output[1][1],
                0,0,0,0,1,  0,0,0,0,0, 0,0,0,0,0, 0, output[2][0],

                0,0,0,0,0,  1,0,0,0,0, 0,0,0,0,0, 0, output[2][1],
                0,0,0,0,0,  0,1,0,0,0, 0,0,0,0,0, 0, output[3][0],
                0,0,0,0,0,  0,0,1,0,0, 0,0,0,0,0, 0, output[3][1],
                0,0,0,0,0,  0,0,0,1,0, 0,0,0,0,0, 0, output[0][3],
                0,0,0,0,0,  0,0,0,0,1, 0,0,0,0,0, 0, output[1][3],

                0,0,0,0,0,  0,0,0,0,0, 1,0,0,0,0, 0, output[2][3],
                0,0,0,0,0,  0,0,0,0,0, 0,1,0,0,0, 0, output[3][3],
            };
            limiting_conditions.setcontent(12,17,c_1);

            int ct_1[] = {
                0,0,0,0,0,  0,0,0,0,0, 0,0
            };
            conditions_relation.setcontent(12,ct_1);

            double s_1[] = {
                1*Calibration,
                1*Calibration,
                1*Calibration,
                1*Calibration,
                1*Calibration,

                1*Calibration,
                1*Calibration,
                1*Calibration,

                1*Calibration,
                1*Calibration,
                1*Calibration,
                1*Calibration,

                1*Calibration,
                1*Calibration,
                theta*0.1,
                side*0.1};
            variables_range.setcontent(16,s_1);

            //use setcontent function to fill the arrays properly
            //http://www.alglib.net/translator/man/manual.cpp.html#gs_datatypes
            //
            // These variables define stopping conditions for the optimizer.
            //
            // We use very simple condition - |g|<=epsg
            //
            double diffstep = 1.0e-6;
            double epsg = 0.000001;
            double epsf = 0;
            double epsx = 0;
            alglib::ae_int_t maxits = 0;

            //
            // Now we are ready to actually optimize something:
            // * first we create optimizer
            // * we add linear constraints
            // * we tune stopping conditions
            // * and, finally, optimize and obtain results...
            //

            alglib::minbleicstate state;
            alglib::minbleicreport rep;
            minbleiccreatef(starting_value_variables, diffstep,state);
            minbleicsetlc(state, limiting_conditions, conditions_relation);
            minbleicsetcond(state, epsg, epsf, epsx, maxits);
            minbleicsetscale(state,variables_range);
            alglib::minbleicoptimize(state, function_ChiSquare);
            minbleicresults(state, starting_value_variables, rep);
            //
            // ...and evaluate these results
            //
            printf("%d\n", int(rep.terminationtype));
            printf("%s\n", starting_value_variables.tostring(16).c_str());
            //2.1 check that the convergence is good
            if (int(rep.terminationtype) < 0){
                qWarning("fit of square failed!!");
                return false;
            }
            //3 evaluate the center of the new square

//            double fitted_square_center_x = 0.;
//            double fitted_square_center_y = 0.;
            double fitted_square_center_x = starting_value_variables[12];
            double fitted_square_center_y = starting_value_variables[13];
            std::cout<<" "<<fitted_square_center_x<<" "<<fitted_square_center_y<<std::endl;
            if(print_raw){
                std::string file_name = "output_raw_POSTFIT.txt";
                std::ofstream ofs (file_name, std::ofstream::app);
                ofs << input_1 <<" "<<input_2<<" "<< fitted_square_center_x <<" "<<fitted_square_center_y<<std::endl;
                ofs.close();
            }

            X_distance = (fitted_square_center_x - RoiImage_out.cols/2)*(1./Calibration); //[um]
            Y_distance = (fitted_square_center_y - RoiImage_out.rows/2)*(1./Calibration); //[um]

            //0.5 added for rounding when converting from double to int
            cv::circle(RoiImage_out_fit, cv::Point(square_center_x+0.5,square_center_y+0.5), 3, cv::Scalar(255,0,0), -1, 8, 0 );
            cv::circle(RoiImage_out_fit, cv::Point(square_center_x+0.5,square_center_y+0.5), 5*Calibration, cv::Scalar(255,0,0), 2, 8, 0 );
            cv::circle(RoiImage_out_fit, cv::Point(fitted_square_center_x+0.5,fitted_square_center_y+0.5), 3, cv::Scalar(0,0,255), -1, 8, 0 );
            cv::imwrite("EXPORT/Circles_FIT_"+one+"_"+two+"_"+dummy+".jpg",RoiImage_out_fit);
            return true;
        }

        if(iterations>2)
            break;
        }
        return false;
}

std::vector<cv::Vec4d>  FiducialFinder::OrderSquare(const std::vector<cv::Vec4d> &input){
    std::vector<cv::Vec4d>  output(4);
    //I am assuming it is a square
    if(input.size()!=4){
        qWarning("Error in square reordering size.");
        return output;
    }
    double X_coord = input[0][0] + input[1][0] + input[2][0] + input[3][0];
    X_coord /= 4;
    double Y_coord = input[0][1] + input[1][1] + input[2][1] + input[3][1];
    Y_coord /= 4;

    cv::Vec2d p_0 = {input[0][0],input[0][1]};
    cv::Vec2d p_1 = {input[1][0],input[1][1]};
    cv::Vec2d p_2 = {input[2][0],input[2][1]};
    cv::Vec2d p_3 = {input[3][0],input[3][1]};
    for(unsigned int i=0;i<4;i++){
        if(input[i][0] < X_coord && input[i][1]<Y_coord)
            output.at(0) = input.at(i);
        else if(input[i][0] < X_coord && input[i][1]>Y_coord)
            output.at(1) = input.at(i);
        else if (input[i][0] > X_coord && input[i][1]>Y_coord)
            output.at(2) = input.at(i);
        else if(input[i][0] > X_coord && input[i][1]<Y_coord)
            output.at(3) = input.at(i);
    }
    if(cv::norm(p_0,p_1) >= cv::norm(p_0,p_2))
        if(cv::norm(p_0,p_1) >= cv::norm(p_0,p_3)){
            output.push_back(input[2]);
            output.push_back(input[1]);//this is the opposite corner
            output.push_back(input[3]);
        }else{
            output.push_back(input[2]);
            output.push_back(input[3]);//this is the opposite corner
            output.push_back(input[1]);
        }
    else
        if(cv::norm(p_0,p_2) >= cv::norm(p_0,p_3)){
            output.push_back(input[1]);
            output.push_back(input[2]);//this is the opposite corner
            output.push_back(input[3]);
        }else{
            output.push_back(input[1]);
            output.push_back(input[3]);//this is the opposite corner
            output.push_back(input[2]);
        }
    return output;
}


bool FiducialFinder::Find_F(const int &DescriptorAlgorithm, double &X_distance, double &Y_distance,
                            std::string &timestamp,
                            int &fail_code,
                            const int &input_1, const int &input_2, const int &input_3,
                            cv::Mat &transform_out){
    //main function for finding fiducials
    //https://gitlab.cern.ch/guescini/fiducialFinder/blob/master/fiducialFinder.py

    bool debug = false;
    fail_code = 0;

    if(image.empty()){
        log->append("Error!! Image is empty!!");
        return false;}
    if(image_fiducial.empty()){
        log->append("Error!! Fiducial is empty!!");
        return false;}

    int center_rows = image.rows/2; //Defining the center of the image
    int center_cols = image.cols/2;
    if(debug)
        cv::imshow("f. 0 image",image);
    const int window_size = ( (image.cols > 2000 && image.rows > 2000) ? 2000 : 420);
    const int kernel_size = ( (image.cols > 2000 && image.rows > 2000) ? 15 : 5);
    if(window_size >= image.rows || window_size >= image.cols){
        log->append("Error!! Window size wrongly set!!");
        return false;}

    cv::Rect regione_interessante(center_cols-(window_size/2),center_rows-(window_size/2),window_size,window_size); //Rectangle that will be the RegionOfInterest (ROI)
    cv::Mat RoiImage = image(regione_interessante);
    if(debug)
        cv::imshow("f. 0.1 image ROI",RoiImage);
    if(debug)
        cv::imshow("f. 0.1.f image ROI",image_fiducial);

    cv::Mat image_gray   = RoiImage.clone(); // Selecting ROI from the input image
    cv::Mat image_F_gray = image_fiducial.clone(); // Selecting ROI from the input image

    image_gray = get_component(image_gray,1);
    image_F_gray = get_component(image_F_gray,1);
    cv::Mat output_mat = image_gray.clone();

    //////////////////////////////////////////////
    /// NOISE SUPPRESSION VIA FOURIER TRANSFORM - FAILED
    //////////////////////////////////////////////
    //        cv::Mat image_gray_float;
    //        image_gray.convertTo(image_gray_float,CV_32FC1, 1.0 / 255.0);
    //        cv::Mat planes[] = {image_gray_float, cv::Mat::zeros(image_gray_float.size(), CV_32F)};
    //        cv::Mat complexI;
    //        cv::merge(planes, 2, complexI);         // Add another plane with zeros

    //        if(debug)
    //            std::cout<<"complexI.rows "<<complexI.rows<<" ;complexI.cols "<<complexI.cols<<std::endl;

    //        cv::Mat dftOfOriginal;
    //        cv::dft(complexI, dftOfOriginal, cv::DFT_COMPLEX_OUTPUT);// Fourier transform

    //        cv::split(dftOfOriginal, planes);// planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
    //        cv::Mat magI = planes[0].clone(); //We need only the real part, not the phase of the transform
    //        magI += cv::Scalar::all(1);                    // switch to logarithmic scale
    //        cv::log(magI, magI);
    //        // rearrange the quadrants of Fourier image  so that the origin is at the image center
    //        int cx = magI.cols/2;
    //        int cy = magI.rows/2;

    //        cv::Mat q0(magI, cv::Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
    //        cv::Mat q1(magI, cv::Rect(cx, 0, cx, cy));  // Top-Right
    //        cv::Mat q2(magI, cv::Rect(0, cy, cx, cy));  // Bottom-Left
    //        cv::Mat q3(magI, cv::Rect(cx, cy, cx, cy)); // Bottom-Right

    //        cv::Mat tmp;                           // swap quadrants (Top-Left with Bottom-Right)
    //        q0.copyTo(tmp);
    //        q3.copyTo(q0);
    //        tmp.copyTo(q3);

    //        q1.copyTo(tmp);                    // swap quadrant (Top-Right with Bottom-Left)
    //        q2.copyTo(q1);
    //        tmp.copyTo(q2);

    //        cv::normalize(magI, magI, 0, 1, CV_MINMAX); // Transform the matrix with float values into a
    //        if(debug)
    //            // viewable image form (float between values 0 and 1).
    //            cv::imshow("3. spectrum magnitude", magI);

    //        cv::Mat mask = cv::Mat(cv::Size(magI.rows,magI.cols),CV_32F,cv::Scalar(1));
    //        if(debug)
    //            cv::imshow("a.1 mask", mask);
    //        cv::circle(mask, cv::Point(mask.rows/2,mask.cols/2),dummy_temp, cv::Scalar(0), -1);
    //        cv::circle(mask, cv::Point(mask.rows/2,mask.cols/2),dummy_temp/2, cv::Scalar(1), -1);
    //        if(debug)
    //            cv::imshow("a.2 mask", mask);
    //        cv::GaussianBlur(mask,mask,cv::Size(kernel_size,kernel_size),0);
    //        if(debug)
    //            cv::imshow("a.3 mask", mask);
    //        cv::Mat new_magI = magI.mul(mask);
    //        if(debug)
    //            cv::imshow("a.4 new_magI",new_magI);
    //        //re-swap quadrants
    //        cv::Mat q0_t(mask, cv::Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
    //        cv::Mat q1_t(mask, cv::Rect(cx, 0, cx, cy));  // Top-Right
    //        cv::Mat q2_t(mask, cv::Rect(0, cy, cx, cy));  // Bottom-Left
    //        cv::Mat q3_t(mask, cv::Rect(cx, cy, cx, cy)); // Bottom-Right

    //        cv::Mat tmp_t;                           // swap quadrants (Top-Left with Bottom-Right)
    //        q0_t.copyTo(tmp_t);
    //        q3_t.copyTo(q0_t);
    //        tmp_t.copyTo(q3_t);

    //        q1_t.copyTo(tmp_t);                    // swap quadrant (Top-Right with Bottom-Left)
    //        q2_t.copyTo(q1_t);
    //        tmp_t.copyTo(q2_t);
    //        if(debug)
    //            cv::imshow("a.5 mask",mask);
    //        planes[0] = planes[0].mul(mask);
    //        cv::Mat dftFiltered;
    //        cv::merge(planes, 2, dftFiltered);

    //        cv::Mat finalImage;
    //        cv::dft(dftFiltered,finalImage, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT | cv::DFT_SCALE);//| cv::DFT_SCALE
    //        if(debug)
    //            cv::imshow("a.6 finalImage",finalImage);
    //        cv::Mat inverseTransform;
    //        finalImage.convertTo(inverseTransform, CV_8U);
    //        if(debug)
    //            cv::imshow("a.7 noIdea",inverseTransform);
    ///////////////////////////////////////////
    for(int i=0;i<1;i++){
        //https://docs.opencv.org/3.4/d3/d8f/samples_2cpp_2tutorial_code_2ImgProc_2Smoothing_2Smoothing_8cpp-example.html#a12
        //cv::bilateralFilter(image_gray,image_gray,9,18,5);
        //cv::bilateralFilter(image_F_gray,image_F_gray,9,18,5);
        cv::medianBlur(image_gray,image_gray,kernel_size);
        cv::medianBlur(image_F_gray,image_F_gray,kernel_size);
    }
    if(debug)
        cv::imshow("1 blur",image_gray);
    if(debug)
        cv::imshow("1.f blur",image_F_gray);

    cv::threshold(image_gray,image_gray,0,255,cv::THRESH_BINARY | cv::THRESH_OTSU );
    cv::threshold(image_F_gray,image_F_gray,0,255,cv::THRESH_BINARY | cv::THRESH_OTSU );
    if(debug)
        cv::imshow("1.1 blur+thr",image_gray);
    if(debug)
        cv::imshow("1.1.f blur+thr",image_F_gray);

    cv::Mat StructElement = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(kernel_size,kernel_size));
    cv::morphologyEx(image_F_gray,image_F_gray,cv::MORPH_CLOSE,StructElement);
    cv::morphologyEx(image_gray,image_gray,cv::MORPH_CLOSE,StructElement);
    if(debug)
        cv::imshow("1.2 blur+thr+close",image_gray);
    if(debug)
        cv::imshow("1.2.f blur+thr+close",image_F_gray);

    //        cv::medianBlur(image_gray,image_gray,kernel_size);
    //        cv::medianBlur(image_F_gray,image_F_gray,kernel_size);

    //        cv::imshow("0.2.1 threshold+blur",image_gray);
    //        cv::imshow("0.2.1.f threshold+blur",image_F_gray);

    cv::adaptiveThreshold(image_F_gray,image_F_gray,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY_INV,kernel_size,2); //CV_THRESH_BINARY
    cv::adaptiveThreshold(image_gray,image_gray,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY_INV,kernel_size,2); //CV_THRESH_BINARY
    if(debug){
        cv::imshow("2 threshold",image_gray);
        cv::imshow("2.f threshold",image_F_gray);
    }
    cv::Ptr <cv::Feature2D> detector;
    //const int DescriptorAlgorithm = ui->algorithm_box->value();//set as input to the function
    std::string algo_name = "none";
    if(DescriptorAlgorithm == 0){
        detector = cv::xfeatures2d::SURF::create();
        algo_name = "SURF";
    }else if (DescriptorAlgorithm == 1){
        detector = cv::xfeatures2d::SIFT::create();
        algo_name = "SIFT";
    }else if (DescriptorAlgorithm == 2){
        //https://docs.opencv.org/2.4/modules/features2d/doc/feature_detection_and_description.html
        detector = cv::ORB::create();//(500, 1.2, 8,0,2,cv::ORB::HARRIS_SCORE,31);
        algo_name = "ORB";
    }else if (DescriptorAlgorithm == 3){
        detector = cv::AKAZE::create();
        algo_name = "AKAZE";
    }else if (DescriptorAlgorithm == 4){
        detector = cv::xfeatures2d ::StarDetector::create();
        algo_name = "STAR";
    }else if (DescriptorAlgorithm == 5){//aruco
        algo_name = "ARUCO";
        auto dictionary = cv::aruco::generateCustomDictionary(512,3);
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::Ptr<cv::aruco::DetectorParameters> parameters;
        //cv::aruco::detectMarkers(RoiImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        cv::aruco::detectMarkers(RoiImage, dictionary, markerCorners, markerIds);
        cv::Mat outputImage = RoiImage.clone();
        auto s     = std::to_string(input_1);
        auto chip  = std::to_string(input_2);
        auto match = std::to_string(markerCorners.size());
        if(markerCorners.size()!=1){
            X_distance = 800000+markerCorners.size();
            Y_distance = 800000+markerCorners.size();
            cv::putText(outputImage,"fail"+match,cv::Point(30,window_size-4),cv::FONT_HERSHEY_PLAIN,4,cv::Scalar(255,255,255),3);
        }else{
            cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
            cv::Point F_center = Square_center(markerCorners.at(0).at(0),markerCorners.at(0).at(1),
                                               markerCorners.at(0).at(2),markerCorners.at(0).at(3));
            int ROIcenter_rows = outputImage.rows/2; //Defining the center of the image
            int ROIcenter_cols = outputImage.cols/2;
            X_distance = (F_center.x - ROIcenter_cols)*(1./Calibration); //[um]
            Y_distance = (F_center.y - ROIcenter_rows)*(1./Calibration); //[um]
            cv::circle(outputImage, F_center, 3, cv::Scalar(255,0,0), -1, 8, 0 );
        }
        if(debug)
            cv::imshow("aruco_image",outputImage);
        std::string time_now_str = "";
        int start_x = 15;
        int start_y = 5;
        addInfo(outputImage,algo_name,start_x,start_y,2,2,time_now_str);
        cv::imwrite("EXPORT/"+algo_name+"_"+s+"_"+time_now_str+".jpg",outputImage);
        return true;
    }else{
        qWarning("Error!! DescriptorAlgorithm not set properly!!");
        return false;}
    std::vector<cv::KeyPoint> keypoints_F(0);
    std::vector<cv::KeyPoint> keypoints_image(0);
    keypoints_F.clear();
    keypoints_image.clear();
    std::vector<cv::DMatch> matches;
    cv::Ptr<cv::DescriptorExtractor> descriptor_extractor;
    std::vector<std::vector< cv::DMatch> > matches_2;
    cv::Mat descriptorImage;
    cv::Mat descriptorFiducial;
    if(DescriptorAlgorithm != 4){
        detector->detectAndCompute(image_gray,cv::Mat(),keypoints_image,descriptorImage,false);
        detector->detectAndCompute(image_F_gray,cv::Mat(),keypoints_F,descriptorFiducial,false);
    }else{
        qInfo("Star feature detection");
        detector->detect(image_gray,keypoints_image);
        detector->detect(image_F_gray,keypoints_F);
        descriptor_extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
        descriptor_extractor->compute(image_gray,keypoints_image,descriptorImage);
        descriptor_extractor->compute(image_F_gray,keypoints_F,descriptorFiducial);
    }
    qInfo("Fiducial keypoints %i",keypoints_F.size());
    qInfo("Image    keypoints %i",keypoints_image.size());

    //https://docs.opencv.org/2.4/modules/features2d/doc/common_interfaces_of_descriptor_matchers.html
    cv::Ptr<cv::BFMatcher> matcher;// = cv::BFMatcher::create(cv::NORM_HAMMING,true);
    bool flann_true = false;
    if(DescriptorAlgorithm == 2){//ORB
        matcher = cv::BFMatcher::create(cv::NORM_HAMMING,true);
        matcher->match(descriptorFiducial,descriptorImage,matches,cv::Mat());
    }else if (DescriptorAlgorithm == 3){//AKAZE
        matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
        matcher->knnMatch(descriptorFiducial,descriptorImage,matches_2,2,cv::Mat(),false);
    }else{
        qInfo("Star, Surf, Sift matching");
        if(flann_true){//flann is NOT working
            int FLANN_INDEX_KDTREE = 0;
            cv::Ptr<cv::flann::IndexParams> index_params;
            index_params->setAlgorithm(FLANN_INDEX_KDTREE);
            index_params->setInt("tree",5);
            cv::Ptr<cv::flann::SearchParams> search_params = new cv::flann::SearchParams(50,0,true);
            cv::Ptr<cv::FlannBasedMatcher> matcher_flann = new cv::FlannBasedMatcher(index_params,search_params);
            matcher_flann->knnMatch(descriptorFiducial,descriptorImage,matches_2,2);
        }else{
            matcher = cv::BFMatcher::create();
            matcher->knnMatch(descriptorFiducial,descriptorImage,matches_2,2,cv::Mat(),false);
        }
    }
    std::vector<cv::DMatch> SortedMatches;
    const float Lowe_ratio = 0.7; //loose: 0.9, tight: 0.7
    if(DescriptorAlgorithm == 2){//ORB
        SortedMatches = matches;
        sort(SortedMatches.begin(),SortedMatches.end(),Distance_sorter);
        //http://www.cplusplus.com/reference/algorithm/sort/
    } else {
        //https://docs.opencv.org/3.0-beta/doc/tutorials/features2d/akaze_tracking/akaze_tracking.html
        for(unsigned int j=0; j< matches_2.size(); j++){
            if(matches_2[j][0].distance < matches_2[j][1].distance*Lowe_ratio)
                SortedMatches.push_back(matches_2[j][0]);
        }
    }

    //debug
    cv::Mat test_1;
    cv::Mat test_2;
    cv::drawKeypoints(image_gray,keypoints_image,test_1,cv::Scalar(0,0,255));
    cv::drawKeypoints(image_F_gray,keypoints_F,test_2,cv::Scalar(0,0,255));
    if(debug)
        cv::imshow("3. keypoints image",test_1);
    if(debug)
        cv::imshow("3. keypoints F",test_2);
    cv::Mat result;
    cv::drawMatches(image_F_gray, keypoints_F, image_gray, keypoints_image, SortedMatches, result);

    const unsigned int min_matches = 4;//add max number of matches?
    if(debug)
        std::cout<<" SortedMatches.size()  "<<SortedMatches.size()<<std::endl;
    if(SortedMatches.size() < min_matches){
        log->append("Error!! Not reached minimum number of matches.");
        return false;}

    //-- Localize the object
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;

    for (unsigned int i = 0; i < SortedMatches.size(); i++){
        //-- Get the keypoints from the good matches
        obj.push_back(keypoints_F[SortedMatches[i].queryIdx].pt);
        scene.push_back(keypoints_image[SortedMatches[i].trainIdx].pt);
    }

    cv::Mat H = cv::estimateAffinePartial2D(obj, scene,cv::noArray(),cv::RANSAC,5.0);
    if(debug){
        std::cout<<"H.rows: "<<H.rows <<" ;H.cols "<<H.cols<<std::endl;
        std::cout<<"H[1,1] "<< cv::Scalar(H.at<double>(0,0)).val[0]<<" H[1,2] "<<cv::Scalar(H.at<double>(0,1)).val[0]<<" H[1,3] "<<cv::Scalar(H.at<double>(0,2)).val[0]<<std::endl;
        std::cout<<"H[2,1] "<< cv::Scalar(H.at<double>(1,0)).val[0]<<" H[2,2] "<<cv::Scalar(H.at<double>(1,1)).val[0]<<" H[2,3] "<<cv::Scalar(H.at<double>(1,2)).val[0]<<std::endl;
        qInfo("H[1,1] : %2.2f H[1,2] : %2.2f H[1,3] : %2.2f",cv::Scalar(H.at<double>(0,0)).val[0],cv::Scalar(H.at<double>(0,1)).val[0],cv::Scalar(H.at<double>(0,2)).val[0]);
        qInfo("H[2,1] : %2.2f H[2,2] : %2.2f H[2,3] : %2.2f",cv::Scalar(H.at<double>(1,0)).val[0],cv::Scalar(H.at<double>(1,1)).val[0],cv::Scalar(H.at<double>(1,2)).val[0]);
    }
    //std::string ty = type2str( H.type() );
    //qInfo("Matrix: %s %dx%d \n", ty.c_str(), H.cols, H.rows );
    //Matrix type 64FC1
    //-- Get the corners from the image_1 ( the object to be "detected" )
    transform_out = H;

    std::vector<cv::Point2f> obj_corners(4);
    obj_corners[0] = cv::Point(0, 0);
    obj_corners[1] = cv::Point(image_F_gray.cols, 0);
    obj_corners[2] = cv::Point(image_F_gray.cols, image_F_gray.rows);
    obj_corners[3] = cv::Point(0, image_F_gray.rows);
    std::vector<cv::Point2f> scene_corners(4);
    cv::transform(obj_corners, scene_corners, H);

    cv::line(result, scene_corners[0] + cv::Point2f(image_F_gray.cols, 0), scene_corners[1] + cv::Point2f(image_F_gray.cols, 0), cv::Scalar(0, 255, 0), 4);
    cv::line(result, scene_corners[1] + cv::Point2f(image_F_gray.cols, 0), scene_corners[2] + cv::Point2f(image_F_gray.cols, 0), cv::Scalar(0, 255, 0), 4);
    cv::line(result, scene_corners[2] + cv::Point2f(image_F_gray.cols, 0), scene_corners[3] + cv::Point2f(image_F_gray.cols, 0), cv::Scalar(0, 255, 0), 4);
    cv::line(result, scene_corners[3] + cv::Point2f(image_F_gray.cols, 0), scene_corners[0] + cv::Point2f(image_F_gray.cols, 0), cv::Scalar(0, 255, 0), 4);

    cv::Point F_center = Square_center(scene_corners.at(0),scene_corners.at(1),scene_corners.at(2),scene_corners.at(3));
    cv::circle(RoiImage, F_center, 3, cv::Scalar(255,0,0), -1, 8, 0 );
    cv::line(RoiImage, scene_corners[0], scene_corners[1], cv::Scalar(0, 255, 0), 4);
    cv::line(RoiImage, scene_corners[1], scene_corners[2], cv::Scalar(0, 255, 0), 4);
    cv::line(RoiImage, scene_corners[2], scene_corners[3], cv::Scalar(0, 255, 0), 4);
    cv::line(RoiImage, scene_corners[3], scene_corners[0], cv::Scalar(0, 255, 0), 4);

    cv::Point2f RoItranslation = cv::Point2f((image.cols-window_size)*0.5, (image.rows-window_size)*0.5);
    cv::line(image, scene_corners[0] + RoItranslation, scene_corners[1] + RoItranslation, cv::Scalar(0, 255, 0), 4);
    cv::line(image, scene_corners[1] + RoItranslation, scene_corners[2] + RoItranslation, cv::Scalar(0, 255, 0), 4);
    cv::line(image, scene_corners[2] + RoItranslation, scene_corners[3] + RoItranslation, cv::Scalar(0, 255, 0), 4);
    cv::line(image, scene_corners[3] + RoItranslation, scene_corners[0] + RoItranslation, cv::Scalar(0, 255, 0), 4);
    cv::circle(image, cv::Point(center_cols,center_rows), 3, cv::Scalar(0,0,255), -1, 8, 0 );

    if(debug){
        //cv::namedWindow(algo_name +" Match", CV_WINDOW_KEEPRATIO);
        //cv::imshow(algo_name +" Match", result);
        cv::imshow(algo_name +" Match - RoI", RoiImage);
        //cv::imshow(algo_name +" Match - original", image);
    }
    //putting labels on output image
    std::string time_now_str = "";
//    int start_x = 15;
//    int start_y = 5;
//    std::cout<<"ok 1x"<<std::endl;
//    addInfo(RoiImage,algo_name,start_x,start_y,3,2,time_now_str);
//    std::cout<<"ok 2x"<<std::endl;
    std::string one      = std::to_string(input_1);
    std::string two      = std::to_string(input_2);
    std::string three    = std::to_string(input_3);
    std::string result_s = "";
    //cv::imwrite("EXPORT/"+chip+"_"+s+"_"+time_now_str+".jpg",output_mat);
    //cv::imwrite("EXPORT/"+algo_name+"_"+one+"_"+two+"_"+three+".jpg",RoiImage);
    //cv::imwrite("EXPORT/"+algo_name+"_"+chip+"_"+s+".jpg",RoiImage);
    //cv::imwrite("EXPORT/"+algo_name+"_match_"+chip+"_"+s+"_"+time_now_str+".jpg",result);

    int ROIcenter_rows = RoiImage.rows/2; //Defining the center of the image
    int ROIcenter_cols = RoiImage.cols/2;

    X_distance = (F_center.x - ROIcenter_cols)*(1./Calibration); //[um]
    Y_distance = (F_center.y - ROIcenter_rows)*(1./Calibration); //[um]
    timestamp = time_now_str;
    descriptor_extractor.release();
    matcher.release();
    detector.release();

    double H_1_1 = cv::Scalar(H.at<double>(0,0)).val[0];
    double H_1_2 = cv::Scalar(H.at<double>(0,1)).val[0];
    if( (sqrt(H_1_1*H_1_1 + H_1_2*H_1_2) > 1.05 || sqrt(H_1_1*H_1_1 + H_1_2*H_1_2) < 0.95) )
        fail_code = 1; //control on the scale of the fiducial, which should be close to 1

    if((abs(F_center.x - ROIcenter_cols) > image_F_gray.cols/3) || (abs(F_center.y - ROIcenter_rows) > image_F_gray.rows/3))
        fail_code = 2;

    if(fail_code != 0){
        result_s = "FAIL";
        cv::imwrite("EXPORT/"+algo_name+"_"+one+"_"+two+"_"+three+"_"+result_s+".jpg",RoiImage);
        return false;
    }else{
        result_s = "SUCCES";
        cv::imwrite("EXPORT/"+algo_name+"_"+one+"_"+two+"_"+three+"_"+result_s+".jpg",RoiImage);
        return true;
    }

}

void function1_func(const alglib::real_1d_array &x, double &func, void *ptr)
{
    //
    // this callback calculates f(x0,x1) = 100*(x0+3)^4 + (x1-3)^4
    //
    func = 100*pow(x[0]+3,4) + pow(x[1]-3,4);
}


int FiducialFinder::dumb_test()
{
    //
    // This example demonstrates minimization of f(x,y) = 100*(x+3)^4+(y-3)^4
    // using numerical differentiation to calculate gradient.
    //
    alglib::real_1d_array x = "[0,0]";
    double epsg = 0.0000000001;
    double epsf = 0;
    double epsx = 0;
    double diffstep = 1.0e-6;
    alglib::ae_int_t maxits = 0;
    alglib::minlbfgsstate state;
    alglib::minlbfgsreport rep;

    alglib::minlbfgscreatef(1, x, diffstep, state);
    alglib::minlbfgssetcond(state, epsg, epsf, epsx, maxits);
    alglib::minlbfgsoptimize(state, function1_func);
    alglib::minlbfgsresults(state, x, rep);

    printf("%d\n", int(rep.terminationtype)); // EXPECTED: 4
    printf("%s\n", x.tostring(2).c_str()); // EXPECTED: [-3,3]
    return 0;
}



