#include "Fiducial_finder.h"

bool Distance_sorter(cv::DMatch m_1,cv::DMatch m_2){
    return m_1.distance < m_2.distance;
}

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

void FiducialFinder::addInfo(cv::Mat &image,const std::string &algo_name, int start_x, int start_y,int text_font_size ,int text_thikness,std::string &timestamp){
    int baseline = 0;
    //int text_font_size = 2;
    //int text_thikness = 2;

    int window_size = image.rows;
    std::string um_str = " 50um";
    cv::putText(image,algo_name,cv::Point(start_x,window_size-start_y), CV_FONT_HERSHEY_PLAIN,text_font_size,cv::Scalar(255,255,255),text_thikness);
    cv::Size text_size = cv::getTextSize(algo_name, CV_FONT_HERSHEY_PLAIN,text_font_size,text_thikness,&baseline);
    QTime now = QTime::currentTime();
    QString time_now = now.toString("hhmmss");
    std::string time_now_str = time_now.toLocal8Bit().constData();
    timestamp = time_now_str;
    start_x += text_size.width;
    cv::putText(image,time_now_str,cv::Point(start_x,window_size-start_y), CV_FONT_HERSHEY_PLAIN,text_font_size-1,cv::Scalar(255,255,255),text_thikness);
    cv::Size time_size = cv::getTextSize(time_now_str, CV_FONT_HERSHEY_PLAIN,text_font_size-1,text_thikness,&baseline);
    start_x += time_size.width;
    cv::putText(image,um_str,cv::Point(start_x,window_size-start_y), CV_FONT_HERSHEY_PLAIN,text_font_size-1,cv::Scalar(255,255,255),text_thikness);
    cv::Size um_size = cv::getTextSize(um_str, CV_FONT_HERSHEY_PLAIN,text_font_size-1,text_thikness,&baseline);
    start_x += um_size.width;
    std::cout<<"base : "<<baseline<<std::endl;
    cv::line(image,cv::Point(start_x,window_size-start_y),cv::Point(start_x+(50*Calibration),window_size-start_y),cv::Scalar(255,255,255),text_thikness);
}


bool FiducialFinder::Is_equal(const double &one, const double &two){
    //function needed when searching for fiducial not using SURF
    //Tolerance 3um, the precision of the gantry
    double tolerance = 3*Calibration; //[px]
    return ( fabs(one-two) <= tolerance);
}

bool FiducialFinder::Is_a_triangle(const cv::Point &P_1, const cv::Point &P_2, const cv::Point &P_3){
    //function needed when searching for fiducial not using SURF
    //http://en.cppreference.com/w/cpp/language/range-for
    //https://softwareengineering.stackexchange.com/questions/176938/how-to-check-if-4-points-form-a-square
    //Distances between points are 22 - 20 um
    const int threshold_H = 27*Calibration;
    const int threshold_L = 15*Calibration;
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

void FiducialFinder::Find_SquareAndTriangles(const std::vector<cv::Point> &Centers, std::vector<std::vector<int> > &Squares, std::vector<std::vector<int> > &Triangles){
    //function needed when searching for fiducial not using SURF
    int Iteration = 0;
    Squares.clear();
    Triangles.clear();
    unsigned int num_points = Centers.size();
    for(unsigned int i=0;i<num_points;i++){
        for(unsigned int j=i+1;j<num_points;j++){
            for(unsigned int k=j+1;k<num_points;k++){
                if(Is_a_triangle(Centers.at(i),Centers.at(j),Centers.at(k))){
                    std::cout<<">> Tria True <<"<<std::endl;
                    std::vector <int> indeces (3);
                    indeces.at(0) = i;
                    indeces.at(1) = j;
                    indeces.at(2) = k;
                    Triangles.push_back(indeces);
                }
                for(unsigned int l=k+1;l<num_points;l++){
                    Iteration++;
                    if(Is_a_square(Centers.at(i),Centers.at(j),Centers.at(k),Centers.at(l))){
                        std::cout<<">> True <<"<<std::endl;
                        std::vector <int> indeces (4);
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
    //function needed when searching for fiducial not using SURF
    //http://en.cppreference.com/w/cpp/language/range-for
    //https://softwareengineering.stackexchange.com/questions/176938/how-to-check-if-4-points-form-a-square
    //Distances between points are 30 um
    const int threshold_H = 35*Calibration;
    const int threshold_L = 25*Calibration;
    cv::Vec2i p_1 = {P_1.x,P_1.y};
    cv::Vec2i p_2 = {P_2.x,P_2.y};
    cv::Vec2i p_3 = {P_3.x,P_3.y};
    cv::Vec2i p_4 = {P_4.x,P_4.y};
    enum Diagonals {NONE, P1P2 , P1P3 , P1P4};
    Diagonals diagonal = NONE;
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
                (cv::norm(p_1,p_3) < threshold_H) &&
                (cv::norm(p_1,p_3) > threshold_L))
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
                (cv::norm(p_1,p_3) < threshold_H) &&
                (cv::norm(p_1,p_3) > threshold_L))
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
    X_coord /= 4.;
    float Y_coord = P_1.y + P_2.y + P_3.y + P_4.y;
    Y_coord /= 4.;
    cv::Point Out;
    Out.x = X_coord;
    Out.y = Y_coord;
    return Out;
}

void FiducialFinder::Find_circles(double &X_distance, double &Y_distance){
    //function needed when searching for fiducial not using SURF
    //DEPRECATED
    //to find the 4 dot fiducial

    if(image.empty()){
        log->append("Error!! Image is empty!!");
        return;
    }
    //------ put this inside a function to have easy omogenisation and code update
    const bool debug = false;
    int center_rows = image.rows/2.0; //Defining the center of the image
    int center_cols = image.cols/2.0;
    //cv::Point2i Center_point = {center_cols,center_rows};
    const int window_size = 420;
    cv::Rect regione_interessante(center_cols-(window_size*0.5),center_rows-(window_size*0.5),window_size,window_size); //Rectangle that will be the RegionOfInterest (ROI)
    cv::Mat RoiImage = image(regione_interessante);
    //cv::circle(image, cv::Point(center_cols,center_rows), 3, cv::Scalar(206,78,137), -1, 8, 0 );
    cv::imshow("0 image",image);
    cv::imshow("0.1 image ROI",RoiImage);
    cv::Mat image_gray = RoiImage.clone(); // Selecting ROI from the input image
    cv::cvtColor(image_gray,image_gray,CV_BGR2GRAY); //in future set the camera to take gray image directly
    cv::medianBlur(image_gray,image_gray,5);
    cv::imshow("1 blur",RoiImage);
    cv::adaptiveThreshold(image_gray,image_gray,255,CV_ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY_INV,11,2); //CV_THRESH_BINARY
    cv::imshow("2 threshold",image_gray);
    //------

    //Size of dot: diameter = 20 um
    //double Calibration; //[px/um]
    int min_radius = 16*0.5*Calibration; //[px]
    int max_radius = 24*0.5*Calibration; //[px]
    int minDist = min_radius*2; //[px]
    int hough_threshold = min_radius*1.5; //[px]
    std::vector<cv::Vec3f> circles;
    circles.clear();

    if(debug)
        log->append("Applying Hough Circles with parameters: minDist, hough_threshold, min_radius, max_radius: "
                    +QString::number(minDist)+" , "+QString::number(hough_threshold)+" , "+QString::number(min_radius)+" , "+QString::number(max_radius));

    cv::HoughCircles(image_gray, circles, CV_HOUGH_GRADIENT, 1, minDist, 150, hough_threshold, min_radius, max_radius ); //image_gray

    std::vector <cv::Point> Centers (circles.size());
    for( size_t i = 0; i < circles.size(); i++ ){
        Centers[i].x = std::round(circles[i][0]);
        Centers[i].y = std::round(circles[i][1]);
        cv::Point center(std::round(circles[i][0]), std::round(circles[i][1]));
        int radius = std::round(circles[i][2]);
        // circle center
        cv::circle(RoiImage, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        cv::circle(RoiImage, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
    }

    std::vector <std::vector <int> > Squares(0);
    std::vector <std::vector <int> > Triangles(0);

    Find_SquareAndTriangles(Centers,Squares,Triangles);
    std::cout<<"Squares.size() "<<Squares.size()<<std::endl;
    for( size_t i = 0; i < Squares.size(); i++ ){
        for( int j = 0; j < 4; j++ )
            cv::line(RoiImage, Centers.at(Squares[i][j]), Centers.at(Squares[i][(j+1)%4]), cv::Scalar(0,255,0), 2, 8);
        cv::Point square_center = Square_center(Centers.at(Squares.at(i).at(0)),Centers.at(Squares.at(i).at(1)),
                                                Centers.at(Squares.at(i).at(2)),Centers.at(Squares.at(i).at(3)));
        cv::circle(RoiImage, square_center, 3, cv::Scalar(255,0,0), -1, 8, 0 );
        cv::circle(RoiImage, square_center, 40*Calibration, cv::Scalar(255,0,0), 3, 8, 0 );
        if(Squares.size() == 1){
            X_distance = (center_cols - square_center.x)*(1./Calibration); //[um]
            Y_distance = (center_rows - square_center.y)*(1./Calibration); //[um]
            cv::circle(RoiImage, cv::Point(RoiImage.cols/2.0,RoiImage.rows/2.0), 3, cv::Scalar(206,78,137), -1, 8, 0 );
        }
    }

    cv::imshow("3 Results",RoiImage);
    return;
    //add return of the fid center
}

void FiducialFinder::Find_F(const int &DescriptorAlgorithm, double &X_distance, double &Y_distance, const int &temp_input){
    //main function for finding fiducials
    //https://gitlab.cern.ch/guescini/fiducialFinder/blob/master/fiducialFinder.py

    if(image.empty()){
        log->append("Error!! Image is empty!!");
        return;}
    if(image_fiducial.empty()){
        log->append("Error!! Fiducial is empty!!");
        return;}

        int center_rows = image.rows/2.0; //Defining the center of the image
        int center_cols = image.cols/2.0;

        cv::imshow("f. 0 image",image);
        const int window_size = ( (image.cols > 2000 && image.rows > 2000) ? 1500 : 420);
        if(window_size >= image.rows || window_size >= image.cols){
            log->append("Error!! Window size wrongly set!!");
            return;}

        cv::Rect regione_interessante(center_cols-(window_size*0.5),center_rows-(window_size*0.5),window_size,window_size); //Rectangle that will be the RegionOfInterest (ROI)
        cv::Mat output_mat = image.clone();
        cv::Mat RoiImage = image(regione_interessante);
        cv::imshow("f. 0.1 image ROI",RoiImage);
        cv::imshow("f. 0.1.f image ROI",image_fiducial);

        cv::Mat image_gray   = RoiImage.clone(); // Selecting ROI from the input image
        cv::Mat image_F_gray = image_fiducial.clone(); // Selecting ROI from the input image
        //        cv::Mat image_gray;
        //        cv::Mat image_F_gray;

        cv::cvtColor(image_gray,image_gray,CV_BGR2GRAY); //in future set the camera to take gray image directly
        cv::cvtColor(image_F_gray,image_F_gray,CV_BGR2GRAY); //in future set the camera to take gray image directly

        for(int i=0;i<1;i++){
            //https://docs.opencv.org/3.4/d3/d8f/samples_2cpp_2tutorial_code_2ImgProc_2Smoothing_2Smoothing_8cpp-example.html#a12
            //cv::bilateralFilter(image_gray,image_gray,9,18,5);
            //cv::bilateralFilter(image_F_gray,image_F_gray,9,18,5);
            cv::medianBlur(image_gray,image_gray,9);
            cv::medianBlur(image_F_gray,image_F_gray,9);
        }

        cv::imshow("f. 1 blur",image_gray);
        cv::imshow("f. 1.f blur",image_F_gray);

        cv::threshold(image_gray,image_gray,0,255,CV_THRESH_BINARY | CV_THRESH_OTSU );
        cv::threshold(image_F_gray,image_F_gray,0,255,CV_THRESH_BINARY | CV_THRESH_OTSU );
        cv::imshow("f. 1.1 blur+thr",image_gray);
        cv::imshow("f. 1.1.f blur+thr",image_F_gray);

        cv::Mat StructElement = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));
        cv::morphologyEx(image_F_gray,image_F_gray,cv::MORPH_CLOSE,StructElement);
        cv::morphologyEx(image_gray,image_gray,cv::MORPH_CLOSE,StructElement);
        cv::imshow("f. 1.2 blur+thr+close",image_gray);
        cv::imshow("f. 1.2.f blur+thr+close",image_F_gray);

        cv::adaptiveThreshold(image_F_gray,image_F_gray,255,CV_ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY_INV,11,2); //CV_THRESH_BINARY
        cv::adaptiveThreshold(image_gray,image_gray,255,CV_ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY_INV,11,2); //CV_THRESH_BINARY

        cv::imshow("f. 2 threshold",image_gray);
        cv::imshow("f. 2.f threshold",image_F_gray);

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
            auto s     = std::to_string(temp_input);
            auto match = std::to_string(markerCorners.size());
            if(markerCorners.size()!=1){
                X_distance = 800000+markerCorners.size();
                Y_distance = 800000+markerCorners.size();
                cv::putText(outputImage,"fail"+match,cv::Point(30,window_size-4), CV_FONT_HERSHEY_PLAIN,4,cv::Scalar(255,255,255),3);
            }else{
                cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
                cv::Point F_center = Square_center(markerCorners.at(0).at(0),markerCorners.at(0).at(1),
                                                   markerCorners.at(0).at(2),markerCorners.at(0).at(3));
                //std::cout<<"center cols : "<< center_cols<< " ;center rows : "<<center_rows <<std::endl;
                int ROIcenter_rows = outputImage.rows/2.0; //Defining the center of the image
                int ROIcenter_cols = outputImage.cols/2.0;
                //std::cout<<"ROI cols : "<< ROIcenter_cols<< " ;ROI rows : "<<ROIcenter_rows <<std::endl;
                //std::cout<<"ceneter cols : "<< F_center.x<< " ;center rows : "<<F_center.y <<std::endl;
                //std::cout<<"Calibration  : "<< Calibration<<std::endl;
                X_distance = (ROIcenter_cols - F_center.x)*(1./Calibration); //[um]
                Y_distance = (ROIcenter_rows - F_center.y)*(1./Calibration); //[um]
                cv::circle(outputImage, F_center, 3, cv::Scalar(255,0,0), -1, 8, 0 );
            }
            cv::imshow("aruco_image",outputImage);
            std::string time_now_str = "";
            int start_x = 15;
            int start_y = 5;
            addInfo(outputImage,algo_name,start_x,start_y,2,2,time_now_str);
            cv::imwrite("EXPORT/"+algo_name+"_"+s+"_"+time_now_str+".jpg",outputImage);
            return;
        }else{
            qWarning("Error!! DescriptorAlgorithm not set properly!!");
            return;}
        std::vector<cv::KeyPoint> keypoints_F(0);
        std::vector<cv::KeyPoint> keypoints_image(0);
        keypoints_F.clear();
        keypoints_image.clear();
        std::vector<cv::DMatch> matches;
        cv::Ptr<cv::DescriptorExtractor> descriptor_extractor;
        std::vector<std::vector< cv::DMatch> > matches_2;
        cv::Mat descriptorImage;
        cv::Mat descriptorFiducial;
        //        if(detector->empty()){
        //            std::cout<<"Detector is empty"<<std::endl;
        //        }
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
            log->append("Star, Surf, Sift matching");
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
        const double Lowe_ratio = 0.7; //loose: 0.9, tight: 0.7
        if(DescriptorAlgorithm == 2){//ORB
            SortedMatches = matches;
            sort(SortedMatches.begin(),SortedMatches.end(),Distance_sorter);
            std::cout<<" ok 5"<<std::endl;
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
        cv::imshow("3. keypoints image",test_1);
        cv::imshow("3. keypoints F",test_2);
        cv::Mat result;
        cv::drawMatches(image_F_gray, keypoints_F, image_gray, keypoints_image, SortedMatches, result);

        const unsigned int min_matches = 4;//add max number of matches?
        std::cout<<" SortedMatches.size()  "<<SortedMatches.size()<<std::endl;
        if(SortedMatches.size() < min_matches){
            log->append("Error!! Not reached minimum number of matches.");
            return;}

        //-- Localize the object
        std::vector<cv::Point2f> obj;
        std::vector<cv::Point2f> scene;

        for (unsigned int i = 0; i < SortedMatches.size(); i++){
            //-- Get the keypoints from the good matches
            obj.push_back(keypoints_F[SortedMatches[i].queryIdx].pt);
            scene.push_back(keypoints_image[SortedMatches[i].trainIdx].pt);
        }

        //cv::Mat H = cv::findHomography(obj, scene, CV_RANSAC,5.0);
        //cv::Mat H = cv::estimateAffine2D(obj, scene,cv::noArray(),cv::RANSAC,5.0);
        cv::Mat H = cv::estimateAffinePartial2D(obj, scene,cv::noArray(),cv::RANSAC,5.0);
        ///////////////////////////////////////////////////////////////////////////////////////
        //WARNING!!! It works but output H is inconsitent with what said by manual of function estimateAffinePartial2D.
        //need to understand meaning of matrix elements
        ///////////////////////////////////////////////////////////////////////////////////////
        std::cout<<"H.rows: "<<H.rows <<" ;H.cols "<<H.cols<<std::endl;
        //cv::Scalar value_t = image_gray.at<uchar>(row,col);
        std::cout<<"H[1,1] "<< cv::Scalar(H.at<uchar>(0,0)).val[0]<<" H[1,2] "<<cv::Scalar(H.at<uchar>(0,1)).val[0]<<" H[1,3] "<<cv::Scalar(H.at<uchar>(0,2)).val[0]<<std::endl;
        std::cout<<"H[2,1] "<< cv::Scalar(H.at<uchar>(1,0)).val[0]<<" H[2,2] "<<cv::Scalar(H.at<uchar>(1,1)).val[0]<<" H[2,3] "<<cv::Scalar(H.at<uchar>(1,2)).val[0]<<std::endl;
        qInfo("H[1,1] : %2.2f H[1,2] : %2.2f H[1,3] : %2.2f",cv::Scalar(H.at<uchar>(0,0)).val[0],cv::Scalar(H.at<uchar>(0,1)).val[0],cv::Scalar(H.at<uchar>(0,2)).val[0]);
        qInfo("H[2,1] : %2.2f H[2,2] : %2.2f H[2,3] : %2.2f",cv::Scalar(H.at<uchar>(1,0)).val[0],cv::Scalar(H.at<uchar>(1,1)).val[0],cv::Scalar(H.at<uchar>(1,2)).val[0]);
        //std::string ty = type2str( H.type() );
        //qInfo("Matrix: %s %dx%d \n", ty.c_str(), H.cols, H.rows );
        //Matrix type 64FC1
        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<cv::Point2f> obj_corners(4);
        obj_corners[0] = cv::Point(0, 0);
        obj_corners[1] = cv::Point(image_F_gray.cols, 0);
        obj_corners[2] = cv::Point(image_F_gray.cols, image_F_gray.rows);
        obj_corners[3] = cv::Point(0, image_F_gray.rows);
        std::vector<cv::Point2f> scene_corners(4);

        //cv::perspectiveTransform(obj_corners, scene_corners, H);
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

        cv::namedWindow(algo_name +" Match", CV_WINDOW_KEEPRATIO);
        cv::imshow(algo_name +" Match", result);
        cv::imshow(algo_name +" Match - RoI", RoiImage);
        cv::imshow(algo_name +" Match - original", image);
        //putting labels on output image
        std::string time_now_str = "";
        int start_x = 15;
        int start_y = 5;
        addInfo(RoiImage,algo_name,start_x,start_y,2,2,time_now_str);
        std::string s = std::to_string(temp_input);

        cv::imwrite("EXPORT/"+s+"_"+time_now_str+".jpg",output_mat);
        cv::imwrite("EXPORT/"+algo_name+"_"+s+"_"+time_now_str+".jpg",RoiImage);
        cv::imwrite("EXPORT/"+algo_name+"_match_"+s+"_"+time_now_str+".jpg",result);

        int ROIcenter_rows = RoiImage.rows/2.0; //Defining the center of the image
        int ROIcenter_cols = RoiImage.cols/2.0;

        X_distance = (ROIcenter_cols - F_center.x)*(1./Calibration); //[um]
        Y_distance = (ROIcenter_rows - F_center.y)*(1./Calibration); //[um]

        //        cv::Scalar value_t = H.at<uchar>(0,0);
        //        double a = value_t.val[0];
        //        value_t = H.at<uchar>(0,1);//image_gray.at<uchar>(row,col)
        //        double b = value_t.val[0];
        //        value_t = H.at<uchar>(0,2);
        //        double c = value_t.val[0];
        //        value_t = H.at<uchar>(1,0);
        //        double d = value_t.val[0];
        //        value_t = H.at<uchar>(1,1);
        //        double e = value_t.val[0];
        //        value_t = H.at<uchar>(1,2);
        //        double f = value_t.val[0];
        //        double p_0 = sqrt(a*a + b*b);
        //        double theta = atan(d/a);
        //        qInfo("Rotation    :\t %.2f deg",((theta*180.)/3.14159));
        //        qInfo("Translation :\t %.0f px,\t %.0f px",c,f);
        //        qInfo("Scale 0      :\t %.2f",p_0);
        //        X_distance = c*Calibration;
        //        Y_distance = f*Calibration;
        descriptor_extractor.release();
        matcher.release();
        detector.release();
        return;
}






















