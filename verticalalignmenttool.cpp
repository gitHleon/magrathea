#include "verticalalignmenttool.h"
//routine to verify the vertical alignement of the optics. From what used by BNL group:
//https://github.com/sciollalab/BNL_ThermomechanicalStave/wiki/Calibration-Methods#camera-calibration
VerticalAlignmentTool::VerticalAlignmentTool(QWidget *parent) : QWidget(parent)
{}

VerticalAlignmentTool::~VerticalAlignmentTool()
{}

void VerticalAlignmentTool::Set_camera(const cv::VideoCapture &m_cap){
    cap = m_cap;
}


void VerticalAlignmentTool::EvaluateEccentricity(const cv::RotatedRect &box, double &Eccentricity){
    double a = (box.size.width > box.size.height)  ? box.size.width : box.size.height;
    double b = (box.size.width <= box.size.height) ? box.size.width : box.size.height;
    std::cout<<"a "<<a <<"  ;b  "<<b <<std::endl;
    double c = sqrt( fabs(a*a - b*b) );
    Eccentricity = c/a;
}

//https://stackoverflow.com/questions/34478402/opencv-how-to-count-objects-in-photo
//https://stackoverflow.com/questions/37540305/how-to-count-white-object-on-binary-image

//steps needed
//0. std preparation of the image
//1.find the mostbright spot
//create a loop of thresholds{
//2. find objects
//3. fit an ellipse through the centers of the found objects
//https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#ga17ed9f5d79ae97bd4c7cf18403e1689a
//4. evaluate eccentricity
//}
//camera is aligned if? always the same eccentrycity ? Or ellipse center not moving?

void VerticalAlignmentTool::Evaluate_vignette(){

    const int window_size = 500;
    if (!cap.isOpened()){
        qWarning("Error : Not able to open camera.");
        return;
    }
    qInfo("Vignette alignment start");

    cv::Mat mat_from_outside;
    cap.read(mat_from_outside);
    int center_rows = mat_from_outside.rows/2.0; //Defining the center of the image
    int center_cols = mat_from_outside.cols/2.0;
    cv::Rect regione_interessante(center_cols-(window_size*0.5),center_rows-(window_size*0.5),window_size,window_size); //Rectangle that will be the RegionOfInterest (ROI)
    cv::Mat RoiImage = mat_from_outside(regione_interessante);
    cv::Mat image_gray   = RoiImage.clone(); // Selecting ROI from the input image
    cv::cvtColor(image_gray,image_gray,CV_BGR2GRAY);
    cv::GaussianBlur(image_gray,image_gray,cv::Size(7,7),2,2);
    cv::medianBlur(image_gray,image_gray,7);
    //find brightest spot
    cv::Point max_position = cv::Point(0,0);
    cv::minMaxLoc(image_gray,nullptr,nullptr,nullptr,&max_position);
    DrawCross(RoiImage,max_position,cv::Scalar(255,0,0));
    std::cout<<"x: " <<max_position.x <<"  ;y:" <<max_position.y <<std::endl;
    cv::imshow("Roiimage",RoiImage);

    //for(int i=0;i<25;i++){
        for(int i=0;i<10;i++){
        int threshold_value=180;
        threshold_value += i*7;
        cv::Mat image_gray_2;
        std::cout<<"i: "<<i <<" ; ok0"<<std::endl;
        cv::threshold(image_gray,image_gray_2,threshold_value,255,CV_THRESH_BINARY);
        std::cout<<"i: "<<i <<" ; ok1"<<std::endl;
        cv::imshow("thr",image_gray_2);
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::RNG rng(12345);
        cv::findContours(image_gray_2, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
        //cv::Mat image_temp   = RoiImage.clone();
        std::cout<<"It. : "<<i<<" thr. :"<<threshold_value<<",contours.size(): "<<contours.size()<<std::endl;
        std::vector <cv::Point> Centers;
        //        for(unsigned  int j=0; j< contours.size(); j++ ){
        //            //https://www.pyimagesearch.com/2016/02/01/opencv-center-of-contour/
        //            cv::Moments M = cv::moments(contours.at(j));
        //            std::cout<<"j: "<<j <<" ; ok2"<<std::endl;
        //            int cX = (M.m10 / M.m00);
        //            int cY = (M.m01 / M.m00);
        //            Centers.push_back(cv::Point(cX,cY));
        //            std::cout<<"j: "<<j <<" ; ok3"<<std::endl;
        //        }
        double eccentricity = 0.;
        cv::Mat image_temp = RoiImage.clone();
        for(unsigned  int j=0; j< contours.size(); j++ ){
            //https://www.pyimagesearch.com/2016/02/01/opencv-center-of-contour/
            cv::Moments M = cv::moments(contours.at(j));
            std::cout<<"j: "<<j <<" ; ok2"<<std::endl;
            int cX = (M.m10 / M.m00);
            int cY = (M.m01 / M.m00);
            Centers.push_back(cv::Point(cX,cY));
            std::cout<<"j: "<<j <<" ; ok3"<<std::endl;
            cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            cv::drawContours( image_temp, contours, j, color, 2, 8, hierarchy, 0, cv::Point() );
            //Set a control for when the contour is too big or too small
            cv::RotatedRect fitted_ellipse = cv::fitEllipseDirect(contours.at(j)); //centers
            std::cout<<"j: "<<j <<" ; ok4"<<std::endl;
            EvaluateEccentricity(fitted_ellipse,eccentricity);
            std::cout<<"I : "<<i<<" ;J : "<<j<<" thr. : "<<threshold_value<<" ,eccentricity: "<<eccentricity<<std::endl;
            cv::ellipse(RoiImage, fitted_ellipse, cv::Scalar(0,j*10,threshold_value), 3);
        }
        cv::imshow("Image + contours",image_temp);
        //        for(unsigned  int j=0; j< contours.size(); j++ )
        //        {
        //            cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        //            cv::drawContours( image_temp, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
        //        }
        //        cv::imshow("Image",RoiImage);
        //        cv::imshow("Image + contours",image_temp);
    }
    //some criteria returning true or false
}

void VerticalAlignmentTool::DrawCross(cv::Mat &img_input, const cv::Point &center, const cv::Scalar &color){
    cv::line(img_input,cv::Point(center.x,0),cv::Point(center.x,img_input.cols),color,3);
    cv::line(img_input,cv::Point(0,center.y),cv::Point(img_input.rows,center.y),color,3);
}
