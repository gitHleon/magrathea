#include "Fiducial_finder.h"

bool Distance_sorter(cv::DMatch m_1,cv::DMatch m_2){
    return m_1.distance < m_2.distance;
}

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

bool FiducialFinder::Is_equal(const double &one, const double &two){
    //Tolerance 3um, the precision of the gantry
    double tolerance = 3*Calibration; //[px]
    return ( fabs(one-two) <= tolerance);
}

bool FiducialFinder::Is_a_triangle(const cv::Point &P_1, const cv::Point &P_2, const cv::Point &P_3){
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
    const int window_size = 420; //1000
    cv::Rect regione_interessante(center_cols-(window_size*0.5),center_rows-(window_size*0.5),window_size,window_size); //Rectangle that will be the RegionOfInterest (ROI)
    cv::Mat RoiImage = image(regione_interessante);
    //cv::circle(image, cv::Point(center_cols,center_rows), 3, cv::Scalar(206,78,137), -1, 8, 0 );
    cv::imshow("0 image",image);
    cv::imshow("0.1 image ROI",RoiImage);
    cv::Mat image_gray   = RoiImage.clone(); // Selecting ROI from the input image
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
            X_distance = (center_cols - square_center.x)*Calibration;
            Y_distance = (center_rows - square_center.y)*Calibration;
            cv::circle(RoiImage, cv::Point(RoiImage.cols/2.0,RoiImage.rows/2.0), 3, cv::Scalar(206,78,137), -1, 8, 0 );
        }
    }

    cv::imshow("3 Results",RoiImage);
    return;
    //add return of the fid center
}

void FiducialFinder::Find_F(const int &DescriptorAlgorithm, double &X_distance, double &Y_distance){

    //to find the 4 dot fiducial

    if(image.empty()){
        log->append("Error!! Image is empty!!");
        return;}
    if(image_fiducial.empty()){
        log->append("Error!! Fiducial is empty!!");
        return;}

        const bool debug = false;
        int center_rows = image.rows/2.0; //Defining the center of the image
        int center_cols = image.cols/2.0;

        cv::imshow("f. 0 image",image);
        //cv::Point2i Center_point = {center_cols,center_rows};
        const int window_size = 420; //1000
        if(window_size >= image.rows || window_size >= image.cols){
            log->append("Error!! Window size wrongly set!!");
            return;}

        cv::Rect regione_interessante(center_cols-(window_size*0.5),center_rows-(window_size*0.5),window_size,window_size); //Rectangle that will be the RegionOfInterest (ROI)
        cv::Mat RoiImage = image(regione_interessante);
        cv::imshow("f. 0.1 image ROI",RoiImage);
        cv::imshow("f. 0.1.f image ROI",image_fiducial);

        cv::Mat image_gray   = RoiImage.clone(); // Selecting ROI from the input image
        cv::Mat image_F_gray = image_fiducial.clone(); // Selecting ROI from the input image

        cv::cvtColor(image_gray,image_gray,CV_BGR2GRAY); //in future set the camera to take gray image directly
        cv::cvtColor(image_F_gray,image_F_gray,CV_BGR2GRAY); //in future set the camera to take gray image directly

        cv::medianBlur(image_gray,image_gray,5);
        cv::medianBlur(image_F_gray,image_F_gray,5);

        cv::imshow("f. 1 blur",image_gray);
        cv::imshow("f. 1.f blur",image_F_gray);

        cv::adaptiveThreshold(image_F_gray,image_F_gray,255,CV_ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY_INV,11,2); //CV_THRESH_BINARY
        cv::adaptiveThreshold(image_gray,image_gray,255,CV_ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY_INV,11,2); //CV_THRESH_BINARY

        cv::imshow("f. 2 threshold",image_gray);
        cv::imshow("f. 2.f threshold",image_F_gray);

        cv::Ptr <cv::Feature2D> detector;
        //const int DescriptorAlgorithm = ui->algorithm_box->value();//set as input to the function

        if(DescriptorAlgorithm == 0){
            //detector = cv::Feature2D::SURF::create();
            log->append("Surf..");
        }else if (DescriptorAlgorithm == 1){
            log->append("Sift..");
        }else if (DescriptorAlgorithm == 2){
            //https://docs.opencv.org/2.4/modules/features2d/doc/feature_detection_and_description.html
            detector = cv::ORB::create();//(500, 1.2, 8,0,2,cv::ORB::HARRIS_SCORE,31);
            std::cout<<" ok 2"<<std::endl;
        }else if (DescriptorAlgorithm == 3){
            detector = cv::AKAZE::create();
        }else if (DescriptorAlgorithm == 4){
            log->append("Star..");
        }else{
            log->append("Error!! DescriptorAlgorithm not set properly!!");
            return;}
        std::vector<cv::KeyPoint> keypoints_F(0);
        std::vector<cv::KeyPoint> keypoints_image(0);
        std::vector<cv::DMatch> matches;
        std::vector<std::vector< cv::DMatch> > matches_2;
        cv::Mat descriptorImage;
        cv::Mat descriptorFiducial;
        if(detector->empty())
            std::cout<<" empty - a"<<std::endl;

        //return;
        if(DescriptorAlgorithm != 4){
            detector->detectAndCompute(image_gray,cv::Mat(),keypoints_image,descriptorImage,false);
            std::cout<<" ok 3"<<std::endl;
            detector->detectAndCompute(image_F_gray,cv::Mat(),keypoints_F,descriptorFiducial,false);
            std::cout<<" ok 3.1"<<std::endl;
        }else{
            log->append("Star feature detection");
        }
        qInfo("Fiducial keypoints %i",keypoints_F.size());
        qInfo("Image    keypoints %i",keypoints_image.size());

        //https://docs.opencv.org/2.4/modules/features2d/doc/common_interfaces_of_descriptor_matchers.html
        cv::Ptr<cv::BFMatcher> matcher;// = cv::BFMatcher::create(cv::NORM_HAMMING,true);
        if(DescriptorAlgorithm == 2){//ORB
            matcher = cv::BFMatcher::create(cv::NORM_HAMMING,true);
            std::cout<<" ok 4.1"<<std::endl;
            matcher->match(descriptorFiducial,descriptorImage,matches,cv::Mat());
            std::cout<<" ok 4.2"<<std::endl;
        }else if (DescriptorAlgorithm == 3){//AKAZE
            matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
            std::cout<<" ok 4.1"<<std::endl;
            matcher->knnMatch(descriptorFiducial,descriptorImage,matches_2,2,cv::Mat(),false);
            std::cout<<" ok 4.2"<<std::endl;
        }else
            log->append("Star, Surf, Sift matching");

        std::vector<cv::DMatch> SortedMatches;
        const double Lowe_ratio = 0.7; //loose: 0.9, tight: 0.7
        if(DescriptorAlgorithm == 2){//ORB
            SortedMatches = matches;
            sort(SortedMatches.begin(),SortedMatches.end(),Distance_sorter);
            std::cout<<" ok 5"<<std::endl;
            //http://www.cplusplus.com/reference/algorithm/sort/
        } else if (DescriptorAlgorithm == 3){//AKAZE
            //https://docs.opencv.org/3.0-beta/doc/tutorials/features2d/akaze_tracking/akaze_tracking.html
            for(unsigned int j=0; j< matches_2.size(); j++){
                if(matches_2[j][0].distance < matches_2[j][1].distance*Lowe_ratio)
                    SortedMatches.push_back(matches_2[j][0]);
            }
        }

        const unsigned int min_matches = 5;//add max number of matches?
        std::cout<<" SortedMatches.size()  "<<SortedMatches.size()<<std::endl;
        if(SortedMatches.size() < min_matches){
            log->append("Error!! Not reached minimum number of matches.");
            return;}


        //debug
        cv::Mat test_1;
        cv::Mat test_2;
        cv::drawKeypoints(image_gray,keypoints_image,test_1,cv::Scalar(0,0,255));
        cv::drawKeypoints(image_F_gray,keypoints_F,test_2,cv::Scalar(0,0,255));
        cv::imshow("3. keypoints image",test_1);
        cv::imshow("3. keypoints F",test_2);
        cv::Mat result;
        cv::drawMatches(image_F_gray, keypoints_F, image_gray, keypoints_image, SortedMatches, result);

 //       cv::Mat index;
 //       int nbMatch = int(SortedMatches.size());
 //       std::vector<cv::DMatch> bestMatches;
 //       int maxMatch = 10;//??
//        if (nbMatch > maxMatch) {
//            cv::Mat tab(nbMatch, 1, CV_32F);
//            for (int i = 0; i < nbMatch; i++)
//            {
//                tab.at<float>(i, 0) = matches[i].distance;
//                //std::cout << "Distance " << i << " is: " << matches[i].distance << std::endl;
//            }
//            cv::sortIdx(tab, index, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
//            for (int i = 0; i < maxMatch; i++)
//            {
//                bestMatches.push_back(matches[index.at<int>(i, 0)]);
//            }
//        }
//        else {
//            for (int i = 0; i < nbMatch; i++)
//            {
//                bestMatches.push_back(matches[index.at<int>(i, 0)]);
//            }
//        }

        //-- Localize the object
        std::vector<cv::Point2f> obj;
        std::vector<cv::Point2f> scene;

        for (unsigned int i = 0; i < SortedMatches.size(); i++){
            //-- Get the keypoints from the good matches
            obj.push_back(keypoints_F[SortedMatches[i].queryIdx].pt);
            scene.push_back(keypoints_image[SortedMatches[i].trainIdx].pt);
        }

        cv::Mat H = cv::findHomography(obj, scene, CV_RANSAC,5.0);

        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<cv::Point2f> obj_corners(4);
        obj_corners[0] = cv::Point(0, 0);
        obj_corners[1] = cv::Point(image_F_gray.cols, 0);
        obj_corners[2] = cv::Point(image_F_gray.cols, image_F_gray.rows);
        obj_corners[3] = cv::Point(0, image_F_gray.rows);
        std::vector<cv::Point2f> scene_corners(4);

        cv::perspectiveTransform(obj_corners, scene_corners, H);

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

        cv::namedWindow("ORB Match", CV_WINDOW_KEEPRATIO);
        cv::imshow("ORB Match", result);

        return;
}




