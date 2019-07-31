#include <chrono>
#include <ctime>
#include <algorithm>
#include "logger.h"
#include "Fiducial_finder.h"
#include "MatrixTransform.h"

void
function_ChiSquare(const alglib::real_1d_array &x, double &func, void *ptr)
{
    //
    // this callback calculates chi square function for square of circles
    // We will have 8+4+4 variables, 8 for the points coordinates and 4 for the paremeters we want to fit.
    // Legend of the valiables : x[0], x[1], etc.. => x_1, y_1, x_2, y_2, etc.. coordinates of the points of the image;
    // variables x[8] , ... ,  x[11] are the score of the circles, this are used as weights to give more importance to good circles
    // (i.e. the one with high score) variables x[12], ... ,  x[15] are the fit prameters
    double calibration_value = 10.5; //[px/um]
    // double one_over_sigma_square = 1./(5*calibration_value); //error of 2 um, calibration is 10 px / um and measures are in
    // pixels
    double real_side_size = 50 * calibration_value;
    double real_side_size_err = real_side_size * 0.2;
    func = pow(x[0] - (x[12] - (x[15] / sqrt(2)) * cos(x[14] + M_PI / 4)), 2) * pow(x[8], 2) +
           pow(x[1] - (x[13] - (x[15] / sqrt(2)) * sin(x[14] + M_PI / 4)), 2) * pow(x[8], 2) +
           pow(x[2] - (x[12] - (x[15] / sqrt(2)) * cos(M_PI / 4 - x[14])), 2) * pow(x[9], 2) +
           pow(x[3] - (x[13] + (x[15] / sqrt(2)) * sin(M_PI / 4 - x[14])), 2) * pow(x[9], 2) +
           pow(x[4] - (x[12] + (x[15] / sqrt(2)) * cos(M_PI / 4 + x[14])), 2) * pow(x[10], 2) +
           pow(x[5] - (x[13] + (x[15] / sqrt(2)) * sin(M_PI / 4 + x[14])), 2) * pow(x[10], 2) +
           pow(x[6] - (x[12] + (x[15] / sqrt(2)) * cos(M_PI / 4 - x[14])), 2) * pow(x[11], 2) +
           pow(x[7] - (x[13] - (x[15] / sqrt(2)) * sin(M_PI / 4 - x[14])), 2) * pow(x[11], 2) +
           (pow(x[15] - real_side_size, 2) / (real_side_size_err * real_side_size));
}




bool Distance_sorter(cv::DMatch m_1, cv::DMatch m_2)
{
    return m_1.distance < m_2.distance;
}

bool Point_sorter(cv::Point2d m_1, cv::Point2d m_2)
{
    return m_1.x < m_2.x;
}

std::string type2str(int type);

FiducialFinder::FiducialFinder()
{
}

FiducialFinder::~FiducialFinder()
{
}

void FiducialFinder::SetImage(const cv::Mat &input)
{
    image = input.clone();
}

void FiducialFinder::SetImage(const std::string &filename, int flags)
{
    image = cv::imread(filename, flags);
}

void FiducialFinder::SetImageFiducial(const cv::Mat &input)
{
    image_fiducial = input.clone();
}

void FiducialFinder::SetImageFiducial(const std::string &filename, int flags)
{
    image_fiducial = cv::imread(filename, flags);
}

bool FiducialFinder::IsImageEmpty()
{
    return image.empty();
}

cv::Mat
FiducialFinder::get_component(const cv::Mat &input_mat,
                              const int input)
{
    cv::Mat bgr[3];             // destination array
    cv::split(input_mat, bgr);  // split source
    return bgr[input];
    // Note: OpenCV uses BGR color order
}

cv::Mat
FiducialFinder::change_gamma(const cv::Mat &input_mat, const double &gamma)
{
    // change the gamma of an image
    if (gamma < 0 || gamma > 255)
    {
        std::cout << "enance_contrast: Error in gamma range" << std::endl;
        return input_mat;
    }

    cv::Mat lookUpTable(1, 256, CV_8U);
    uchar *p = lookUpTable.ptr();
    for (int i = 0; i < 256; ++i)
        p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);

    cv::Mat output = input_mat.clone();
    cv::LUT(input_mat, lookUpTable, output);

    return output;
}

cv::Mat
FiducialFinder::enance_contrast(const cv::Mat &input_mat, const double &alpha, const double &beta = 0)
{
    if (alpha < 0 || alpha > 3)
    {
        std::cout << "enance_contrast: Error in alpha range" << std::endl;
        return input_mat;
    }
    cv::Mat output;
    cv::Mat betas = cv::Mat::ones(input_mat.rows, input_mat.cols, input_mat.type());
    betas = betas * beta;
    output = input_mat.clone();
    output = output * alpha + betas;
    return output;
}

cv::Mat
FiducialFinder::dan_contrast(const cv::Mat &input_mat, const double &max_alpha)
{
    if (max_alpha < 0 || max_alpha > 3)
    {
        std::cout << "dan_contrast: Error in alpha range" << std::endl;
        return input_mat;
    }
    cv::Mat output;
    cv::Scalar mean_t;
    cv::Scalar stddev_t;
    cv::meanStdDev(input_mat, mean_t, stddev_t);
    const int steps = 5;
    double alpha_steps = max_alpha / (steps - 2);
    bool debug = false;

    std::vector<cv::Mat> matrices;
    std::vector<cv::Mat> matrices_mod;
    std::vector<double> threshold_steps;
    threshold_steps.push_back(0);
    threshold_steps.push_back(mean_t[0] - 1 * stddev_t[0]);
    threshold_steps.push_back(mean_t[0] + 1 * stddev_t[0]);
    threshold_steps.push_back(mean_t[0] + 2 * stddev_t[0]);
    threshold_steps.push_back(255);

    matrices.push_back(input_mat);
    for (unsigned int i = 1; i < (steps - 1); i++)
    {
        cv::Mat temp_thr;
        cv::threshold(input_mat, temp_thr, threshold_steps.at(i), 255, cv::THRESH_TOZERO);
        matrices.push_back(temp_thr);
    }
    matrices.push_back(cv::Mat::zeros(input_mat.rows, input_mat.cols, input_mat.type()));

    for (unsigned int i = 0; i < (steps - 1); i++)
    {
        cv::Mat temp_thr;
        temp_thr = matrices.at(i) - matrices.at(i + 1);
        double square_step = i / (steps - 2);
        std::cout << " value " << square_step << " square " << pow(square_step, 2.) << std::endl;
        matrices_mod.push_back(enance_contrast(temp_thr, alpha_steps * pow(square_step, 2)));
        // matrices_mod.push_back(enance_contrast(temp_thr,alpha_steps*sqrt(square_step)));
    }

    if (debug)
    {
        cv::imshow("test 1", matrices_mod.at(0));
        cv::imshow("test 2", matrices_mod.at(1));
        cv::imshow("test 3", matrices_mod.at(2));
        cv::imshow("test 4", matrices_mod.at(3));
    }

    output = matrices_mod.at(0).clone();
    for (unsigned int i = 1; i < matrices_mod.size(); i++)
    {
        output = output + matrices_mod.at(i);
    }
    return output;
}


void
FiducialFinder::addInfo(cv::Mat &image,
                        const std::string &algo_name,
                        int start_x, int start_y,
                        int text_font_size,
                        int text_thikness, std::string &timestamp)
{
    int baseline = 0;
    // int text_font_size = 2;
    // int text_thikness = 2;

    int window_size = image.rows;
    std::string um_str = " 50um";
    cv::putText(image, algo_name,
                cv::Point(start_x, window_size - start_y),
                cv::FONT_HERSHEY_PLAIN,
                text_font_size,
                cv::Scalar(255, 255, 255),
                text_thikness);

    cv::Size text_size = cv::getTextSize(algo_name,
                                         cv::FONT_HERSHEY_PLAIN,
                                         text_font_size,
                                         text_thikness,
                                         &baseline);

    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::string time_now_str = std::ctime(&now_time);

    start_x += text_size.width;
    cv::putText(image, time_now_str,
                cv::Point(start_x, window_size - start_y),
                cv::FONT_HERSHEY_PLAIN,
                text_font_size - 1,
                cv::Scalar(255, 255, 255),
                text_thikness);

    cv::Size time_size = cv::getTextSize(time_now_str,
                                         cv::FONT_HERSHEY_PLAIN,
                                         text_font_size - 1,
                                         text_thikness, &baseline);

    start_x += time_size.width;
    cv::putText(image, um_str,
                cv::Point(start_x, window_size - start_y),
                cv::FONT_HERSHEY_PLAIN,
                text_font_size - 1,
                cv::Scalar(255, 255, 255),
                text_thikness);

    cv::Size um_size = cv::getTextSize(um_str,
                                       cv::FONT_HERSHEY_PLAIN, text_font_size - 1,
                                       text_thikness,
                                       &baseline);
    start_x += um_size.width;
    cv::line(image,
             cv::Point(start_x, window_size - start_y),
             cv::Point(start_x + 50, window_size - start_y),
             cv::Scalar(255, 255, 255), text_thikness);
}

bool
FiducialFinder::Is_equal(const double &one, const double &two)
{
    // function needed when searching for fiducial not using SURF
    double tolerance = 10; //[px]
    return (fabs(one - two) <= tolerance);
}

bool
FiducialFinder::Is_a_triangle(const cv::Point2d &P_1, const cv::Point2d &P_2, const cv::Point2d &P_3)
{
    // function needed when searching for fiducial not using SURF
    // http://en.cppreference.com/w/cpp/language/range-for
    // https://softwareengineering.stackexchange.com/questions/176938/how-to-check-if-4-points-form-a-square
    // Distances between points are 22 - 20 um
    const double threshold_H = 27; // * Calibration;
    const double threshold_L = 15; // * Calibration;
    cv::Vec2d p_1 = {P_1.x, P_1.y};
    cv::Vec2d p_2 = {P_2.x, P_2.y};
    cv::Vec2d p_3 = {P_3.x, P_3.y};

    if (Is_equal(cv::norm(p_1, p_2), cv::norm(p_2, p_3)) &&
        Is_equal(cv::norm(p_2, p_3), cv::norm(p_3, p_1)) &&
        Is_equal(cv::norm(p_3, p_1), cv::norm(p_1, p_2)) &&
        (cv::norm(p_1, p_3) < threshold_H) &&
        (cv::norm(p_1, p_3) > threshold_L))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void
FiducialFinder::Find_SquareAndTriangles(const std::vector<cv::Vec4f> &Circles, std::vector<std::vector<unsigned int>> &Squares,
                                        std::vector<std::vector<unsigned int>> &Triangles)
{
    // function needed when searching for fiducial not using SURF
    int Iteration = 0;
    Squares.clear();
    Triangles.clear();
    unsigned int num_points = Circles.size();
    for (unsigned int i = 0; i < num_points; i++)
    {
        for (unsigned int j = i + 1; j < num_points; j++)
        {
            for (unsigned int k = j + 1; k < num_points; k++)
            {
                cv::Point point_i = cv::Point(Circles[i][0], Circles[i][1]);
                cv::Point point_j = cv::Point(Circles[j][0], Circles[j][1]);
                cv::Point point_k = cv::Point(Circles[k][0], Circles[k][1]);
                if (Is_a_triangle(point_i, point_j, point_k))
                {
                    std::cout << ">> Tria True <<" << std::endl;
                    std::vector<unsigned int> indeces(3);
                    indeces.at(0) = i;
                    indeces.at(1) = j;
                    indeces.at(2) = k;
                    Triangles.push_back(indeces);
                }
                for (unsigned int l = k + 1; l < num_points; l++)
                {
                    cv::Point point_l = cv::Point(Circles[l][0], Circles[l][1]);
                    Iteration++;
                    if (Is_a_square(point_i, point_j, point_k, point_l))
                    {
                        std::cout << ">> True <<" << std::endl;
                        std::vector<unsigned int> indeces(4);
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
}

bool
FiducialFinder::Is_a_square(const cv::Point2d &P_1,
                            const cv::Point2d &P_2,
                            const cv::Point2d &P_3,
                            const cv::Point2d &P_4)
{
    bool debug = false;
    // function needed when searching for fiducial not using SURF
    // http://en.cppreference.com/w/cpp/language/range-for
    // https://softwareengineering.stackexchange.com/questions/176938/how-to-check-if-4-points-form-a-square
    // Distances between points are 50 um. threshold is on the diagonal i.e. 50*1.4
    const int threshold_H = 85; //* Calibration;
    const int threshold_L = 55; //* Calibration;
    cv::Vec2d p_1 = {P_1.x, P_1.y};
    cv::Vec2d p_2 = {P_2.x, P_2.y};
    cv::Vec2d p_3 = {P_3.x, P_3.y};
    cv::Vec2d p_4 = {P_4.x, P_4.y};
    enum Diagonals
    {
        NONE,
        P1P2,
        P1P3,
        P1P4
    };
    Diagonals diagonal = NONE;
    if (debug)
    {
        std::cout << "cv::norm(p_1,p_2)" << cv::norm(p_1, p_2) << std::endl;
        std::cout << "cv::norm(p_1,p_3)" << cv::norm(p_1, p_3) << std::endl;
        std::cout << "cv::norm(p_1,p_4)" << cv::norm(p_1, p_4) << std::endl;
        std::cout << "cv::norm(p_2,p_3)" << cv::norm(p_2, p_3) << std::endl;
        std::cout << "cv::norm(p_2,p_4)" << cv::norm(p_2, p_4) << std::endl;
        std::cout << "cv::norm(p_3,p_4)" << cv::norm(p_3, p_4) << std::endl;
    }
    // searching for two equal diagonals
    // Diagonal is longer than the sides ;)
    if (cv::norm(p_1, p_2) < cv::norm(p_1, p_3))
    {
        if (cv::norm(p_1, p_3) < cv::norm(p_1, p_4))
            diagonal = P1P4;
        else
            diagonal = P1P3;
    }
    else
    {
        if (cv::norm(p_1, p_2) < cv::norm(p_1, p_4))
            diagonal = P1P4;
        else
            diagonal = P1P2;
    }

    if (diagonal == P1P2 && Is_equal(cv::norm(p_1, p_2), cv::norm(p_3, p_4)))
    {
        // If diagonals are equal, all the sides needs to be equal
        if (Is_equal(cv::norm(p_1, p_3), cv::norm(p_3, p_2)) &&
            Is_equal(cv::norm(p_3, p_2), cv::norm(p_2, p_4)) &&
            Is_equal(cv::norm(p_2, p_4), cv::norm(p_4, p_1)) &&
            Is_equal(cv::norm(p_4, p_1), cv::norm(p_1, p_3)) &&
            (cv::norm(p_1, p_2) < threshold_H) && (cv::norm(p_1, p_2) > threshold_L))
        {
            return true;
        }
    }
    else if (diagonal == P1P3 && Is_equal(cv::norm(p_1, p_3), cv::norm(p_2, p_4)))
    {
        // If diagonals are equal, all the sides needs to be equal
        if (Is_equal(cv::norm(p_1, p_2), cv::norm(p_2, p_3)) &&
            Is_equal(cv::norm(p_2, p_3), cv::norm(p_3, p_4)) &&
            Is_equal(cv::norm(p_3, p_4), cv::norm(p_4, p_1)) &&
            Is_equal(cv::norm(p_4, p_1), cv::norm(p_1, p_2)) &&
            (cv::norm(p_1, p_3) < threshold_H) && (cv::norm(p_1, p_3) > threshold_L))
        {
            return true;
        }
    }
    else if (diagonal == P1P4 && Is_equal(cv::norm(p_1, p_4), cv::norm(p_2, p_3)))
    {
        // If diagonals are equal, all the sides needs to be equal
        if (Is_equal(cv::norm(p_1, p_2), cv::norm(p_2, p_4)) &&
            Is_equal(cv::norm(p_2, p_4), cv::norm(p_4, p_3)) &&
            Is_equal(cv::norm(p_4, p_3), cv::norm(p_3, p_1)) &&
            Is_equal(cv::norm(p_3, p_1), cv::norm(p_1, p_2)) &&
            (cv::norm(p_1, p_4) < threshold_H) && (cv::norm(p_1, p_4) > threshold_L))
        {
            return true;
        }
    }
    else
    {
        return false;
    }
    // if you reach here, it is not a square...
    return false;
}

cv::Point2d
FiducialFinder::Square_center(const cv::Point2d &P_1,
                              const cv::Point2d &P_2,
                              const cv::Point2d &P_3,
                              const cv::Point2d &P_4)
{
    float X_coord = P_1.x + P_2.x + P_3.x + P_4.x;
    X_coord /= 4;
    float Y_coord = P_1.y + P_2.y + P_3.y + P_4.y;
    Y_coord /= 4;
    cv::Point2d Out;
    Out.x = X_coord;
    Out.y = Y_coord;
    return Out;
}

cv::Mat FiducialFinder::find_region_of_intetest(const Point &image_O)
{

    /*
     * We define a region of interest centered in origin
     */
    int window_size = get_window_size();
    if (window_size >= image.rows || window_size >= image.cols)
    {
        return cv::Mat();
    }

    //Rectangle that will be the RegionOfInterest (ROI)
    cv::Rect region_of_interest(image_O.x()-(window_size/2),
                                image_O.y()-(window_size/2),
                                window_size,
                                window_size);
    return image(region_of_interest);
}

int FiducialFinder::get_kernel_size() const
{
    int kernel_size = ( (image.cols > 2000 && image.rows > 2000) ? 15 : 5);
    return kernel_size;
}

int FiducialFinder::get_window_size() const
{
    /*
     * TODO: find a way of computing this
     */
    int window_size = ( (image.cols > 2000 && image.rows > 2000)
            ? 2000
            : std::min(image.cols, image.rows))/4;
    return window_size;

}

Point FiducialFinder::Find_circles_(bool fit, bool single, const Point &origin, bool debug)
{
    //function needed when searching for fiducial not using SURF
    //to find the 4 dot fiducial
    LoggerStream os;
    bool print_raw = true;
    Point position = Point::NaN();

    if (image.empty())
    {
        LoggerStream os;
        os << loglevel(Log::error) << "Find_circles: image is empty" << std::endl;
        return position;
    }

    if(debug)
        cv::imshow("f. 0 image", image);

    int kernel_size = get_kernel_size();

    /*
     * Define the center of the  Region of Interest
     */
    Point image_O = origin;
    if (origin.is_nan())
        image_O.set(image.cols/2.0, image.rows/2.0);

    /*
     * Define the center of the Region of Interest
     */
    cv::Mat RoiImage = find_region_of_intetest(image_O);
    if (RoiImage.empty())
    {
        os << loglevel(Log::error) << "Window size wrongly set!!" << std::endl;
        return position;
    }


    if(debug)
        cv::imshow("f. 0.1 image ROI",RoiImage);

    cv::Mat output_mat = RoiImage.clone(); // Selecting ROI from the input image
    output_mat = get_component(output_mat,1);

    for (int iterations = 0; ; iterations++)
    {
        cv::Mat image_gray = output_mat.clone();
        if (iterations == 1)
        {
            //image_gray = enance_contrast(image_gray,1.5);
            image_gray = dan_contrast(image_gray, 2);
        }
        else if (iterations == 2)
        {
            image_gray = change_gamma(image_gray, 3);
        }
        else if (iterations == 3)
        {
            image_gray = dan_contrast(image_gray, 2);
            image_gray = change_gamma(image_gray, 3);
        }

        if (debug)
        {
            cv::imshow("contrast", image_gray);
            os << loglevel(Log::info) << " iteration : " << iterations << std::endl;
        }
        for (int i = 0; i < 1; i++)
        {
            //https://docs.opencv.org/3.4/d3/d8f/samples_2cpp_2tutorial_code_2ImgProc_2Smoothing_2Smoothing_8cpp-example.html#a12
            cv::medianBlur(image_gray, image_gray, kernel_size);
        }

        if(debug)
            cv::imshow("1 blur",image_gray);

        cv::threshold(image_gray, image_gray, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU );
        if(debug)
            cv::imshow("1.1 blur+thr",image_gray);

        cv::Mat StructElement = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(kernel_size, kernel_size));
        cv::morphologyEx(image_gray, image_gray, cv::MORPH_CLOSE,StructElement);
        if(debug)
            cv::imshow("1.2 blur+thr+close",image_gray);

        cv::adaptiveThreshold(image_gray, image_gray, 255,
                              cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                              cv::THRESH_BINARY_INV,
                              kernel_size, 2); //CV_THRESH_BINARY

        if(debug)
            cv::imshow("2 threshold",image_gray);

        if(debug)
        {
            std::string ty =  type2str( image_gray.type() );
            std::cout<< ty<<std::endl;
        }

        //-------------------------------------------------------------------------
        if (single)
        {
            //Size of dot: diameter = 300 um
            //double Calibration; //[px/um]
            int min_radius = 270*0.5; //*Calibration; //[px]
            int max_radius = 330*0.5; //*Calibration; //[px]
            int minDist = min_radius*4; //[px]
            double correction_factor = 0.4;
            int hough_threshold = min_radius*correction_factor; //[px]
            if(debug)
                std::cout<< " min_radius " << min_radius
                         << " max_radius " << max_radius
                         << " hough_threshold " << hough_threshold<<std::endl;

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

            cv::Mat RoiImage_out     = RoiImage.clone();
            //if not exactly one circle go to next iteration and apply modification to the image

            if(circles.size() != 1)
                continue;

            for (size_t i = 0; i < circles.size(); i++)
            {
                cv::Point2d center(circles[i][0], circles[i][1]);
                auto radius = circles[i][2];
                auto vote = circles[i][3];
                if (debug)
                   os << loglevel(Log::debug)
                      << " radius " << radius
                      << " CX " << center.x
                      << " CY " << center.y
                      << " vote : " << vote
                      << std::endl;
                // circle center
                cv::circle(RoiImage_out, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
                // circle outline
                cv::circle(RoiImage_out, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
                Point O_RoI(RoiImage_out.cols/2, RoiImage_out.rows/2.0);
                position = center - O_RoI;
            }
            if(debug)
                cv::imshow("3 Results",RoiImage_out);

            return position;
        }
        else  //if Single
        {
            //Size of dot: diameter = 20 um
            //double Calibration; //[px/um]
            int min_radius = 15*0.5; //*Calibration; //[px] //14 for SCToptics
            int max_radius = 25*0.5; //*Calibration; //[px] //30 for SCToptics
            //int min_radius = 20*0.5*Calibration; //[px] //for Calibration plate
            //int max_radius = 30*0.5*Calibration; //[px] //for Calibration plate
            int minDist = min_radius*4; //[px]
            double correction_factor = 0.5; //0.2 for SCToptics, 0.4 for calibration plate
            int hough_threshold = min_radius*correction_factor; //[px]
            if (debug)
                std::cout << " min_radius " << min_radius
                          << " max_radius " << max_radius
                          << " hough_threshold " << hough_threshold << std::endl;

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
            for (size_t i = 0; i < circles.size(); i++)
            {
                Centers[i].x = circles[i][0];
                Centers[i].y = circles[i][1];
                cv::Point2f center(circles[i][0], circles[i][1]);
                auto radius = circles[i][2];
                auto vote = circles[i][3];
                if (debug)
                    std::cout << " radius " << radius << " CX " << Centers[i].x << " CY "
                            << Centers[i].y << " vote : " << vote << std::endl;
                // circle center
                cv::circle(RoiImage_out, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
                cv::circle(RoiImage_out_fit, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
                // circle outline
                cv::circle(RoiImage_out, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
                cv::circle(RoiImage_out_fit, center, radius, cv::Scalar(0, 0, 255), 3, 8,
                           0);
            }
            if (debug)
                for (size_t i = 0; i < circles.size(); i++)
                {
                    std::cout << " CX " << circles[i][0]
                              << " CY " << circles[i][1]
                              << std::endl;

                    cv::Point point_A = cv::Point2f(circles[i][0], circles[i][1]);
                    cv::Point point_B = cv::Point2f(circles[(i + 1) % Centers.size()][0],
                                                    circles[(i + 1) % Centers.size()][1]);
                    std::cout << " OLD norm "
                              << cv::norm( Centers.at(i) - Centers.at((i + 1) % Centers.size()))
                              << " CXX " << Centers[(i + 1) % 4].x
                              << " CYY " << Centers[(i + 1) % 4].y
                              << std::endl;

                    std::cout << " NEW norm " << cv::norm(point_A - point_B)
                              << " CXX " << point_B.x
                              << " CYY " << point_B.y
                              << std::endl;
                }

            std::vector <std::vector <unsigned int> > Squares(0);
            std::vector <std::vector <unsigned int> > Triangles(0);

            Find_SquareAndTriangles(circles,Squares,Triangles);
            if(debug)
                std::cout<<"Squares.size() "<<Squares.size()<<std::endl;

            //define square center variables, to be used later in the fit
            double square_center_x = 0.;
            double square_center_y = 0.;
            for (size_t i = 0; i < Squares.size(); i++)
            {
                for (unsigned int j = 0; j < 4; j++)
                {
                    cv::Point point_A = cv::Point2f(circles[Squares[i][j]][0],
                                                    circles[Squares[i][j]][1]);
                    cv::Point point_B = cv::Point2f(circles[Squares[i][(j + 1) % 4]][0],
                                                    circles[Squares[i][(j + 1) % 4]][1]);
                    cv::line(RoiImage_out, point_A, point_B, cv::Scalar(0, 255, 0), 2, 8);
                }

                //            cv::Point square_center = Square_center(Centers.at(Squares.at(i).at(0)),Centers.at(Squares.at(i).at(1)),
                //                                                    Centers.at(Squares.at(i).at(2)),Centers.at(Squares.at(i).at(3)));
                cv::Point square_center = Square_center(
                        cv::Point2f(circles[Squares.at(i).at(0)][0],
                                    circles[Squares.at(i).at(0)][1]),
                        cv::Point2f(circles[Squares.at(i).at(1)][0],
                                    circles[Squares.at(i).at(1)][1]),
                        cv::Point2f(circles[Squares.at(i).at(2)][0],
                                    circles[Squares.at(i).at(2)][1]),
                        cv::Point2f(circles[Squares.at(i).at(3)][0],
                                    circles[Squares.at(i).at(3)][1]));
                cv::circle(RoiImage_out, square_center, 3, cv::Scalar(255, 0, 0), -1, 8, 0);
                cv::circle(RoiImage_out,
                           square_center, 50, // * Calibration,
                           cv::Scalar(255, 0, 0), 3, 8, 0);

                Point O_RoI(RoiImage_out.cols/2, RoiImage_out.rows/2.0);
                if (Squares.size() == 1)
                {
                    position = square_center - O_RoI;
                    cv::circle(RoiImage_out, O_RoI, 3, cv::Scalar(0, 0, 255), -1, 8, 0);

                    std::cout << "pre fit : " << square_center.x << " " << square_center.y
                            << std::endl;
                    square_center_x = square_center.x;
                    square_center_y = square_center.y;
                }
            }

            if(debug)
                cv::imshow("3 Results",RoiImage_out);

            if(!fit && Squares.size() == 1)
            {
                return position;
            }
            else if(fit && Squares.size() == 1)
            {

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
                double side = 50; //*Calibration; //side is approximately 50 um
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

                const alglib::ae_int_t ct_1[] = {
                                                 0,0,0,0,0,  0,0,0,0,0, 0,0
                };
                conditions_relation.setcontent(12,ct_1);

                double s_1[] = {
                                1,
                                1,
                                1,
                                1,
                                1,

                                1,
                                1,
                                1,

                                1,
                                1,
                                1,
                                1,

                                1,
                                1,
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
                if (int(rep.terminationtype) < 0)
                {
                    LoggerStream os;
                    os << loglevel(Log::warning) << "Fit of square failed!!" << std::endl;
                    return position;
                }
                //3 evaluate the center of the new square

                double fitted_square_center_x = starting_value_variables[12];
                double fitted_square_center_y = starting_value_variables[13];
                os << loglevel(Log::debug)
                   << " " << fitted_square_center_x
                   << " " << fitted_square_center_y
                   <<std::endl;

                Point O_RoI(RoiImage_out.cols/2, RoiImage_out.rows/2.0);
                position = Point(fitted_square_center_x, fitted_square_center_y) - O_RoI;

                //0.5 added for rounding when converting from double to int
                cv::circle(RoiImage_out_fit, cv::Point(square_center_x+0.5,square_center_y+0.5), 3, cv::Scalar(255,0,0), -1, 8, 0 );
                cv::circle(RoiImage_out_fit, cv::Point(square_center_x+0.5,square_center_y+0.5), 5, cv::Scalar(255,0,0), 2, 8, 0 );
                cv::circle(RoiImage_out_fit, cv::Point(fitted_square_center_x+0.5,fitted_square_center_y+0.5), 3, cv::Scalar(0,0,255), -1, 8, 0 );
                //cv::imwrite("EXPORT/Circles_FIT_"+one+"_"+two+"_"+dummy+".jpg",RoiImage_out_fit);
                return position;
            }
        }

        if(iterations>2)
            break;
    }//end loop on Itrations
    return position;
}

std::vector<cv::Vec4d>  FiducialFinder::OrderSquare(const std::vector<cv::Vec4d> &input){
    std::vector<cv::Vec4d>  output(4);
    //I am assuming it is a square
    if (input.size() != 4)
    {
        LoggerStream os;
        os << loglevel(Log::warning) << "Error in square reordering size." << std::endl;
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

cv::Mat FiducialFinder::prepare_image(const cv::Mat &image, int kernel_size, bool debug, const std::string &msg)
{
    cv::Mat image_gray = image.clone();

    // Select the blue component
    image_gray = get_component(image_gray,1);

    // Blur the image
    cv::medianBlur(image_gray, image_gray, kernel_size);
    if (debug)
        cv::imshow(msg + " - blur", image_gray);

    // Get a binary image
    cv::threshold(image_gray,image_gray, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    if (debug)
        cv::imshow(msg+" - blur+thr",image_gray);

    cv::Mat elem = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    cv::morphologyEx(image_gray, image_gray, cv::MORPH_CLOSE, elem);
    if (debug)
        cv::imshow(msg + " - blur+thr+close",image_gray);

    cv::adaptiveThreshold(image_gray,image_gray,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY_INV,kernel_size,2);
    if (debug)
        cv::imshow(msg + " - threshold",image_gray);

    return image_gray;
}


int FiducialFinder::FindCircles(std::vector<Circle> &out_circles,
                                double expected_R, double width_R,
                                double min_dist,
                                const Point &origin,
                                bool debug)
{
    LoggerStream os;
    out_circles.clear();

    /*
     * Check the values of r_min and r_max
     */
    double r_min=0.0, r_max=0.0;
    if ( width_R < 1.0 && width_R >0.0 )
    {
        r_min = (1.0-width_R)*expected_R;
        r_max = (1.0+width_R)*expected_R;
    }
    else
    {
        r_min = 0.0;
        r_max = 2.0*expected_R;
    }


    /*
     * Check that hte images are OK
     */
    if (image.empty())
    {
        os << loglevel(Log::error) << "Image is empty!!" << std::endl;
        return -1;
    }

    if (debug)
    {
        os << loglevel(Log::info)
           << "Image size " << image.cols << 'x' << image.rows
           << std::endl;
        os << "Expected R: " << expected_R << " - " << width_R
           << " [" << r_min << " - " << r_max << "]"
           << std::endl;
        cv::imshow("Input Image", image);
    }

    cv::Mat RoiImage;
    if (origin.is_nan())
    {
        RoiImage = image.clone();
    }
    else
    {
        /*
         * Define the center of the  Region of Interest
         */
        Point image_O = origin;
        if (origin.is_nan())
            image_O.set(image.cols/2.0, image.rows/2.0);

        /*
         * Define the center of the  Region of Interest
         */
        RoiImage = find_region_of_intetest(image_O);
        if (RoiImage.empty())
        {
            os << loglevel(Log::error) << "Window size wrongly set!!" << std::endl;
            return -2;
        }
    }

    if (min_dist<0)
    {
        if (r_min>0.0)
            min_dist = 4.0*r_min; //std::min(RoiImage.rows, RoiImage.cols)/16;
        else
            min_dist = 1.1*r_max;
    }

    cv::Mat image_gray;
    cv::cvtColor(RoiImage, image_gray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(image_gray, image_gray, get_kernel_size());


    std::vector<cv::Vec3f> circles;
    double correction_factor = 0.4;
    int hough_threshold = correction_factor*(r_min >0.0 ? r_min : r_max);
    cv::HoughCircles(image_gray, circles, cv::HOUGH_GRADIENT,
                     2, // dp
                     image_gray.rows/16,  // change this value to detect circles with different distances to each other
                     135, hough_threshold, // To investigate
                     r_min, // min radius
                     r_max // max radius
         );

    if (debug)
    {
        cv::imshow("Image Gray", image_gray);
        os << loglevel(Log::debug)
           << "Number of circles " << circles.size()
           << std::endl;
    }

    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Vec3i c = circles[i];
        Point center = Point(c[0], c[1]);
        out_circles.push_back( Circle(c[2], center) );

        if (debug)
        {
            // circle center
            cv::circle( RoiImage, center, 3, cv::Scalar(0,0,255), 3, cv::LINE_4);

            // circle outline
            int radius = c[2];
            cv::circle( RoiImage, center, radius, cv::Scalar(0, 255,0), 3, cv::LINE_4);

            os << loglevel(Log::debug)
               << i << ".- " << center << " R= " << radius << std::endl;
        }

    }

    if (debug)
        cv::imshow("Detected circles", RoiImage);

    return 0;
}

Point FiducialFinder::FindFiducial(MatrixTransform &outM, int &fail_code, const Point &origin, bool debug)
{
    /*
     * Find_F
     * main function for finding fiducials using surf
     * https://gitlab.cern.ch/guescini/fiducialFinder/blob/master/fiducialFinder.py
     *
     */
    LoggerStream os;
    Point position;
    fail_code = 0;

    //debug
    cv::Mat result;


    /*
     * Check that hte images are OK
     */
    if (image.empty())
    {
        os << loglevel(Log::error) << "Image is empty!!" << std::endl;
        fail_code = -1;
        return position;
    }

    if (image_fiducial.empty())
    {
        os << loglevel(Log::error) << "Fiducial is empty!!" << std::endl;
        fail_code = -2;
        return position;
    }

    /*
     * Define the center of the  Region of Interest
     */
    Point image_O = origin;
    if (origin.is_nan())
        image_O.set(image.cols/2.0, image.rows/2.0);

    /*
     * Define the center of the  Region of Interest
     */
    cv::Mat RoiImage = find_region_of_intetest(image_O);
    if (RoiImage.empty())
    {
        os << loglevel(Log::error) << "Window size wrongly set!!" << std::endl;
        fail_code = -3;
        return position;
    }

    if(debug)
    {
        os << loglevel(Log::info)
           << "RoI size "
           << RoiImage.cols <<  "x" << RoiImage.rows << std::endl;
        cv::imshow("f. 0 image", image);
        cv::imshow("f. 0.1 image ROI", RoiImage);
        cv::imshow("f. 0.1.f image ROI", image_fiducial);
    }

    /*
     * Prepare the image for feature detection
     */
    int kernel_size = get_kernel_size();
    int window_size = get_window_size();
    cv::Mat image_gray = prepare_image(RoiImage, kernel_size, false, "Image");
    cv::Mat image_F_gray = prepare_image(image_fiducial, kernel_size, false, "Fiducial");


    /*
     * Feature detection (use SURF)
     */
    cv::Ptr<cv::Feature2D> detector = cv::xfeatures2d::SURF::create();

    std::vector<cv::KeyPoint> keypoints_fiducial, keypoints_image;
    std::vector<cv::DMatch> matches;
    cv::Ptr<cv::DescriptorExtractor> descriptor_extractor;
    std::vector<std::vector< cv::DMatch> > nn_matches;
    cv::Mat descriptorImage, descriptorFiducial;
    detector->detectAndCompute(image_gray, cv::noArray(),
                               keypoints_image,
                               descriptorImage, false);
    detector->detectAndCompute(image_F_gray, cv::noArray(),
                               keypoints_fiducial,
                               descriptorFiducial, false);

    if (debug)
    {
        os << loglevel(Log::info) << "Fiducial keypoints " << keypoints_fiducial.size() << std::endl;
        os << loglevel(Log::info) << "Image    keypoints " << keypoints_image.size() << std::endl;
    }

    /*
     * Match the features in both images.
     * We use the Brute Force matcher.
     * https://docs.opencv.org/2.4/modules/features2d/doc/common_interfaces_of_descriptor_matchers.html
     */
    cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create();
    matcher->knnMatch(descriptorFiducial, descriptorImage,  nn_matches, 2);

    /*
     * Select "good quality matches
     * https://docs.opencv.org/3.0-beta/doc/tutorials/features2d/akaze_tracking/akaze_tracking.html
     */
    std::vector<cv::DMatch> SortedMatches;
    const double Lowe_ratio = 0.7; //loose: 0.9, tight: 0.7
    for (unsigned int j = 0; j < nn_matches.size(); j++)
    {
        if (nn_matches[j][0].distance < nn_matches[j][1].distance * Lowe_ratio)
            SortedMatches.push_back(nn_matches[j][0]);
    }



    if (debug)
    {
        cv::imshow("RoI gray: ", image_gray);
        cv::imshow("Fiducial gray: ", image_F_gray);

        cv::Mat test_1;
        cv::Mat test_2;
        cv::drawKeypoints(image_gray, keypoints_image, test_1, cv::Scalar(0,0,255));
        cv::drawKeypoints(image_F_gray, keypoints_fiducial, test_2, cv::Scalar(0,0,255));
        cv::imshow("3. keypoints image",test_1);
        cv::imshow("3. keypoints F",test_2);
        cv::drawMatches(image_F_gray, keypoints_fiducial, image_gray, keypoints_image, SortedMatches, result);
        os << loglevel(Log::info)<<" SortedMatches.size()  "<< SortedMatches.size()<<std::endl;

    }

    /**
     * Check that we have enough matches
     */
    const unsigned int min_matches = 4;//add max number of matches?
    if( SortedMatches.size() < min_matches)
    {
        os << loglevel(Log::error) << "Not reached minimum number of matches." << std::endl;
        fail_code = -5;
        cv::imshow("Result window", result);
        return position;
    }

    /*
     * Find the object. For this we compute an affine transformations
     * to move from the fiducial to the RoI image
     */
    std::vector<cv::Point2d> obj;
    std::vector<cv::Point2d> scene;
    for (unsigned int i = 0; i < SortedMatches.size(); i++)
    {
        //-- Get the keypoints from the good matches
        obj.push_back(keypoints_fiducial[SortedMatches[i].queryIdx].pt);
        scene.push_back(keypoints_image[SortedMatches[i].trainIdx].pt);
    }

    /*
     * Find the transform from Fiducial to Image
     */
    cv::Mat H = cv::estimateAffinePartial2D(obj, scene, cv::noArray(), cv::RANSAC);
    if ( H.empty() )
    {
        os << loglevel(Log::error)
           << "Could not find the Affine Transformation"
           << std::endl;
        fail_code = -6;
        return position;
    }
    outM.set( (double []){ H.at<double>(0,0), H.at<double>(1,0),
               H.at<double>(0,1), H.at<double>(1,1),
               H.at<double>(0,2), H.at<double>(1,2)} );

    /*
     * Find the fiducial bounding box
     */
    std::vector<Point> vscene;
    std::vector<Point> vobj = {Point(0,0),
                               Point(image_F_gray.cols, 0),
                               Point(image_F_gray.cols, image_F_gray.rows),
                               Point(0, image_F_gray.rows)};

    // Find the bounding box in the RoI and both centers
    //Point vscene[4];
    Point center, I_center;
    for (auto iv : vobj)
    {
        center += iv;
        vscene.push_back( outM*iv );
    }
    center /=4.0;
    I_center = outM*center;

    /*
     * The fiducial position in the image
     */
    Point O_RoI(RoiImage.cols/2, RoiImage.rows/2.0);
    Point delta = (I_center - O_RoI);

    position = delta;


    if (debug)
    {

        os << loglevel(Log::none)
                   << "Transform: \n" << outM << std::endl
                   << "Fid. center: " << center << std::endl
                   << "Image center " << I_center
                   << std::endl;


        std::vector<cv::Point2d> obj_corners(4);
        obj_corners[0] = cv::Point(0, 0);
        obj_corners[1] = cv::Point(image_F_gray.cols, 0);
        obj_corners[2] = cv::Point(image_F_gray.cols, image_F_gray.rows);
        obj_corners[3] = cv::Point(0, image_F_gray.rows);
        std::vector<cv::Point2d> scene_corners(4);
        cv::transform(obj_corners, scene_corners, H);

        Point shift(image_F_gray.cols, 0);
        cv::line(result, vscene[0] + shift, vscene[1] + shift, cv::Scalar(0, 255, 0), 4);
        cv::line(result, vscene[1] + shift, vscene[2] + shift, cv::Scalar(0, 255, 0), 4);
        cv::line(result, vscene[2] + shift, vscene[3] + shift, cv::Scalar(0, 255, 0), 4);
        cv::line(result, vscene[3] + shift, vscene[0] + shift, cv::Scalar(0, 255, 0), 4);
        cv::imshow("Result ", result);

        /*
         * Draw in the RoI image the fiducial box and the center
         */
        cv::Point F_center = Square_center(scene_corners.at(0),scene_corners.at(1),scene_corners.at(2),scene_corners.at(3));
        os << "F_center " << F_center << " I_center " << I_center << std::endl;
        cv::circle(RoiImage, I_center, 3, cv::Scalar(255,0,0), -1, 8, 0 );
        cv::line(RoiImage, vscene[0], vscene[1], cv::Scalar(0, 255, 0), 4);
        cv::line(RoiImage, vscene[1], vscene[2], cv::Scalar(0, 255, 0), 4);
        cv::line(RoiImage, vscene[2], vscene[3], cv::Scalar(0, 255, 0), 4);
        cv::line(RoiImage, vscene[3], vscene[0], cv::Scalar(0, 255, 0), 4);

        cv::Point2d RoItranslation = cv::Point2d((image.cols-window_size)*0.5, (image.rows-window_size)*0.5);
        cv::line(image, scene_corners[0] + RoItranslation, scene_corners[1] + RoItranslation, cv::Scalar(0, 255, 0), 4);
        cv::line(image, scene_corners[1] + RoItranslation, scene_corners[2] + RoItranslation, cv::Scalar(0, 255, 0), 4);
        cv::line(image, scene_corners[2] + RoItranslation, scene_corners[3] + RoItranslation, cv::Scalar(0, 255, 0), 4);
        cv::line(image, scene_corners[3] + RoItranslation, scene_corners[0] + RoItranslation, cv::Scalar(0, 255, 0), 4);
        cv::circle(image, image_O, 3, cv::Scalar(0,0,255), -1, 8, 0 );


        cv::imshow("SURF Match - RoI", RoiImage);
        cv::imshow("The final image", image);
        os << loglevel(Log::info) << "Position of fiducial " << position << std::endl;
    }

    /*
     * Move from RoI to image
     */
    position += image_O;

    /*
     * A few checks before returning
     */
    double scale = sqrt(outM[0]*outM[0] + outM[1]*outM[1]);
    if ( fabs(scale-1.0)> 0.2 )
    {
        os << loglevel(Log::error) << "Scale too far from one: " << scale << std::endl;
        fail_code = 1; //control on the scale of the fiducial, which should be close to 1
    }

    if((abs(delta.x()) > image_F_gray.cols/3) || (abs(delta.y()) > image_F_gray.rows/3))
    {
        os << loglevel(Log::error) << "Fiducial is off-center: " << scale << std::endl;
        fail_code = 2;
    }

    /*
     * clean up
     */
    descriptor_extractor.release();
    matcher.release();
    detector.release();

    return position;
}

void function1_func(const alglib::real_1d_array &x, double &func, void *ptr)
{
    //
    // this callback calculates f(x0,x1) = 100*(x0+3)^4 + (x1-3)^4
    //
    func = 100*pow(x[0]+3, 4) + pow(x[1]-3, 4);
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

    printf("%d\n", int(rep.terminationtype));  // EXPECTED: 4
    printf("%s\n", x.tostring(2).c_str());     // EXPECTED: [-3,3]
    return 0;
}
