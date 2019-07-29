/*
 * test_f_finder.cc
 *
 *  Created on: Jul 29, 2019
 *      Author: lacasta
 */

#include <string>
#include "logger.h"
#include "Fiducial_finder.h"

enum Algorithms
{
    SURF,
    SIFT,
    ORB,
    AKAZE,
    STAR,
    LAST
};
int main(int argc, char **argv)
{
    std::string home = std::getenv("HOME");
    std::string input_image = home + "/Desktop/Fiducials/SURF_F_SUCCES.jpg";
    std::string fiducial_template = home + "/Desktop/Fiducials/Templates/ATLAS_F.jpg";


    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    FiducialFinder finder;


    finder.SetImageFiducial(fiducial_template, cv::IMREAD_COLOR);
    finder.SetImage(input_image, cv::IMREAD_COLOR);
    finder.Set_calibration(1.0);

    int code=0;
    double x, y;
    std::string timestamp;
    cv::Mat output_H;
    auto rc = finder.Find_F(SURF, x, y, timestamp, code, 0, 0, 0, output_H);

    std::cout << "Aqui; " << rc << ": " << x << ", " << y << std::endl;

    cv::waitKey(0);
    return 0;
}

//function for image debugging in opencv
// This is in Calibrator. Need to find a proper place
std::string type2str(int type)
{
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth)
    {
        case CV_8U:
            r = "8U";
            break;
        case CV_8S:
            r = "8S";
            break;
        case CV_16U:
            r = "16U";
            break;
        case CV_16S:
            r = "16S";
            break;
        case CV_32S:
            r = "32S";
            break;
        case CV_32F:
            r = "32F";
            break;
        case CV_64F:
            r = "64F";
            break;
        default:
            r = "User";
            break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}
