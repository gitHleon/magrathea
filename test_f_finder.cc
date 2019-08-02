/*
 * test_f_finder.cc
 *
 *  Created on: Jul 29, 2019
 *      Author: lacasta
 */
#include <getopt.h>
#include <iostream>
#include <string>
#include "Point.h"
#include "logger.h"
#include "Fiducial_finder.h"

int main(int argc, char **argv)
{
    const char  *c_home = std::getenv("HOME");
    std::string home;
    if (c_home)
        home = c_home;

    std::string ifile, ifiducial;
    int isurf = 1;
    int icircle = 0;
    int idebug = 0;
    int option_index;
    Point origin(Point::NaN());
    double expected_R = 80.0; // size in pixels
    double range=0.1;
    double min_dist=-1;


    LoggerStream os;
    os << loglevel(Log::info) << Point(0,0) << " " << Point(1,1) << std::endl;

    static struct option long_options[] = {
             {"image", 1, 0, 'i'},
             {"fiducial", 1, 0, 'f'},
             {"radius", 1, 0, 'r'},
             {"range", 1, 0, 'w'},
             {"origin", 1, 0, 'o'},
             {"min_dist", 1, 0, 'd'},
             { "surf", 0, &isurf, 1 },
             { "debug", 0, &idebug, 1},
             { "circle", 0,&icircle, 1},
             { "help", 0, 0, 'h'},
             { 0, 0, 0, 0 } };

    while ( 1)
    {
        char c;
        option_index = 0;
        c = getopt_long(argc, argv, ":i:f:o:r:w:h:t:ud", long_options, &option_index);
        if (c == -1)
            break;
        switch (c)
        {
            case 0:
                break;
            case 'i':
                ifile = optarg;
                break;
            case 'f':
                ifiducial = optarg;
                break;
            case 'r':
                expected_R = std::stod(optarg);
                break;
            case 'w':
                range = std::stod(optarg);
                break;
            case 'd':
                min_dist = std::stod(optarg);
                break;
            case 'o':
                {
                    std::string coord(optarg);
                    std::string::size_type pos = coord.find(',');
                    double x = std::stod(coord.substr(0, pos));
                    double y = std::stod(coord.substr(pos+1));
                    origin.set(x, y);
                    std::cout << "Origin @ " << origin << std::endl;
                }
                break;
             case 'h':
                std::cout << "test-f-finder" << std::endl
                          << "--image     input file" << std::endl
                          << "--fiducial  fiducial file" << std::endl
                          << "--surf      call FindFiducials" << std::endl
                          << "--circle    call FindCircles" << std::endl
                          << "--radius    circle radius" << std::endl
                          << "--range     radius range in percentage of radius" << std::endl
                          << "--origin    'x,y' coordinates of RoI in image" << std::endl
                          << "--debug     verbose" << std::endl;
                return 0;
            default:
                std::cout << "Error " << argv[optind-1] << " ";
                if (optind<argc)
                    std::cout << argv[optind];
                std::cout << " [Ughhhh !!!!]" << std::endl;
                return 1;
        }
    }
    if (icircle)
        isurf = 0;

    bool debug = (idebug!=0);


    /* F */
    std::string input_image = home + "/Desktop/Fiducials/Image_0_1_2.jpg";
    std::string fiducial_template = home + "/Desktop/Fiducials/Templates/F.png";


    /* E
    std::string input_image = home + "/Desktop/Fiducials/Image_0_0_5.jpg";
    std::string fiducial_template = home + "/Desktop/Fiducials/Templates/E.jpg";
*/

    if (ifile.empty())
        ifile = input_image;

    if (ifiducial.empty())
        ifiducial = fiducial_template;


    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);


    FiducialFinder finder;

    finder.SetImage(ifile, cv::IMREAD_COLOR);
    if (isurf)
    {
        finder.SetImageFiducial(ifiducial, cv::IMREAD_COLOR);
        int code = 0;
        MatrixTransform outM;
        Point pos = finder.FindFiducial(outM, code, Point::NaN(), debug);

        std::cout << "SURF; " << code << ": " << pos << std::endl
                << "Transformation\n"
                << outM << std::endl;
    }
    else
    {
        std::vector<Circle> circles;
        int rc = finder.FindCircles(circles, expected_R, range, min_dist, origin, debug);
        std::cout << "CIRCLES: rc=" << rc << " No. of circles: " << circles.size() << std::endl;
        for (auto c : circles )
        {
            std::cout << c.get_center() << " - R=" << c.get_R() << std::endl;
        }
    }
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
