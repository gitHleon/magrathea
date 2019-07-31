#ifndef FIDUCIALFINDER_H
#define FIDUCIALFINDER_H

#include <string>
#include <vector>
#ifdef VALENCIA
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <fstream>
#endif

#include <ap.h>
#include <optimization.h>

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <MatrixTransform.h>

class Circle;
class Circle
{
    private:
        double R;
        Point center;

        void cpy(const Circle &C)
        {
            R = C.R;
            center = C.center;
        }
    public:
        Circle() : R(0.0), center(Point::NaN()) {}
        Circle(double r, Point C)
            : R(r), center(C) {}

        Circle(double r, double x, double y)
            : R(r), center(x, y) {}

        Circle(const Circle &C)
        {
            if (&C != this)
                cpy(C);
        }

        Point get_center() const { return center; }
        double get_R() const { return R; }
};

class FiducialFinder
{
    public:
        FiducialFinder();
        ~FiducialFinder();

        /**
        * These are a set of methods to give the input image to FiducialFinder
        * @param input [description]
        */
        void SetImage(const cv::Mat &input);
        void SetImageFiducial(const cv::Mat &input);
        void SetImage(const std::string &filename, int flags);

        /**
         * Sets the reference image
         * @param filename [description]
         * @param flags    [description]
         */
        void SetImageFiducial(const std::string &filename, int flags);

        /**
         * Checks the emptiness fo the image
         * @return True if the image is empty
         */
        bool IsImageEmpty();

        cv::Mat find_region_of_intetest(const Point &origin);

        int get_kernel_size() const;

        int get_window_size() const;

        Point Find_circles_(bool fit = false, bool single = false,
                           const Point &origin=Point::NaN(), bool debug = false);


        /**
         * Finds circles in the image. It uses OpenCV Hough Circle Transform based on
         * the paper by H.K. Yuen (doi:10.5244/C.3.29)
         *
         * @param  circles    A vector of Circles that wil lreceive the coordinates of
         *                    the circles
         * @param  expected_R Expected value of the radius
         * @param  R_width    This expresses the allowed range of values for the radius
         *                    in percentage. If >1 or <0, the rane is [0-2*R]
         * @param  min_dist   minimum distance among circles
         * @param  debug      set to true to debug the process
         * @return            the error code. =0 if success
         *
         */
        int FindCircles(std::vector<Circle> &circles,
                        double expected_R, double R_width=0.0, double min_dist=-1,
                        const Point &origin=Point::NaN(),
                        bool debug = false);

        /**
         * find the fiducial given in SetImageFiducial in the Image given in SetImage
         * @param  outM      Transform to go from fiducial to image
         * @param  fail_code 0 if not problem found.
         * @param  debug     set to true to debug the process
         * @param  origin    the position of the region of interest
         * @return           position of the fiducial
         */
        Point FindFiducial(MatrixTransform &outM, int &fail_code,
                           const Point &origin=Point::NaN(), bool debug = false);


        /**
         * Returns one of the RGB components of the image at imput
         * @param  input_mat The original matrix
         * @param  input     The RGB component fof the image we want
         * @return The iamge wit honly the selected component
         */
        static cv::Mat get_component(const cv::Mat &input_mat, const int input);

        /**
         * Prepares the image for feature detection
         * @param  image       Original image
         * @param  kernel_size Kernel size for bluring
         * @param  debug       true for debug
         * @param  msg         Prefix to prepend in debug messages
         * @return             AN OPENCV Mat
         */
        static cv::Mat
        prepare_image(const cv::Mat &image, int kernel_size,
                      bool debug = false, const std::string &msg = "");


        /**
         * [enance_contrast description]
         * @param  input_mat [description]
         * @param  alpha     [description]
         * @param  beta      [description]
         * @return           [description]
         */
        cv::Mat enance_contrast(const cv::Mat &input_mat, const double &alpha,
                                const double &beta);

        /**
         * [dan_contrast description]
         * @param  input_mat [description]
         * @param  max_alpha [description]
         * @return           [description]
         */
        cv::Mat dan_contrast(const cv::Mat &input_mat, const double &max_alpha);

        /**
         * [change_gamma description]
         * @param  input_mat [description]
         * @param  gamma     [description]
         * @return           [description]
         */
        cv::Mat change_gamma(const cv::Mat &input_mat, const double &gamma);
        int dumb_test();

    private:
        cv::Mat image;
        cv::Mat image_fiducial;

        /**
         * [Is_equal description]
         * @param  one [description]
         * @param  two [description]
         * @return     [description]
         */
        bool Is_equal(const double &one, const double &two);

        /**
         * [Is_a_triangle description]
         * @param  P_1 [description]
         * @param  P_2 [description]
         * @param  P_3 [description]
         * @return     [description]
         */
        bool Is_a_triangle(const cv::Point2d &P_1, const cv::Point2d &P_2,
                           const cv::Point2d &P_3);

        /**
         * [Is_a_square description]
         * @param  P_1 [description]
         * @param  P_2 [description]
         * @param  P_3 [description]
         * @param  P_4 [description]
         * @return     [description]
         */
        bool Is_a_square(const cv::Point2d &P_1, const cv::Point2d &P_2,
                         const cv::Point2d &P_3, const cv::Point2d &P_4);

        /**
         * [Find_SquareAndTriangles description]
         * @param Centers   [description]
         * @param Squares   [description]
         * @param Triangles [description]
         */
        void Find_SquareAndTriangles(const std::vector<cv::Vec4f> &Centers,
                                     std::vector<std::vector<unsigned int> > &Squares,
                                     std::vector<std::vector<unsigned int> > &Triangles);

        /**
         * [Square_center description]
         * @param  P_1 [description]
         * @param  P_2 [description]
         * @param  P_3 [description]
         * @param  P_4 [description]
         * @return     [description]
         */
        cv::Point2d Square_center(const cv::Point2d &P_1, const cv::Point2d &P_2,
                                  const cv::Point2d &P_3, const cv::Point2d &P_4);

        /**
         * [addInfo description]
         * @param image          [description]
         * @param algo_name      [description]
         * @param start_x        [description]
         * @param start_y        [description]
         * @param text_font_size [description]
         * @param text_thikness  [description]
         * @param timestamp      [description]
         */
        void addInfo(cv::Mat &image, const std::string &algo_name, int start_x,
                     int start_y, int text_font_size, int text_thikness,
                     std::string &timestamp);

        /**
         * [OrderSquare description]
         * @param input [description]
         */
        std::vector<cv::Vec4d> OrderSquare(const std::vector<cv::Vec4d> &input);

};

#endif // FIDUCIALFINDER_H
