#ifndef FIDUCIALFINDER_H
#define FIDUCIALFINDER_H

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
#include <opencv2/aruco.hpp>

class FiducialFinder
{
    public:
        FiducialFinder();
        ~FiducialFinder();
        void Set_calibration(double m_calib);

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


        bool Find_circles(double &X_distance, double &Y_distance, const int &temp_input,
                          const int &temp_input_2, bool fit = false, bool single = false);
        bool Find_F(const int &DescriptorAlgorithm, double &X_distance,
                    double &Y_distance, std::string &timestamp, int &fail_code,
                    const int &temp_input, const int &temp_input_2, const int &dummy_temp,
                    cv::Mat &transform_out);


        /**
         * Returs one of the RGB components of the image at imput
         * @param  input_mat The original matrix
         * @param  input     The RGB component fof the image we want
         * @return The iamge wit honly the selected component
         */
        cv::Mat get_component(const cv::Mat &input_mat, const int input);

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
        double Calibration = 1.0;  // [px/um]
        cv::Mat image;
        cv::Mat image_fiducial;
        double measured_points[8];

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
