#ifndef FIDUCIALFINDER_H
#define FIDUCIALFINDER_H

#include <QObject>
#include <QWidget>
#include <QTime>
#include <QTextEdit>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/aruco.hpp>
#ifdef VALENCIA
#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "optimization.h"
#endif

class FiducialFinder : public QWidget
{
    Q_OBJECT
public:
    explicit FiducialFinder(QWidget *parent = nullptr);
    ~FiducialFinder();
    void Set_log(QTextEdit *m_log);
    void Set_calibration(double m_calib);

signals:
    void Log_append(QString TextToWrite);

public slots:
    void SetImage(const cv::Mat &input);
    void SetImageFiducial(const cv::Mat &input);
    void SetImage(const std::string& filename, int flags);
    void SetImageFiducial(const std::string& filename, int flags);
    bool IsImageEmpty();
    bool Find_circles(double &X_distance, double &Y_distance, const int &temp_input, const int &temp_input_2, bool fit = false);
    bool Find_F(const int &DescriptorAlgorithm, double &X_distance, double &Y_distance,
                const int &temp_input, const int &temp_input_2, std::string &timestamp, int dummy_temp,
                cv::Mat &transform_out);
    cv::Mat get_component(const cv::Mat &input_mat,const unsigned int &input);
    cv::Mat enance_contrast(const cv::Mat &input_mat, const double &alpha, const double &beta);
    cv::Mat dan_contrast(const cv::Mat &input_mat, const double &max_alpha);
    cv::Mat change_gamma(const cv::Mat &input_mat, const double &gamma);
    int dumb_test();

private:
    double Calibration = -1.1; //[px/um]
    cv::Mat image;
    cv::Mat image_fiducial;
    QTextEdit *log;

    bool Is_equal(const double &one,const double &two);
    bool Is_a_triangle(const cv::Point& P_1, const cv::Point& P_2, const cv::Point& P_3);
    bool Is_a_square(const cv::Point& P_1, const cv::Point& P_2, const cv::Point& P_3, const cv::Point& P_4);
    void Find_SquareAndTriangles(const std::vector <cv::Point> &Centers,
                                 std::vector<std::vector<unsigned int> > &Squares,
                                 std::vector<std::vector<unsigned int> > &Triangles);
    cv::Point Square_center(const cv::Point& P_1, const cv::Point& P_2,
                            const cv::Point& P_3, const cv::Point& P_4);
    void addInfo(cv::Mat &image,const std::string &algo_name, int start_x, int start_y,int text_font_size ,int text_thikness,std::string &timestamp);
    int OrderSquare(const std::vector<cv::Point2d> &input, std::vector<cv::Point2d> &output);
};

#endif // FIDUCIALFINDER_H
