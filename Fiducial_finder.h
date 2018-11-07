#ifndef FIDUCIALFINDER_H
#define FIDUCIALFINDER_H

#include <QObject>
#include <QWidget>
#include <QTime>
#include <QTextEdit>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/aruco.hpp>

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
    void Find_circles(double &X_distance, double &Y_distance);
    void Find_F(const int &DescriptorAlgorithm,double &X_distance, double &Y_distance, const int &temp_input);

private:
    double Calibration = -1.1; //[px/um]
    cv::Mat image;
    cv::Mat image_fiducial;
    QTextEdit *log;

    bool Is_equal(const double &one,const double &two);
    bool Is_a_triangle(const cv::Point& P_1, const cv::Point& P_2, const cv::Point& P_3);
    bool Is_a_square(const cv::Point& P_1, const cv::Point& P_2, const cv::Point& P_3, const cv::Point& P_4);
    void Find_SquareAndTriangles(const std::vector <cv::Point> &Centers,
                                 std::vector <std::vector <int> > &Squares,
                                 std::vector <std::vector <int> > &Triangles);
    cv::Point Square_center(const cv::Point& P_1, const cv::Point& P_2,
                            const cv::Point& P_3, const cv::Point& P_4);
    void addInfo(cv::Mat &image,const std::string &algo_name, int start_x, int start_y,int text_font_size ,int text_thikness,std::string &timestamp);
};

#endif // FIDUCIALFINDER_H
