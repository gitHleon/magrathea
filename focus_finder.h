#ifndef FOCUS_FINDER_H
#define FOCUS_FINDER_H

/******************************************************************** /
 * Focus_finder V2.0 Daniele Madaffari xx/05/2018
 * Class with functions to serach the height of the focus of the camera.
 *
 * The main function find_focus() scans several heights for the coord
 * giving the image with highest laplacian variance. At last itaration
 * a pol(2) is performed to find the point giving the best value.
 *
 * Three systematic functions perform scans in time and distance
 * to study the response of the camera.
 *
 * Comments at daniele.madaffari@cern.ch
 *
 * *******************************************************************/

#include <QThread>
#include <QObject>
#include <QTime>
#include <QElapsedTimer>
#include <opencv2/opencv.hpp>
#ifdef VANCOUVER
#include <AerotechMotionhandler.h>
#elif VALENCIA
#include <ACSCMotionHandler.h>
#endif
#include <QTextEdit>
#include <vector>
#include <fstream>

#ifndef SLEEPER_CLASS
#define SLEEPER_CLASS
class Sleeper : public QThread
{
public:
    static void usleep(unsigned long usecs){QThread::usleep(usecs);}
    static void msleep(unsigned long msecs){QThread::msleep(msecs);}
    static void sleep(unsigned long secs){QThread::sleep(secs);}
};
#endif // SLEEPER_CLASS

class Focus_finder : public QObject
{
    Q_OBJECT
public:
    explicit Focus_finder(QObject *parent = nullptr);
    ~Focus_finder();
    void Set_camera(const cv::VideoCapture &m_cap);
    void Set_log(QTextEdit *m_log);
    void Set_ksize(int ksize);
    void Set_gantry(MotionHandler *m_gantry);
    void Set_color_int(const int &value);

signals:
    void Log_append(QString TextToWrite);
    void Log_write(QString TextToWrite);

public slots:
    void   eval_stddev(const cv::Mat &input_image, std::vector<double> &output);
    void   eval_stddev_ROI(const cv::Mat &input_image, std::vector<double> &output);
    bool   find_focus(double &focus_height);
    void   Eval_syst_time();
    bool Eval_syst_scan();


private slots:
    cv::Mat  get_component(const cv::Mat &input_mat,const unsigned int &input);
    cv::Rect get_rect(const cv::Mat &input_image);
    cv::Mat  get_frame_from_camera();

private:
    cv::VideoCapture cap;
    cv::Mat image;
    MotionHandler *gantry;
    const int measure_points = 7;
    const unsigned int z_pos_index = 4;
    const int window_size = 2700;
    int color_int = -1;
    int ksize = 5;
    double x[7] = {};
    double y[7] = {};
    double EvalVertex_x(double a,double b, double c);
    double EvalVertex_y(double a,double b, double c);
    //void perform_fit(double &z_output);
    QTextEdit *log;
    void addInfo(cv::Mat &image, const std::string &algo_name, int start_x, int start_y, int text_font_size , int text_thikness, std::string &timestamp, const std::vector<double> &values);
};

#endif // FOCUS_FINDER_H
