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
#include <opencv2/opencv.hpp>
#ifdef VANCOUVER
#include <AerotechMotionhandler.h>
#elif VALENCIA
#include <ACSCMotionHandler.h>
#endif
#include <QTextEdit>

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
    void Set_gantry(MotionHandler *m_gantry);


signals:
    void Log_append(QString TextToWrite);
    void Log_write(QString TextToWrite);

public slots:
    double eval_stddev(const cv::Mat &input_image);
    void   find_focus(double &focus_height);
    void   Eval_syst_time();
    void   Eval_syst_scan();
    void   Eval_syst_moving();

private:
    cv::VideoCapture cap;
    cv::Mat image;
    MotionHandler *gantry;
    const int measure_points = 6;
    const unsigned int z_pos_index = 2;
    const int window_size = 1000;
    double x[6] = {};
    double y[6] = {};
    double EvalVertex_x(double a,double b, double c);
    double EvalVertex_y(double a,double b, double c);
    void perform_fit(double &z_output);
    QTextEdit *log;

};

#endif // FOCUS_FINDER_H
