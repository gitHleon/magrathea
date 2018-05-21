#ifndef FIDUCIALFINDER_H
#define FIDUCIALFINDER_H

#include <QObject>
#include <QWidget>
#include <QTextEdit>
#include <opencv2/opencv.hpp>

class FiducialFinder : public QWidget
{
    Q_OBJECT
public:
    explicit FiducialFinder(QWidget *parent = nullptr);
    ~FiducialFinder();
    void Set_camera(const cv::VideoCapture &m_cap);
    void Set_log(QTextEdit *m_log);
    void Set_calibration(double m_calib);

signals:
    void Log_append(QString TextToWrite);

public slots:
    void SetImage(const cv::Mat &input);
    void SetImage(const std::string& filename, int flags);
    bool IsImageEmpty();

private:
    double Calibration = -1.1; //[px/um]
    cv::VideoCapture cap;
    cv::Mat image;
    QTextEdit *log;
};

#endif // FIDUCIALFINDER_H
