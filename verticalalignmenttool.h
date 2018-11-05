#ifndef VERTICALALIGNMENTTOOL_H
#define VERTICALALIGNMENTTOOL_H

#include <QObject>
#include <QWidget>
#include <QThread>
#include <QTextEdit>
#include <opencv2/opencv.hpp>
class VerticalAlignmentTool : public QWidget
{
    Q_OBJECT
public:
    explicit VerticalAlignmentTool(QWidget *parent = nullptr);
    ~VerticalAlignmentTool();
    void Set_camera(const cv::VideoCapture &m_cap);
    void Evaluate_vignette();

private:
    cv::VideoCapture cap;
    void EvaluateEccentricity(const cv::RotatedRect &box, double &Eccentricity);
    void DrawCross(cv::Mat &img_input, const cv::Point &center, const cv::Scalar &color);

};

#endif // VERTICALALIGNMENTTOOL_H
