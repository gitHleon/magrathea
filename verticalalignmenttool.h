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

private:
    cv::VideoCapture cap;
    cv::Mat image;

};

#endif // VERTICALALIGNMENTTOOL_H
