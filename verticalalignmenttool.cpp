#include "verticalalignmenttool.h"

VerticalAlignmentTool::VerticalAlignmentTool(QWidget *parent) : QWidget(parent)
{}

VerticalAlignmentTool::~VerticalAlignmentTool()
{}

void VerticalAlignmentTool::Set_camera(const cv::VideoCapture &m_cap){
    cap = m_cap;
}


//https://stackoverflow.com/questions/34478402/opencv-how-to-count-objects-in-photo
//https://stackoverflow.com/questions/37540305/how-to-count-white-object-on-binary-image
