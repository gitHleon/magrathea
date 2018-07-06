#include "fiducial_locations.h"

fiducial_locations::fiducial_locations()
{}

fiducial_locations::~fiducial_locations()
{}

void fiducial_locations::initialise()
{
//    locations["P01"] = cv::Point2f(0.,1.);
    fid_point temp;
    temp.identifier = "P00"; temp.coordinate = cv::Point2f(0.,0.);
    locations.push_back(temp);
    locations.push_back(fid_point("P01",cv::Point2f(586.5250,131.1046)));
    locations.push_back(fid_point("R0_01",cv::Point2f(0.9637,36.0329)));
    locations.push_back(fid_point("R0_02",cv::Point2f(104.8008,48.0958)));
    locations.push_back(fid_point("R0_03",cv::Point2f(104.6681,-49.4183)));
    locations.push_back(fid_point("R0_04",cv::Point2f(0.4858,-40.7877)));
    std::cout<<"ok1 : "<<locations.size()<<std::endl;
}

bool fiducial_locations::get_value(unsigned int index, std::string &m_identifier,cv::Point2f &m_coordinate)
{
    std::cout<<"ok2 : "<<locations.size()<<std::endl;
    if(index >= locations.size())
        return false;
    std::cout<<"ok3"<<std::endl;
    m_coordinate = locations[index].coordinate;
    m_identifier = locations[index].identifier;
    std::cout<<"identifier: "<<locations[index].identifier<<" ; x: "<<locations[index].coordinate.x<<" y: "<<locations[index].coordinate.y<<std::endl;
    return true;
}

