#ifndef FIDUCIAL_LOCATIONS_H
#define FIDUCIAL_LOCATIONS_H
//#include<unordered_map>
#include <vector>
#include <opencv2/opencv.hpp>

//typedef std::unordered_map<std::string,cv::Point2f> m_locations;

struct fid_point{
    std::string identifier;
    cv::Point2f coordinate;
    fid_point() : identifier("null"), coordinate(-100.,-100.)
    {}
    fid_point(std::string m_identifier,cv::Point2f m_coordinate)
    {
        identifier = m_identifier;
        coordinate = m_coordinate;
    }
};

class fiducial_locations
{
public:
    fiducial_locations();
    ~fiducial_locations();
    void initialise();
    bool get_value(unsigned int index, std::string &m_identifier, cv::Point2f &m_coordinate);

private:
//    m_locations locations;
    std::vector<fid_point> locations;
};

#endif // FIDUCIAL_LOCATIONS_H
