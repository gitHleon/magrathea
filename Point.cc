/*
 * Point.cc
 *
 *  Created on: Jul 30, 2019
 *      Author: lacasta
 */

#include <iostream>
#include <iomanip>
#include "Point.h"

std::ostream& operator<<(std::ostream &os, const Point &P)
{
    auto old = os.setf(std::ios_base::fixed, std::ios::floatfield);
    auto oldp = os.precision(3);
    auto oldw = os.width(9);
    auto oldf = os.fill(' ');

    os << "Point(" << std::right << std::setfill(' ')  << P.x() << "," << P.y() << "," << P.z() << ")";
    os.flush();
    os.setf(old);
    os.precision(oldp);
    os.width(oldw);
    os.fill(oldf);
    return os;
}




