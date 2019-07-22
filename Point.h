/*
 * Point.h
 *
 *  Created on: Jul 19, 2019
 *      Author: lacasta
 */

#ifndef POINT_H_
#define POINT_H_

#include <iosfwd>
#include <cmath>
class Point;
Point operator*(double v, const Point &P);
Point operator+(const Point &A, const Point &B);
Point operator-(const Point &A, const Point &B);
std::ostream &operator<<(std::ostream &os, const Point &P);

/*
 * This class represents a Point
 */
class Point
{
    private:
        double _v[2];

    public:
        Point(): _v{ 0.0, 0.0 } {}

        Point(double *p)
        {
            _v[0] = p[0];
            _v[1] = p[1];
        }
        Point(double x, double y): _v{ x, y} {}

        Point(const Point &P)
        {
            _v[0] = P._v[0];
            _v[1] = P._v[1];
        }

        double x() const
        {
            return _v[0];
        }
        double y() const
        {
            return _v[1];
        }
        void x(double v)
        {
            _v[0] = v;
        }
        void y(double v)
        {
            _v[1] = v;
        }
        double* ptr()
        {
            return _v;
        }

        Point operator-() const
        {
            return Point(-x(), -y());
        }

        Point& operator +=(const Point &P)
        {
            _v[0] += P._v[0];
            _v[1] += P._v[1];
            return *this;
        }
        Point& operator -=(const Point &P)
        {
            _v[0] -= P._v[0];
            _v[1] -= P._v[1];
            return *this;
        }

        Point& operator *(double v)
        {
            _v[0] *= v;
            _v[1] *= v;
            return *this;
        }

        double mag2() const
        {
            return _v[0] * _v[0] + _v[1] * _v[1];
        }
        double mag() const
        {
            return sqrt(mag2());
        }

        // REturn unit vector
        Point norm() const
        {
            double vn = mag();
            return Point(x() / vn, y() / vn);
        }

        // return pooint rotated +90 deg
        Point cw() const
        {
            return Point(-y(), x());
        }

        // return point rotated -90 deg
        Point ccw() const
        {
            return Point(y(), -x());
        }
        double dot(const Point &P) const
        {
            return _v[0] * P._v[0] + _v[1] * P._v[1];
        }
        double cross(const Point &b) const
        {
            return dot(b.cw());
        }
        double phi() const
        {
            return atan2(y(), x());
        }
        double distance(const Point &P) const
        {
            return (*this - P).mag();
        }

};

inline Point operator*(double v, const Point &P)
{
    return Point(v*P.x(), v*P.y());
}
inline double operator*(const Point &A, const Point &B)
{
    return A.dot(B);
}


inline Point operator+(const Point &A, const Point &B)
{
    return Point(A.x()+B.x(), A.y()+B.y());
}

inline Point operator-(const Point &A, const Point &B)
{
    return Point(A.x()-B.x(), A.y()-B.y());
}


#endif /* POINT_H_ */
