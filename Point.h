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
#include <limits>
#include <vector>
#include <opencv2/core/types.hpp>

class Point;
Point operator*(double v, const Point &P);
Point operator+(const Point &A, const Point &B);
Point operator-(const Point &A, const Point &B);
std::ostream &operator<<(std::ostream &os, const Point &P);

/*
 * This class represents a Point. It is mainly intended as a 2D Point,
 * and the operations are just 2D. However it can store a third
 * coordinate since we assume it "oeprates" on the petal surface.
 */
class Point
{
    private:
        double _v[3];

        void cpy(const double *p)
        {
            for (int i=0; i<3; ++i)
                _v[i] = p[i];
        }
    public:
        // Default constructor
        Point(): _v{ 0.0, 0.0, 0.0 } {}

        // Constructor from double pointer
        Point(double *p) { cpy(p);}

        // Constructor from coordinates
        Point(double x, double y, double z=0.0): _v{ x, y, z} {}

        // Copy constructor
        Point(const Point &P) { cpy(P._v); }

        // return a NaN point
        static Point NaN()
        {
            return Point(std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::quiet_NaN());
        }

        // Assignment operator
        Point &operator=(const Point &P)
        {
            if (this != &P)
                cpy(P._v);

            return *this;
        }

        /**
         * Constructors from OpenCV vectors
         */
        template<typename _Tp>
        Point(const cv::Point_<_Tp> &P)
        {
            _v[0] = P.x; _v[1] = P.y; _v[2] = 0.0;
        }

        template<typename _Tp>
        Point(const cv::Point3_<_Tp> &P)
        {
            _v[0] = P.x; _v[1] = P.y; _v[2] = P.z;
        }

        /*
         * vector<_Tp>
         * We silently assume that _Tp are objects tha tcanbe casted to a double
         */
        template<typename _Tp>
        Point(const std::vector<_Tp> &V)
        {
            _v[0] = V[0];
            _v[1] = V[1];
            if (V.size()>2)
                _v[2] = V[2];
        }

        /*
         * Typecast operators
         */
        template<typename _Tp>
        operator cv::Point_<_Tp>() const { return cv::Point_<_Tp>(x(), y()); }

        template<typename _Tp>
        operator cv::Point3_<_Tp>() const { return cv::Point_<_Tp>(x(), y(), z()); }


        /*
         * Getters/Setters
         */
        double x() const { return _v[0]; }
        double y() const { return _v[1]; }
        double z() const { return _v[2]; }
        double* ptr() {  return _v; }

        void x(double v) { _v[0] = v; }
        void y(double v) { _v[1] = v; }
        void z(double v) { _v[2] = v; }
        void set(double vx, double vy, double vz=0.0)
        {
            _v[0] = vx;
            _v[1] = vy;
            _v[2] = vz;
        }

        /*
         * Check if values are OK
         */
        bool is_nan() const
        {
            return std::isnan(_v[0]) || std::isnan(_v[1] || std::isnan(_v[2]) );
        }

        /*
         * Operations, mainly in the XY plane
         */
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

        Point operator *(double v)
        {
            return Point(_v[0]*v, _v[1]*v);
        }
        Point operator /(double v)
        {
            return Point(_v[0]/v, _v[1]/v);
        }

        Point& operator *=(double v)
        {
            _v[0] *= v;
            _v[1] *= v;
            return *this;
        }
        Point& operator /=(double v)
                        {
            _v[0] /= v;
            _v[1] /= v;
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

        // Return unit vector. Z coordinate remains untouched
        Point unit() const
        {
            double vn = mag();
            return Point(x() / vn, y() / vn, z());
        }

        // return point rotated +90 deg
        Point cw() const
        {
            return Point(-y(), x(), z());
        }

        // return point rotated -90 deg
        Point ccw() const
        {
            return Point(y(), -x(), z());
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

/*
 * scalar multiplication
 */
inline Point operator*(double v, const Point &P)
{
    return Point(v*P.x(), v*P.y());
}
inline double operator*(const Point &A, const Point &B)
{
    return A.dot(B);
}

/*
 * Point addition
 */
inline Point operator+(const Point &A, const Point &B)
{
    return Point(A.x()+B.x(), A.y()+B.y());
}

/*
 * Point difference
 */
inline Point operator-(const Point &A, const Point &B)
{
    return Point(A.x()-B.x(), A.y()-B.y());
}

/*
 * Calculates difference norm
 */
inline double norm(const Point &A, const Point &B)
{
    return (A-B).mag2();
}


#endif /* POINT_H_ */
