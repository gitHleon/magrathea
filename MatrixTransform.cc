/*
 * MatrixTransform.cc
 *
 *  Created on: Jul 19, 2019
 *      Author: lacasta
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include "MatrixTransform.h"



MatrixTransform MatrixTransform::matrix_multiply(const MatrixTransform &M1, const MatrixTransform &M2)
{
    double val[6] =
    {
        M1._M[0]*M2._M[0]+M1._M[2]*M2._M[1], M1._M[1]*M2._M[0]+M1._M[3]*M2._M[1],
        M1._M[0]*M2._M[2]+M1._M[2]*M2._M[3], M1._M[1]*M2._M[2]+M1._M[3]*M2._M[3],
        M1._M[0]*M2._M[4]+M1._M[2]*M2._M[5]+M1._M[4], M1._M[1]*M2._M[4]+M1._M[3]*M2._M[5]+M1._M[5]
    };
    return MatrixTransform(val);
}
MatrixTransform::MatrixTransform()
    : _M{1.0, 0.0, 0.0, 1.0, 0.0, 0.0}
{
    // TODO Auto-generated constructor stub

}

MatrixTransform::MatrixTransform(double *ptr)
{
    set(ptr);
}


MatrixTransform::MatrixTransform(const MatrixTransform &T)
    : _M{1.0, 0.0, 0.0, 1.0, 0.0, 0.0}
{
    for (int i=0; i<6; ++i)
        _M[i] = T._M[i];
}

MatrixTransform::~MatrixTransform()
{
    // TODO Auto-generated destructor stub
}

void MatrixTransform::set(double *ptr)
{
    for (int i=0; i<6; ++i)
        _M[i] = ptr[i];
}

MatrixTransform MatrixTransform::identity()
{
    double id[6] = {1.0, 0.0, 0.0, 1.0, 0.0, 0.0};
    return MatrixTransform((double *)id);
}

MatrixTransform MatrixTransform::rotation() const
{
    double val[6];
    for (int i=0; i<4; i++)
        val[i] = _M[i];

    val[4] = val[5] = 0.0;

    return MatrixTransform(val);
}

MatrixTransform MatrixTransform::operator*(const MatrixTransform &M)
{
    return matrix_multiply(*this, M);
}


MatrixTransform operator*(const MatrixTransform &a, const MatrixTransform &b)
{
    return MatrixTransform::matrix_multiply(a,  b);
}

Point MatrixTransform::operator*(const Point &P) const
{
    return Point(_M[0]*P.x() + _M[2]*P.y() + _M[4], _M[1]*P.x() + _M[3]*P.y() + _M[5]);
}

MatrixTransform& MatrixTransform::translate(double tx, double ty)
{
    double v[] = { 1.0, 0.0, 0.0, 1.0, tx, ty };
    MatrixTransform T(v);
    *this = matrix_multiply(T, *this);
    return *this;
}

MatrixTransform &MatrixTransform::translate(const Point &P)
{
    return translate(P.x(), P.y());
}

MatrixTransform& MatrixTransform::scale(double sx, double sy)
{
    double v[] = { sx, 0.0, 0.0, sy, 0.0, 0.0 };
    MatrixTransform S(v);
    *this = matrix_multiply(S, *this);
    return *this;
}

MatrixTransform& MatrixTransform::skew_x(double angle)
{
    double v[] = { 1.0, 0.0, tan(angle), 1.0, 0.0, 0.0 };
    MatrixTransform S(v);
    *this = matrix_multiply(S, *this);
    return *this;
}

MatrixTransform& MatrixTransform::skew_y(double angle)
{
    double v[] = { 1.0, tan(angle), 0.0, 1.0, 0.0, 0.0 };
    MatrixTransform S(v);
    *this = matrix_multiply(S, *this);
    return *this;
}
MatrixTransform& MatrixTransform::rotate(double angle)
{
    double ca = cos(angle);
    double sa = sin(angle);
    double v[] = { ca, sa, -sa, ca, 0.0, 0.0 };
    MatrixTransform S(v);
    *this = matrix_multiply(S, *this);
    return *this;
}
MatrixTransform& MatrixTransform::rotate_around_point(double angle, const Point &P)
{
    return translate(-P.x(), -P.y()).rotate(angle).translate(P.x(), P.y());
}
MatrixTransform MatrixTransform::inverse()
{
    double det = _M[0]*_M[3]-_M[1]*_M[2];
    double A[] = {_M[3]/det, -_M[1]/det, -_M[2]/det, _M[0]/det, 0.0, 0.0 };
    A[4] = -A[0]*_M[4]-A[2]*_M[5];
    A[5] = -A[1]*_M[4]-A[3]*_M[5];

    return MatrixTransform(A);
}

std::ostream& operator<<(std::ostream &os, const MatrixTransform &M)
{
    auto old = os.setf(std::ios_base::fixed, std::ios::floatfield);
    auto oldp = os.precision(5);
    auto oldw = os.width(11);
    os << std::fixed << std::setw(11) << std::setprecision(5) << M[0] << "  " << M[2] << "  " << M[4] << std::endl
       << std::fixed << std::setw(11) << std::setprecision(5) << M[1] << "  " << M[3] << "  " << M[5] << std::endl;
    os.flush();
    os.setf(old);
    os.precision(oldp);
    os.width(oldw);
    return os;
}

