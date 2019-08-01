/*
 * alglibtools.cc
 *
 *  Created on: Jul 18, 2019
 *      Author: lacasta
 */

#include "alglibtools.h"
#include <iomanip>
#include <iostream>
#include <limits>
#include <linalg.h>
#include <statistics.h>

std::ostream&
operator<<(std::ostream &os, const alglib::real_2d_array &M)
{
    auto old = os.setf(std::ios_base::fixed, std::ios::floatfield);
    auto oldp = os.precision(5);
    auto oldw = os.width(11);
    for (int irow = 0; irow < M.rows(); ++irow)
    {
        for (int icol = 0; icol < M.cols(); ++icol)
        {
            os << std::fixed << std::setw(11) << std::setprecision(5)
               << M(irow, icol) << " ";
        }
        os << '\n';
    }
    os.flush();
    os.setf(old);
    os.precision(oldp);
    os.width(oldw);
    return os;
}

alglib::real_1d_array real_1d_array_with_value(int n, double val)
{
    alglib::real_1d_array V;
    V.setlength(n);
    for (int i = 0; i < n; i++)
        V(i) = val;

    return V;
}

double vdot(const alglib::real_1d_array &a, const alglib::real_1d_array &b)
{
    int n = std::min(a.length(), b.length());
    return alglib::vdotproduct(a.getcontent(), b.getcontent(), n);
}

alglib::real_1d_array operator*(double a, const alglib::real_1d_array &W)
{
    alglib::real_1d_array V(W);
    int n = V.length();
    for (int i = 0; i < n; ++i)
        V(i) = a * V(i);

    return V;
}

alglib::real_1d_array operator*(const alglib::real_1d_array &W, double a)
{
    alglib::real_1d_array V(W);
    int n = V.length();
    for (int i = 0; i < n; ++i)
        V(i) = a * V(i);

    return V;
}

alglib::real_1d_array cross(const alglib::real_1d_array &a,
                            const alglib::real_1d_array &b)
{
    double vout[] = { a(1) * b(2) - b(1) * a(2),
                      a(2) * b(0) - b(2) * a(0),
                      a(0) * b(1) - b(0) * a(1) };
    alglib::real_1d_array V;
    V.setcontent(3, vout);
    return V;
}

double vnorm(const alglib::real_1d_array &a)
{
    double v = sqrt(vdot(a, a));
    return v;
}

alglib::real_2d_array ones(int nrow, int ncol)
{
    alglib::real_2d_array M;
    M.setlength(nrow, ncol);
    for (int irow = 0; irow < nrow; ++irow)
        for (int icol = 0; icol < ncol; ++icol)
            M[irow][icol] = 1.0;

    return M;
}

alglib::real_2d_array zeros(int nrow, int ncol)
{
    alglib::real_2d_array M;
    M.setlength(nrow, ncol);
    for (int irow = 0; irow < nrow; ++irow)
        for (int icol = 0; icol < ncol; ++icol)
            M[irow][icol] = 0.0;

    return M;
}

double vdot(const alglib::real_2d_array &a, const alglib::real_2d_array &b)
                    throw (AlglibException)
{
    double val = 0.0;
    if (a.cols() == 1 && b.cols() == 1)
    {
        int n = std::min(a.rows(), b.rows());
        for (int i = 0; i < n; i++)
            val += a[i][0] * b[i][0];
    }
    else if (a.rows() == 1 && b.rows() == 1)
    {
        int n = std::min(a.cols(), b.cols());
        for (int i = 0; i < n; i++)
            val += a[0][i] * b[0][i];
    }
    else
        throw AlglibException("Vdot does not work with 2D matrices, only vectors");

    return val;
}

double vnorm(const alglib::real_2d_array &a)
{
    double v = sqrt(vdot(a, a));
    return v;
}

alglib::real_2d_array cross(const alglib::real_2d_array &a,
                            const alglib::real_2d_array &b) throw (AlglibException)
{
    alglib::real_2d_array V;
    if (a.cols() == 1 && b.cols() == 1)
    {
        if (a.rows() != 3 || b.rows() != 3)
            throw AlglibException("cross only defined for dim-3 vectors");

        V.setlength(3, 1);
        V(0, 0) = a(1, 0) * b(2, 0) - b(1, 0) * a(2, 0);
        V(1, 0) = a(2, 0) * b(0, 0) - b(2, 0) * a(0, 0);
        V(2, 0) = a(0, 0) * b(1, 0) - b(0, 0) * a(1, 0);
    }
    else if (a.rows() == 1 && b.rows() == 1)
    {
        if (a.cols() != 3 || b.cols() != 3)
            throw AlglibException("cross only defined for dim-3 vectors");

        V.setlength(1, 3);
        V(0, 0) = a(0, 1) * b(0, 2) - b(0, 1) * a(0, 2);
        V(0, 1) = a(0, 2) * b(0, 0) - b(0, 2) * a(0, 0);
        V(0, 2) = a(0, 0) * b(0, 1) - b(0, 0) * a(0, 1);
    }
    else
        throw AlglibException("Vcross does not work with 2D matrices, only vectors");

    return V;
}

alglib::real_2d_array operator/(const alglib::real_2d_array &W, double a)
{
    alglib::real_2d_array V(W);
    int ncol = V.cols();
    int nrow = V.rows();
    for (int irow = 0; irow < nrow; ++irow)
        for (int icol = 0; icol < ncol; ++icol)
            V(irow, icol) = V(irow, icol) / a;

    return V;
}

alglib::real_2d_array operator*(const alglib::real_2d_array &W, double a)
{
    alglib::real_2d_array V(W);
    int ncol = V.cols();
    int nrow = V.rows();
    for (int irow = 0; irow < nrow; ++irow)
        for (int icol = 0; icol < ncol; ++icol)
            V(irow, icol) = a * V(irow, icol);

    return V;
}

alglib::real_2d_array operator*(double a, const alglib::real_2d_array &W)
{
    alglib::real_2d_array V(W);
    int ncol = V.cols();
    int nrow = V.rows();
    for (int irow = 0; irow < nrow; ++irow)
        for (int icol = 0; icol < ncol; ++icol)
            V(irow, icol) = a * V(irow, icol);

    return V;
}

alglib::real_2d_array operator-(const alglib::real_2d_array &a)
{
    return -1.0 * a;
}

alglib::real_2d_array operator+(const alglib::real_2d_array &a,
                                const alglib::real_2d_array &b)
{
    alglib::real_2d_array B = ones(a.cols(), b.rows());
    alglib::real_2d_array V(b);
    alglib::rmatrixgemm(a.rows(), b.cols(), a.cols(),
                        1.0,
                        a, 0, 0, 0,
                        B, 0, 0, 0,
                        1.0,
                        V, 0, 0);
    return V;
}

alglib::real_2d_array operator-(const alglib::real_2d_array &a,
                                const alglib::real_2d_array &b)
{
    alglib::real_2d_array B = ones(a.cols(), b.rows());
    alglib::real_2d_array V(b);
    alglib::rmatrixgemm(a.rows(), b.cols(), a.cols(),
                        1.0,
                        a, 0, 0, 0,
                        B, 0, 0, 0,
                        -1.0,
                        V, 0, 0);
    return V;
}

alglib::real_2d_array operator*(const alglib::real_2d_array &a,
                                const alglib::real_2d_array &b)
{
    alglib::real_2d_array V = ones(a.rows(), b.cols());
    alglib::rmatrixgemm(a.rows(), b.cols(), a.cols(),
                        1.0,
                        a, 0, 0, 0,
                        b, 0, 0, 0,
                        0.0,
                        V, 0, 0);
    return V;
}

alglib::real_2d_array inv(const alglib::real_2d_array &a)
{
    alglib::ae_int_t info;
    alglib::matinvreport rep;
    alglib::real_2d_array V(a);
    alglib::rmatrixinverse(V, info, rep);
    return V;
}

double max(const alglib::real_2d_array &V)
{
    int ncol = V.cols();
    int nrow = V.rows();
    double vmax = std::numeric_limits<double>::min();
    for (int irow = 0; irow < nrow; ++irow)
        for (int icol = 0; icol < ncol; ++icol)
            if (V(irow, icol) > vmax)
                vmax = V(irow, icol);

    return vmax;
}

double min(const alglib::real_2d_array &V)
{
    int ncol = V.cols();
    int nrow = V.rows();
    double vmin = std::numeric_limits<double>::max();
    for (int irow = 0; irow < nrow; ++irow)
        for (int icol = 0; icol < ncol; ++icol)
            if (V(irow, icol) < vmin)
                vmin = V(irow, icol);

    return vmin;
}

alglib::real_2d_array get_column(int icol, const alglib::real_2d_array &a, bool norm)
{
    int n = a.rows();
    alglib::real_2d_array O;

    O.setlength(n, 1);
    alglib::rmatrixcopy(a.rows(), 1, a, 0, icol, O, 0, 0);
    if (norm)
        O = O / vnorm(O);
    return O;
}

alglib::real_1d_array sample_mean(const alglib::real_2d_array &M)
{
    int nvar = M.cols();
    int nsamples = M.rows();
    alglib::real_1d_array out = zeros(nvar);

    for (int i = 0; i < nsamples; ++i)
    {
        for (int j = 0; j < nvar; ++j)
        {
            out(j) += M(i, j);
        }
    }
    for (int j = 0; j < nvar; ++j)
        out(j) = out(j) / ((double) nsamples);

    return out;
}
