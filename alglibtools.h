/*
 * alglibtools.h
 *
 * This is a set of tools to make live easier when handling alglib real 1D and 2D arrays
 *
 * 2D matrices are stored in raw-major order.
 * M.setlenght(nrow, ncol)
 * M[irow][icol]
 * M(irow, icol)
 */
#ifndef ALGLIBTOOLS_H_
#define ALGLIBTOOLS_H_

#include <iosfwd>
#include <exception>
#include <ap.h>

/*
 * This exception is thrown when the tools find a problem with alglib
 */
class AlglibException : public std::exception
{
    protected:
        /// stores the exception message
        std::string _why;

    public:
        /// Constructor
        AlglibException() {}
        AlglibException(const std::string &msg) : _why(msg) {}

        /// Copy constructor
        AlglibException(const AlglibException &e)
        : _why(e._why) {}

        /// Destructor
        virtual ~AlglibException() throw() {}

        /** Returns a C-style character string describing the general
                cause of the current error.
         */
        const char* what() const throw() { return _why.c_str(); }
};



/*
 * The ostream<< operator for 2D arrays
 */
std::ostream &operator<<(std::ostream &os, const alglib::real_2d_array &M);

/*================================================================
 * methods to operate with 1D arrays
 *===============================================================*/
/*
 * Creates a 1D array with all elements set to val
 */
alglib::real_1d_array real_1d_array_with_value(int n, double val);

/*
 * return a 1D array with all elements set to 1
 */
inline
alglib::real_1d_array ones(int n)
{
    return real_1d_array_with_value(n, 1.0);
}

/*
 * return a 1D array with all elements set to 0
 */
inline
alglib::real_1d_array zeros(int n)
{
    return real_1d_array_with_value(n, 0.0);
}

/*
 * Dot product of two 1D arrays.
 * It takes the smallerdim of the 2 vectors
 */
double vdot(const alglib::real_1d_array &a, const alglib::real_1d_array &b);

/*
 * Calculates the norm of the array
 */
double vnorm(const alglib::real_1d_array &a);

/*
 * Calculates the cross product of 2 arrays. Only dim-3 is allowed
 */
alglib::real_1d_array  cross(const alglib::real_1d_array &a, const alglib::real_1d_array &b);

/*
 * Operator to multiply an array by a number
 */
alglib::real_1d_array operator* (double a, const alglib::real_1d_array &W);
alglib::real_1d_array operator* (const alglib::real_1d_array &W, double a);


/*================================================================
 * methods to operate with 1D arrays
 *===============================================================*/
alglib::real_2d_array ones(int nrow, int ncol);
alglib::real_2d_array zeros(int nrow, int ncol);

double vdot(const alglib::real_2d_array &a, const alglib::real_2d_array &b)
    throw(AlglibException);

double vnorm(const alglib::real_2d_array &a);
alglib::real_2d_array  cross(const alglib::real_2d_array &a, const alglib::real_2d_array &b)
    throw(AlglibException);

alglib::real_2d_array operator/ (const alglib::real_2d_array &W, double a);
alglib::real_2d_array operator* (const alglib::real_2d_array &W, double a);
alglib::real_2d_array operator* (double a, const alglib::real_2d_array &W);
alglib::real_2d_array operator- (const alglib::real_2d_array &a);
alglib::real_2d_array operator+ (const alglib::real_2d_array &a, const alglib::real_2d_array &b);
alglib::real_2d_array operator- (const alglib::real_2d_array &a, const alglib::real_2d_array &b);
alglib::real_2d_array operator* (const alglib::real_2d_array &a, const alglib::real_2d_array &b);
alglib::real_2d_array inv(const alglib::real_2d_array &a);
double max(const alglib::real_2d_array &V);
double min(const alglib::real_2d_array &V);
alglib::real_2d_array get_column(int icol, const alglib::real_2d_array &a, bool unit=false);
alglib::real_1d_array sample_mean(const alglib::real_2d_array &M);





#endif /* ALGLIBTOOLS_H_ */
