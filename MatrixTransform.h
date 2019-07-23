/*
 * MatrixTransform.h
 *
 *  Created on: Jul 19, 2019
 *      Author: lacasta
 */

#ifndef MATRIXTRANSFORM_H_
#define MATRIXTRANSFORM_H_

#include <iosfwd>
#include <Point.h>

/*
  A matrix to perform 2D transformations
  All transformations can be represented as 3x3 transformation
  matrices of the following form:
      | a c e |
      | b d f |
      | 0 0 1 |

  Since only six values are used in the above 3x3 matrix,
  a transformation matrix is also expressed as a vector:

  [a b c d e f].

  Transformations map coordinates and lengths from a new coordinate system
  into a previous coordinate system:

    | x_prev |    | a c e | | x_new |
    | y_prev | =  | b d f | | y_new |
    | 1      |    | 0 0 1 | | 1     |


  3-by-3 transformation matrix.

*/
class MatrixTransform
{
    private:
        // the matrix
        double _M[6];
    public:
        MatrixTransform();
        MatrixTransform(double*ptr);
        MatrixTransform(const MatrixTransform &T);

        virtual ~MatrixTransform();
        static MatrixTransform matrix_multiply(const MatrixTransform &a, const MatrixTransform &b);
        static MatrixTransform identity();

        MatrixTransform rotation() const;
        MatrixTransform operator*(const MatrixTransform &M);
        Point operator*(const Point &P) const;
        Point transform(const Point &P) const { return (*this)*P; }
        double operator[](int i) const { return _M[i]; }

        /*
            A translation is equivalent to
            | 1 0 tx |
            | 0 1 ty | or [ 1 0 0 1 tx ty ]
            | 0 0 1  |
         */
        MatrixTransform &translate(double x, double y=0);
        MatrixTransform &translate(const Point &P);

        /*
            Scaling is eqivalent to
            | sx  0  0 |
            | 0   sy 0 | or [ sx 0 0 sy 0 0 ]
            | 0   0  1 |
         */
        MatrixTransform &scale(double x, double y=1.0);

        /*
           A skew transformation around the X axis
           | 1  tan(angle)  0 |
           | 0  1           0 |
           | 0  0           1 |
         */
        MatrixTransform &skew_x(double angle);
        /*
            A skew transformation around the Y axis
            | 1           0  0 |
            | tan(angle)  1  0 |
            | 0           0  1 |
         */
        MatrixTransform &skew_y(double angle);

        /*
           A rotation around the origin is equivalent to
            | cos(a) -sin(a) 0 |
            | sin(a)  cos(a) 0 |
            | 0       0      1 |
         */
        MatrixTransform &rotate(double angle);

        /*
           A rotation around a point is equivalent to
           translate(cx, cy) rotate(angle) translate(-cx, -cy)
         */
        MatrixTransform &rotate_around_point(double angle, const Point &P);

        /*
         * Invert the matrix
         */
        MatrixTransform inverse();
};
MatrixTransform operator*(const MatrixTransform &a, const MatrixTransform &b);
std::ostream& operator<<(std::ostream &os, const MatrixTransform &M);
#endif /* MATRIXTRANSFORM_H_ */
