/*
 * flatness.h
 */

#ifndef FLATNESS_H_
#define FLATNESS_H_

#include <alglibtools.h>

/*
 * Computes the flatness of a set of points as described in
 * Verification of form tolerances Part I: Basic issues, flatness, and straightness
 * (Kirsten Carr* and Placid Ferreira)
 *
 * The input date is a Nx3 2D array where columns are X, Y and Z coordinates
 * and each row represents a 3D point in space.
 *
 * It returns the flatness
 */
double flatness_lin(const alglib::real_2d_array &M)
    throw (AlglibException);



#endif /* FLATNESS_H_ */
