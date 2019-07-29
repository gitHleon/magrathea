/*
 * test-matrix.cc
 *
 *  Created on: Jul 19, 2019
 *      Author: lacasta
 */
#include "MatrixTransform.h"
#include "PetalCoordinates.h"
#include <cmath>
#include <cstdlib>
#include <iostream>

double
randuniform()
{
    double x;
    x = ((double)rand()) / ((double)RAND_MAX);
    return x;
}

double
randuniform(double a, double b)
{
    double x = randuniform();
    return a + x * (b - a);
}

double degree = M_PI / 180.;
int
main(int argc, char **argv)
{
    Point P(1.0, 2.0), Q(-1.0, 2.0);
    Point R = (P * Q) * P + 5.0 * Q;

    std::cout << R << " norm " << R.unit() << std::endl;

    std::cout << "## Testing matrices" << std::endl;
    MatrixTransform m;
    std::cout << m << std::endl;

    std::cout << "Translate" << std::endl;
    m.translate(2.0, 1.0);
    std::cout << m << std::endl;

    std::cout << "Rotate 30 deg around (2,2)" << std::endl;
    MatrixTransform m1;
    std::cout << m1 << std::endl;
    m1.rotate_around_point(30.0 * degree, Point(2.0, 2.0));
    std::cout << "--\n" << m1 << std::endl;
    std::cout << "(0,2) -> " << m.transform(Point(0.0, 2.0)) << std::endl;
    std::cout << "(0,0) -> " << m.transform(Point(0.0, 0.0)) << std::endl;
    std::cout << "(2,2) -> " << m.transform(Point(2.0, 2.0)) << std::endl;

    std::cout << "Matrix inversion " << std::endl;
    MatrixTransform m2 = m1.inverse();
    MatrixTransform m3 = m1 * m2;
    std::cout << m2 << std::endl;
    std::cout << "---\n" << std::endl;
    std::cout << m3 << std::endl;

    //    Point upper_locator(131.104, 968.526);
    //    Point lower_locator(0, 382.000);
    //    MatrixTransform Rm;
    //    Rm.translate(Point(randuniform(-50, 50), randuniform(-150, 150)));
    //    Rm.rotate(randuniform(M_PI_2 - 20.0*degree, M_PI_2 + 20.0*degree));
    //
    //    Point Pl1 = Rm*upper_locator;
    //    Point Pl2 = Rm*lower_locator;
    //
    //    PetalCoordinates Petal(Pl1, Pl2);

    Point upper_locator(23.8152, 1059.4657);
    Point lower_locator(48.9825, 458.9928);
    PetalCoordinates Petal(upper_locator, lower_locator);
    std::cout << "------ Petal Coordinates ------- " << std::endl << Petal << std::endl;
    for (int i = 0; i < PetalCoordinates::LAST; ++i)
    {
        std::cout << "Sensor R" << i << " "
                  << Petal.get_sensor_pos_in_petal((PetalCoordinates::Sensor)i) << " - "
                  << Petal.get_sensor_pos_in_gantry((PetalCoordinates::Sensor)i)
                  << std::endl;
    }

    std::cout << std::endl << "origin in gantry " << lower_locator << Petal.petal_to_gantry(Point(0, 0)) << std::endl;

    std::cout << "F fiducials" << std::endl;
    for (int iring = 0; iring < 6; ++iring)
    {
        int nsensor = 1;
        if (iring > 2)
            nsensor = 2;

        for (int j = 0; j < nsensor; ++j)
        {
            for (int ik = 0; ik < 4; ++ik)
            {
                std::cout << "R" << iring << "_T4_" << ik << "_" << j << ": "
                          << "  \tP " << Petal.get_fiducial_in_petal("T4", iring, j, ik) << "  \tG "
                          << Petal.get_fiducial_in_gantry("T4", iring, j, ik) << "  \tX "
                          << Petal.gantry_to_petal(Petal.get_fiducial_in_gantry("T4", iring, j, ik)) << std::endl;
            }
            std::cout << std::endl;
        }
    }
    return 0;
}
