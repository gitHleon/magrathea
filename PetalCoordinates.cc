/*
 * PetalCoordinates.cpp
 *
 *  Created on: Jul 22, 2019
 *      Author: lacasta
 */

#include "PetalCoordinates.h"
static Point __nominal_upper_locator__(131.104, 968.526);
static Point __nominal__lower_locator__(0, 382.000);
static double __nominal_phi__    = (__nominal_upper_locator__ - __nominal__lower_locator__).phi();
static double __petal_rotation__ = -3.5e-3;

struct SensorData
{
        int iring;
        double Ri;
        double R;
        double Ro;
        double angle;
};
static SensorData __sensor_data__[] =
{
 {0, 384.050000, 438.614000, 488.873000, 0.000000},
 {1, 489.373000, 534.639000, 574.644000, 0.000000},
 {2, 575.144000, 609.405000, 637.659000, 0.000000},
 {3, 638.159000, 697.899000, 755.951000, 0.050397},
 {4, 756.451000, 812.471000, 866.512000, 0.050377},
 {5, 867.012000, 918.749000, 968.235000, 0.050381},
};


PetalCoordinates::PetalCoordinates(const Point &upper_locator, const Point &lower_locator)
{
    /*
     * upper_locator and lower_locator, together with the Y axis define the petal
     * coordinate system
     */
    /*
     * First set the origin in the lower locator
     */
    translate(-lower_locator);

    /*
     * Transform upper_locator and compute the actual phi angle
     */
    rotate(__nominal_phi__ - transform(upper_locator).phi());
    _inv = inverse();

    for (int iring=0, cntr=0; iring<6; ++iring)
    {
        int nsensor = 1;
        if (iring>2)
            nsensor = 2;

        for (int isensor=0; isensor < nsensor; ++isensor)
        {
            double angle = __sensor_data__[iring].angle;
            if (isensor)
                angle = -angle;

            angle += __petal_rotation__;

            Point Psensor(0,0);
            MatrixTransform M;
            M.translate(0.0, __sensor_data__[iring].R);
            M.rotate(angle);
            M.translate(-__nominal__lower_locator__);
            sensor_transform.push_back(M);

            Point pos = M.transform(Point(0,0));
            sensor_positions.push_back(pos);
            sensor_positions_gantry.push_back(_inv.transform(pos));
            cntr++;
        }
    }
}

void PetalCoordinates::cpy(const PetalCoordinates &PC)
{
    // copy the Matrix transform
    *dynamic_cast<MatrixTransform *>(this) = *dynamic_cast<const MatrixTransform *>(&PC);

    // now the local members
}

PetalCoordinates::~PetalCoordinates()
{
    // TODO Auto-generated destructor stub
}

PetalCoordinates::PetalCoordinates(const PetalCoordinates &other)
{
    cpy(other);
    // TODO Auto-generated constructor stub

}

PetalCoordinates& PetalCoordinates::operator=(const PetalCoordinates &other)
{
    // TODO Auto-generated method stub
    if (this != &other)
        cpy(other);

    return *this;
}

Point PetalCoordinates::get_sensor_pos_in_petal(Sensor ss)
{
    return sensor_positions[ss];
}
Point PetalCoordinates::get_sensor_pos_in_gantry(Sensor ss)
{
    return sensor_positions_gantry[ss];
}

Point PetalCoordinates::gantry_to_petal(const Point &P)
{
    return transform(P);
}
Point PetalCoordinates::petal_to_gantry(const Point &P)
{
    return _inv.transform(P);
}
