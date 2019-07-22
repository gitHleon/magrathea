/*
 * PetalCoordinates.h
 *
 *  Created on: Jul 22, 2019
 *      Author: lacasta
 */

#ifndef PETALCOORDINATES_H_
#define PETALCOORDINATES_H_

#include <MatrixTransform.h>
#include <vector>

class PetalCoordinates: public MatrixTransform
{
    public:
        enum Sensor {R0, R1, R2, R3L, R3R, R4L, R4R, R5L, R5R, LAST};

    private:
        void cpy(const PetalCoordinates &PC);

        std::vector<MatrixTransform> sensor_transform;
        std::vector<Point> sensor_positions;
        std::vector<Point> sensor_positions_gantry;
        MatrixTransform _inv;

    public:
        PetalCoordinates(const Point &upper_locator, const Point &lower_locator);
        virtual ~PetalCoordinates();
        PetalCoordinates(const PetalCoordinates &other);
        PetalCoordinates& operator=(const PetalCoordinates &other);

        Point get_sensor_pos_in_petal(Sensor);
        Point get_sensor_pos_in_gantry(Sensor);

        Point gantry_to_petal(const Point &P);
        Point petal_to_gantry(const Point &P);

};

#endif /* PETALCOORDINATES_H_ */
