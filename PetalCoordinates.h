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
#include <map>
#include <string>
#include <exception>

class PetalCoordException : public std::exception
{
    protected:
        /// stores the exception message
        std::string _why;

    public:
        /// Constructor
        PetalCoordException() {}
        PetalCoordException(const std::string &msg): _why(msg) {}
        PetalCoordException(const PetalCoordException &e): _why(e._why) {}
        virtual ~PetalCoordException() throw() {}

        // returns the message
        const char* what() const throw() { return _why.c_str(); }
};

/**
 * This class allows to transform from gantry to petal coordinates
 * and back. It will also provide the coordinates of the sensor
 * centers in the petal as well as the sensor fiducial coordinates.
 *
 * The petal reference system will be defined by the two fiducials in
 * the petal locators and the Y axis. We assume that the Y axis goes
 * along the radius of the petal. The X axis goes across the petal
 * sensors.
 *
 * The origin of the petal reference will be the fiducial in the lower
 * petal locator.
 *
 */
class PetalCoordinates: public MatrixTransform
{
    public:
        /*
         * Sensors ate numbered from right to left in rings 3-5, as
         * specified in the module specifications.
         */
        enum Sensor {R0, R1, R2, R3R, R3L, R4R, R4L, R5R, R5L, LAST};

    private:
        void cpy(const PetalCoordinates &PC);

        // The list if sensor transformations. Indices are like in
        // Sensor enum.
        std::vector<MatrixTransform> sensor_transform;

        // Sensor positions (in the petal coord system)
        std::vector<Point> sensor_positions;

        // Sensor position (in the gantry coord system)
        std::vector<Point> sensor_positions_gantry;

        // Fiducial coordinates in the Petal reference
        std::map<std::string, Point> fiducials;

        // Fiducial coordinates in the Gantry reference
        std::map<std::string, Point> fiducials_gantry;

        MatrixTransform _inv;

    public:
        /**
         * Constructor. Receives as input the two fiducials of the
         * petal locators. This will define the petal coordinate
         * system. In this system we assume that the petal in
         * "vertical".
         *
         * @param upper_locator Position in Gantry of the upper
         *                      locator (wide part of petal)
         *
         * @param lower_locator Position in gantry of the lower
         *                      locator. This will be, in fact, the
         *                      origin of the petal reference system.
         */
        PetalCoordinates(const Point &upper_locator, const Point &lower_locator);
        virtual ~PetalCoordinates();
        PetalCoordinates(const PetalCoordinates &other);
        PetalCoordinates& operator=(const PetalCoordinates &other);

        /**
         * These methods return the coordinates of the sensor center
         * in the corresponding system of reference.
         */
        Point get_sensor_pos_in_petal(Sensor indx) const;
        Point get_sensor_pos_in_gantry(Sensor indx) const;

        /**
         * Methods to transform from one system of reference to the
         * other.
         */
        Point gantry_to_petal(const Point &P) const;
        Point petal_to_gantry(const Point &P) const;


        /**
         * Methods to get fiducial coordinates
         *
         * @param  type   T1, T2, T3, T4 for type 1, 2, 3, or 4.
         *                H1 ... H5 for HPK fiducials
         * @param  iring  sensor number (R0:0, R1:1, etc)
         * @param  side   (0: right, 1:left)
         * @param  cntr   The index of the possible various fiducials of
         *                this kind
         *
         * @return        A Point with the position of the fiducial.
         */
        Point get_fiducial_in_petal(const std::string &type, int iring, int side, int cntr) const
            throw(PetalCoordException);

        Point get_fiducial_in_gantry(const std::string &type, int iring, int side, int cntr) const
            throw(PetalCoordException);


};

#endif /* PETALCOORDINATES_H_ */
