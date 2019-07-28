/*
 * QPetalLocator.h
 *
 */

#ifndef QPETALLOCATOR_H_
#define QPETALLOCATOR_H_

#include <QDialog>
#include <MotionHandler.h>
#include <Point.h>

struct QPetalLocatorPriv;

class QPetalLocator : public QDialog
{
        Q_OBJECT

    private:
        MotionHandler *motion;
        QPetalLocatorPriv *priv;

        Point top;
        Point bottom;
    public:
        QPetalLocator(MotionHandler *M);
        virtual ~QPetalLocator();

    public slots:
        void find_top_locator();
        void find_bottom_locator();

        Point get_top_position() const { return top; }
        Point get_bottom_position() const { return bottom; }


};

#endif /* QPETALLOCATOR_H_ */
