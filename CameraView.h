/*
 * CameraView.h
 *
 *  Created on: 2 ago. 2019
 *      Author: GantryUser
 */

#ifndef CAMERAVIEW_H_
#define CAMERAVIEW_H_

#include <QCameraViewfinder>

class CameraView : public QCameraViewfinder
{
        Q_OBJECT
    public:
        explicit CameraView(QWidget *parent = Q_NULLPTR): QCameraViewfinder(parent) {}
        virtual ~CameraView();

        void mouseMoveEvent(QMouseEvent *event);

    signals:
        void mouseMoved(QMouseEvent *);
};

#endif /* CAMERAVIEW_H_ */
