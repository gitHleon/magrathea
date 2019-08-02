/*
 * CameraView.cpp
 *
 *  Created on: 2 ago. 2019
 *      Author: GantryUser
 */

#include "CameraView.h"
#include <iostream>

CameraView::~CameraView()
{
    // TODO Auto-generated destructor stub

}

void CameraView::mouseMoveEvent(QMouseEvent *event)
{
    emit mouseMoved(event);
}
