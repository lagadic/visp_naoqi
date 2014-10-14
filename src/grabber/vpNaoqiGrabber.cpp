/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://team.inria.fr/lagadic/visp for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://team.inria.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * This class allows to get images from the robot remotely.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/


// Aldebaran includes.
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>

// ViSP includes
#include <visp/vpImageConvert.h>

#include <visp_naoqi/vpNaoqiGrabber.h>

vpNaoqiGrabber::vpNaoqiGrabber()
  : m_camProxy(NULL), m_robotIp("198.18.0.1"),
    m_robotPort(9559), m_fps(30), m_isOpen(false), m_width(0), m_height(0)
{

}

vpNaoqiGrabber::~vpNaoqiGrabber()
{
  cleanup();
}

void vpNaoqiGrabber::open()
{
  if (! m_isOpen) {
    // Create a proxy to ALVideoDevice on the robot
    m_camProxy = new AL::ALVideoDeviceProxy(m_robotIp, m_robotPort);

    // Subscribe a client image requiring 320*240 and BGR colorspace
    std::string subscriberID = "subscriberID";
    m_clientName = m_camProxy->subscribe(subscriberID, AL::kQVGA, AL::kBGRColorSpace, m_fps);

    // Select the camera left(0) or right(1)
    m_camProxy->setCameraParameter(m_clientName, AL::kCameraSelectID, 0);

    // update image size
    /* Retrieve an image from the camera.
     * The image is returned in the form of a container object, with the
     * following fields:
     * 0 = width
     * 1 = height
     * 2 = number of layers
     * 3 = colors space index (see alvisiondefinitions.h)
     * 4 = time stamp (seconds)
     * 5 = time stamp (micro seconds)
     * 6 = image buffer (size of width * height * number of layers)
     */
    AL::ALValue img = m_camProxy->getImageRemote(m_clientName);
    m_width  = (int) img[0];
    m_height = (int) img[1];
    m_camProxy->releaseImage(m_clientName);

    m_isOpen = true;
  }
}

void vpNaoqiGrabber::cleanup()
{
  if (m_camProxy != NULL) {
    m_camProxy->unsubscribe(m_clientName);
    delete m_camProxy;
  }
  m_isOpen = false;
}

void vpNaoqiGrabber::acquire(vpImage<unsigned char> &I)
{
  struct timeval timestamp;
  acquire(I, timestamp);
}

void vpNaoqiGrabber::acquire(vpImage<unsigned char> &I, struct timeval &timestamp)
{
  if (! m_isOpen)
    open();

  /* Retrieve an image from the camera.
   * The image is returned in the form of a container object, with the
   * following fields:
   * 0 = width
   * 1 = height
   * 2 = number of layers
   * 3 = colors space index (see alvisiondefinitions.h)
   * 4 = time stamp (seconds)
   * 5 = time stamp (micro seconds)
   * 6 = image buffer (size of width * height * number of layers)
   */
  AL::ALValue img = m_camProxy->getImageRemote(m_clientName);

  m_width  = (int) img[0];
  m_height = (int) img[1];
  double tv_sec  = (double)img[4];
  double tv_usec = (double)img[5];
  timestamp.tv_sec  = (unsigned long) tv_sec;
  timestamp.tv_usec = (unsigned long) tv_usec;

  // Access the image buffer (6th field) and assign it to the ViSP image container
  unsigned char *img_buffer = (unsigned char *) img[6].GetBinary();

  // Tells to ALVideoDevice that it can give back the image buffer to the
  // driver. Optional after a getImageRemote but MANDATORY after a getImageLocal.
  m_camProxy->releaseImage(m_clientName);

  I.resize(m_height, m_width);
  vpImageConvert::BGRToGrey(img_buffer, (unsigned char *)I.bitmap, m_width, m_height);
}

void vpNaoqiGrabber::acquire(cv::Mat &I)
{
  struct timeval timestamp;
  acquire(I, timestamp);
}

/*!
  \warning Should be improved to detect the type cv::CV_8UC3, cv::CV_8UC1
 */
void vpNaoqiGrabber::acquire(cv::Mat &I, struct timeval &timestamp)
{
  if (! m_isOpen)
    open();

  /* Retrieve an image from the camera.
   * The image is returned in the form of a container object, with the
   * following fields:
   * 0 = width
   * 1 = height
   * 2 = number of layers
   * 3 = colors space index (see alvisiondefinitions.h)
   * 4 = time stamp (seconds)
   * 5 = time stamp (micro seconds)
   * 6 = image buffer (size of width * height * number of layers)
   */
  AL::ALValue img = m_camProxy->getImageRemote(m_clientName);

  m_width  = (int) img[0];
  m_height = (int) img[1];
  double tv_sec  = (double)img[4];
  double tv_usec = (double)img[5];
  timestamp.tv_sec  = (unsigned long) tv_sec;
  timestamp.tv_usec = (unsigned long) tv_usec;

  //  if (I.size() != cv::Size(m_width, m_height)) {
  //    if (I.type() == cv::CV_8UC3)
  //      I.resize( cv::Size(m_width, m_height), cv::Scalar(0,0,0) );
  //    else if(I.type() == cv::CV_8UC1)
  //      I.resize( cv::Size(m_width, m_height), cv::Scalar(0) );
  //  }

  // Access the image buffer (6th field) and assign it to the opencv image container
  I.data = (unsigned char*) img[6].GetBinary();

  // Tells to ALVideoDevice that it can give back the image buffer to the
  // driver. Optional after a getImageRemote but MANDATORY after a getImageLocal.
  //m_camProxy->releaseImage(m_clientName);
}

void vpNaoqiGrabber::release()
{
  // Tells to ALVideoDevice that it can give back the image buffer to the
  // driver. Optional after a getImageRemote but MANDATORY after a getImageLocal.
  m_camProxy->releaseImage(m_clientName);
}
