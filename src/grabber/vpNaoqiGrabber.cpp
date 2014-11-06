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
 * Giovanni Claudio
 *
 *****************************************************************************/


// Aldebaran includes.
#include <alvision/alimage.h>

// ViSP includes
#include <visp/vpImageConvert.h>
#include <visp/vpXmlParserCamera.h>

#include <visp_naoqi/vpNaoqiConfig.h>
#include <visp_naoqi/vpNaoqiGrabber.h>

/*!
  Default constructor that set the default parameters as:
  - robot ip address: "198.18.0.1"
  - robot port: 9559
  - camera framerate: 30 fps
  */
vpNaoqiGrabber::vpNaoqiGrabber()
  : m_videoProxy(NULL), m_robotIp("198.18.0.1"),
    m_robotPort(9559), m_fps(30), m_isOpen(false), m_width(0), m_height(0)
{

}

/*!
  Destructor that call cleanup().
 */
vpNaoqiGrabber::~vpNaoqiGrabber()
{
  cleanup();
}

/*!
  Select the camera to use.
  \param camera_id : Camera identifier; camera left(0) or right(1).
 */
void vpNaoqiGrabber::setCamera(const int &camera_id)
{
  m_cameraId = camera_id;
  // TODO: add specialisation Romeo/Nao
  if (m_cameraId == 0)
    m_cameraName = "CameraLeft";
  else if (m_cameraId == 1)
    m_cameraName = "CameraRight";
}

void vpNaoqiGrabber::open()
{
  if (! m_isOpen) {
    // Create a proxy to ALVideoDevice on the robot
    m_videoProxy = new AL::ALVideoDeviceProxy(m_robotIp, m_robotPort);
    // Subscribe a client image requiring 320*240 and BGR colorspace
    m_clientName = "subscriberID";
    m_videoProxy->unsubscribeAllInstances(m_clientName);
    m_clientName = m_videoProxy->subscribe(m_clientName, AL::kQVGA, AL::kBGRColorSpace, m_fps);

    //std::cout << m_clientName << std::endl;

    //AL::ALValue index_cameras = m_videoProxy->getCameraIndexes();
    //std::cout << "Avaible index cameras: "<< index_cameras << std::endl;

    // Select the camera left(0) or right(1)


    m_videoProxy->setCameraParameter(m_clientName, AL::kCameraSelectID, m_cameraId);

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
    m_img = m_videoProxy->getImageRemote(m_clientName);
    m_width  = (int) m_img[0];
    m_height = (int) m_img[1];
    m_videoProxy->releaseImage(m_clientName);

    m_isOpen = true;
  }
}

void vpNaoqiGrabber::cleanup()
{
  if (m_videoProxy != NULL) {
    m_videoProxy->unsubscribe(m_clientName);
    delete m_videoProxy;
    m_videoProxy = NULL;
  }
  m_isOpen = false;
}

/*!

  The image is copied.

 */
void vpNaoqiGrabber::acquire(vpImage<unsigned char> &I)
{
  struct timeval timestamp;
  acquire(I, timestamp);
}

/*!

  The image is copied.

 */
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
  m_img = m_videoProxy->getImageRemote(m_clientName);

  m_width  = (int) m_img[0];
  m_height = (int) m_img[1];
  double tv_sec  = (double)m_img[4];
  double tv_usec = (double)m_img[5];
  timestamp.tv_sec  = (unsigned long) tv_sec;
  timestamp.tv_usec = (unsigned long) tv_usec;

  // Access the image buffer (6th field) and assign it to the ViSP image container
  unsigned char *img_buffer = (unsigned char *) m_img[6].GetBinary();

  // Tells to ALVideoDevice that it can give back the image buffer to the
  // driver. Optional after a getImageRemote but MANDATORY after a getImageLocal.
  m_videoProxy->releaseImage(m_clientName);

  I.resize(m_height, m_width);
  vpImageConvert::BGRToGrey(img_buffer, (unsigned char *)I.bitmap, m_width, m_height);
}

/*!

  The image is not copied. Here we just update to pointer to the NaoQi image.

  \warning Should be improved to detect the type cv::CV_8UC3, cv::CV_8UC1
 */

void vpNaoqiGrabber::acquire(cv::Mat &I)
{
  struct timeval timestamp;
  acquire(I, timestamp);
}

/*!

  The image is not copied. Here we just update to pointer to the NaoQi image.

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
  m_img = m_videoProxy->getImageRemote(m_clientName);

  m_width  = (int) m_img[0];
  m_height = (int) m_img[1];
  double tv_sec  = (double)m_img[4];
  double tv_usec = (double)m_img[5];
  timestamp.tv_sec  = (unsigned long) tv_sec;
  timestamp.tv_usec = (unsigned long) tv_usec;

  //  if (I.size() != cv::Size(m_width, m_height)) {
  //    if (I.type() == cv::CV_8UC3)
  //      I.resize( cv::Size(m_width, m_height), cv::Scalar(0,0,0) );
  //    else if(I.type() == cv::CV_8UC1)
  //      I.resize( cv::Size(m_width, m_height), cv::Scalar(0) );
  //  }

  // Access the image buffer (6th field) and assign it to the opencv image container
  I.data = (unsigned char*) m_img[6].GetBinary();

  //cv::imshow("images", I);

  // Tells to ALVideoDevice that it can give back the image buffer to the
  // driver. Optional after a getImageRemote but MANDATORY after a getImageLocal.
  m_videoProxy->releaseImage(m_clientName);
}

/*!
  Return the camera parameters corresponding to the camera that is selected using setCamera().
  \warning The grabber should be open prior calling this function.

  \param projModel : Model that is used.
  \return The camera parameters

  \code
#include <visp_naoqi/vpNaoqiGrabber.h>

int main()
{
  vpNaoqiGrabber g;
  g.setRobotIp("131.254.13.37");
  g.setFramerate(15);
  g.setCamera(0);
  g.open();
  vpCameraParameters cam = g.getCameraParameters();
}

 */
vpCameraParameters
vpNaoqiGrabber::getCameraParameters(vpCameraParameters::vpCameraParametersProjType projModel) const
{
  vpCameraParameters cam;
  char filename[FILENAME_MAX];
  vpXmlParserCamera p; // Create a XML parser

  // Parse the xml file to find the intrinsic camera depending on the camera name and image resolution
  sprintf(filename, "%s", VISP_NAOQI_INTRINSIC_CAMERA_FILE);
  if (p.parse(cam, filename, m_cameraName, projModel, m_width, m_height) != vpXmlParserCamera::SEQUENCE_OK) {
    std::cout << "Cannot found camera parameters in file: " << filename << std::endl;
  }

  return cam;
}
