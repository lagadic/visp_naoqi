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

// ViSP includes
#include <visp/vpImageConvert.h>
#include <visp/vpXmlParserCamera.h>

#include <visp_naoqi/vpNaoqiConfig.h>
#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>


#include "al/alvisiondefinitions.h"
#include "al/naoqi_image.hpp"
#include "al/from_any_value.hpp"

/*!
  Default constructor that set the default parameters as:
  - camera framerate: 30 fps
  - m_cameraId: 0
  */
vpNaoqiGrabber::vpNaoqiGrabber(const qi::SessionPtr &session)
  : m_pVideo(session->service("ALVideoDevice")), m_pMemory(session->service("ALMemory")), m_fps(30), m_isOpen(false), m_width(0), m_height(0),
    m_img(), m_cameraName("CameraLeftEye"), m_cameraId (0), m_cameraMulti(false), m_resolution(AL::kQVGA), m_robotType(""), m_romeo(0), m_pepper(0),
    m_nao(0)
{

  // Get the robot type
  m_robotType = m_pMemory.call<std::string>("getData", "RobotConfig/Body/Type");
  std::transform(m_robotType.begin(), m_robotType.end(), m_robotType.begin(), ::tolower);

  if (m_robotType == "nao")
    m_nao =  1;
  if (m_robotType == "pepper" || m_robotType == "juliette")
  {
    m_robotType = "pepper";
    m_pepper = 1;
  }
  if (m_robotType == "romeo")
    m_romeo = 1;

  std::cout << "Connected to a " << m_robotType << " robot." << std::endl;
}

/*!
  Destructor that call cleanup().
 */
vpNaoqiGrabber::~vpNaoqiGrabber()
{
  if (m_isOpen)
    cleanup();
}

/*!
  Select the camera to use.
  \param camera_id : Camera identifier for Romeo: CameraLeft(0),CameraRight(1),CameraLeftEye(2),CameraRightEye(3). Pepper and Nao: CameraTop(0), CameraBottom(1)
 */
void vpNaoqiGrabber::setCamera(const int &camera_id)
{
  if (m_romeo)
  {
    if (camera_id == 0)
    {
      m_cameraName = "CameraLeft";
      m_cameraId = camera_id;
    }
    else if (camera_id == 1)
    {
      m_cameraName = "CameraRight";
      m_cameraId = camera_id;
    }
    else if (camera_id == 2)
    {
      m_cameraName = "CameraLeftEye";
      m_cameraId = 0;
    }
    else if (camera_id == 3)
    {
      m_cameraName = "CameraRightEye";
      m_cameraId = 1;
    }

  }
  if (m_pepper || m_nao)
  {
    if (camera_id == 0)
    {
      m_cameraName = "CameraTop";
      m_cameraId = camera_id;
    }
    else if (camera_id == 1)
    {
      m_cameraName = "CameraBottom";
      m_cameraId = camera_id;
    }
  }

  return;

}

/*!
  Open the connection with the robot.
  All the parameters should be set before calling this function.
  \code
  #include <visp_naoqi/vpNaoqiGrabber.h>

  int main()
  {
    // Create a session to connect with the Robot
    qi::SessionPtr m_session = qi::makeSession();
    std::string ip_port = "tcp://" + opt_ip + ":9559";
    m_session->connect(ip_port);
    if (! opt_ip.empty()) {
      std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
    }

    // Open Grabber
    vpNaoqiGrabber g(m_session);
    g.setCamera(opt_cam); // Select camera
    g.setCameraResolution(AL::kQVGA);
    g.open();
  }
  \endcode
 */
void vpNaoqiGrabber::open()
{
  if (! m_isOpen) {

    if (!m_handle.empty())
    {
      m_pVideo.call<qi::AnyValue>("unsubscribe", m_handle);
      m_handle.clear();
    }

    m_handle = m_pVideo.call<std::string>(
          "subscribeCamera",
          m_cameraName,
          m_cameraId,
          m_resolution,
          AL::kYUV422ColorSpace,
          m_fps
          );

    qi::AnyValue image_anyvalue = m_pVideo.call<qi::AnyValue>("getImageRemote", m_handle);
    naoqi::tools::NaoqiImage image;
    try{
      image = naoqi::tools::fromAnyValueToNaoqiImage(image_anyvalue);
    }
    catch(std::runtime_error& e)
    {
      std::cout << "Cannot retrieve image" << std::endl;
      return;
    }

    // Convert the image in RGB
    cv::Mat cv_YUV(image.height, image.width, CV_8UC2, image.buffer);
    cv::cvtColor(cv_YUV, m_img, CV_YUV2RGB_YUYV);

    m_width  = image.width;
    m_height = image.height;

    m_isOpen = true;
  }
}


void vpNaoqiGrabber::cleanup()
{
  if (!m_handle.empty())
  {
    std::cout << "Unsubscribe camera handle " << m_handle << std::endl;
    m_pVideo.call<qi::AnyValue>("unsubscribe", m_handle);
    m_handle.clear();
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

  // Get the image
  qi::AnyValue image_anyvalue = m_pVideo.call<qi::AnyValue>("getImageRemote", m_handle);
  naoqi::tools::NaoqiImage image;
  try{
    image = naoqi::tools::fromAnyValueToNaoqiImage(image_anyvalue);
  }
  catch(std::runtime_error& e)
  {
    std::cout << "Cannot retrieve image" << std::endl;
    return;
  }

  //Convert in ViSP format
  m_width  = image.width;
  m_height = image.height;
  I.resize(m_height, m_width);
  vpImageConvert::YUYVToGrey((unsigned char *)image.buffer, (unsigned char *)I.bitmap, m_width * m_height);

  double tv_sec  = (double)image.timestamp_s;
  double tv_usec = (double)image.timestamp_us;
  timestamp.tv_sec  = (unsigned long) tv_sec;
  timestamp.tv_usec = (unsigned long) tv_usec;

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

 */
void vpNaoqiGrabber::acquire(cv::Mat &I, struct timeval &timestamp)
{
  if (! m_isOpen)
    open();

  // Get the image
  qi::AnyValue image_anyvalue = m_pVideo.call<qi::AnyValue>("getImageRemote", m_handle);
  naoqi::tools::NaoqiImage image;
  try{
    image = naoqi::tools::fromAnyValueToNaoqiImage(image_anyvalue);
  }
  catch(std::runtime_error& e)
  {
    std::cout << "Cannot retrieve image" << std::endl;
    return;
  }

  cv::Mat cv_YUV(image.height, image.width, CV_8UC2, image.buffer);
  cv::cvtColor(cv_YUV, I, CV_YUV2RGB_YUYV);

  m_width  = image.width;
  m_height = image.height;

  double tv_sec  = (double)image.timestamp_s;
  double tv_usec = (double)image.timestamp_us;
  timestamp.tv_sec  = (unsigned long) tv_sec;
  timestamp.tv_usec = (unsigned long) tv_usec;

}

/*!
  Return the camera parameters corresponding to the camera that is selected using setCamera().
  \warning The grabber should be open prior calling this function.

  \param projModel : Model that is used.
  \return The camera parameters

 */
vpCameraParameters
vpNaoqiGrabber::getCameraParameters(vpCameraParameters::vpCameraParametersProjType projModel) const
{

  std::cout <<"This function is deprecated. Please use getIntrinsicCameraParameters instead" << std::endl;
    vpCameraParameters cam;
    char filename[FILENAME_MAX];
    vpXmlParserCamera p; // Create a XML parser

    // Parse the xml file to find the intrinsic camera depending on the camera name and image resolution
    sprintf(filename, "%s", VISP_NAOQI_INTRINSIC_CAMERA_FILE);
    if (p.parse(cam, filename, m_cameraName, projModel, m_width, m_height) != vpXmlParserCamera::SEQUENCE_OK) {
      std::cout << "Cannot found camera parameters in file: " << filename << std::endl;
      exit (EXIT_FAILURE);
    }

    return cam;
}


/*!
  Return the camera parameters corresponding to the camera with the desired resolution.
  \param projModel : Model that is used.
  \return The camera parameters

 */
vpCameraParameters vpNaoqiGrabber::getIntrinsicCameraParameters(const int & resolution, const std::string &cameraName, vpCameraParameters::vpCameraParametersProjType projModel)
{
  vpCameraParameters cam;
  char filename[FILENAME_MAX];
  vpXmlParserCamera p; // Create a XML parser

  int width,height;

  switch(resolution)
  {
  // Image of 160*120px
  case (AL::kQQVGA):
    width = 160;
    height = 120;
    break;

    // Image of 320*240px
  case (AL::kQVGA):
    width = 320;
    height = 240;
    break;

    // Image of 640*480px
  case (AL::kVGA):
    width = 640;
    height = 480;
    break;

    // Image of 1280*960px
  case (AL::k4VGA):
    width = 1280;
    height = 960;
    break;

  default:
    std::cout << "ERROR: Resolution not supported. Check the parameters resolution." <<std::endl;
    exit(0);
  }

  std::cout << "Look for camera with "<< width << " x " << height << std::endl;

  // Parse the xml file to find the intrinsic camera depending on the camera name and image resolution
  sprintf(filename, "%s", VISP_NAOQI_INTRINSIC_CAMERA_FILE);
  if (p.parse(cam, filename, cameraName, projModel, width, height) != vpXmlParserCamera::SEQUENCE_OK) {
    std::cout << "Cannot found camera parameters in file: " << filename << std::endl;
    exit(0);
  }

  return cam;
}


/*!
  Return the camera parameters corresponding to the camera with the desired resolution.
  \param projModel : Model that is used.
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
vpNaoqiGrabber::getCameraParameters( const int & resolution, const std::string &cameraName, vpCameraParameters::vpCameraParametersProjType projModel) const
{
  return getIntrinsicCameraParameters(resolution,cameraName,projModel);
}


/*!
  Return the extrinsic camera parameters corresponding to the camera specified.

  \param cameraName : Name of the Camera
  \param projModel : Model that is used

  \return The extrinsic camera parameters (Homogeneous matrix)

 */

vpHomogeneousMatrix vpNaoqiGrabber::getExtrinsicCameraParameters(const std::string &cameraName, vpCameraParameters::vpCameraParametersProjType projModel)
{
  vpHomogeneousMatrix eMc;
  std::string name;
  vpXmlParserHomogeneousMatrix p; // Create a XML parser

  if (cameraName == "CameraLeftEye" || cameraName == "CameraRightEye")
  {
    for(unsigned int i=0; i<3; i++)
      eMc[i][i] = 0; // remove identity
    //Set Rotation
    eMc[0][2] = 1.;
    eMc[1][0] = -1.;
    eMc[2][1] = -1.;
    //Set Translation:
    eMc[0][3] = 0.01299;
    eMc[1][3] = 0.;
    eMc[2][3] = 0.;
  }
  else if ( cameraName == "CameraTopPepper" )
  {
    for(unsigned int i=0; i<3; i++)
      eMc[i][i] = 0; // remove identity
    //Set Rotation
    eMc[0][2] = 1.;
    eMc[1][0] = -1.;
    eMc[2][1] = -1.;
    //Set Translation:
    eMc[0][3] = 0.08601;
    eMc[1][3] = 0.;
    eMc[2][3] = 0.16284;
  }
  else if (cameraName == "CameraBottomPepper")
  {
    vpRzyxVector r(0., 0.698132, 0.); // Roll pitch and yaw
    vpRotationMatrix R(r);
    vpTranslationVector t(0.09262, 0., 0.06177 );
    vpHomogeneousMatrix eMca_(t, R);
    vpHomogeneousMatrix aMv; // From aldebaran to visp frame convention

    for(unsigned int i=0; i<3; i++)
      aMv[i][i] = 0; // remove identity
    //Set Rotation
    aMv[0][2] = 1.;
    aMv[1][0] = -1.;
    aMv[2][1] = -1.;
    //Set Translation:
    aMv[0][3] = 0. ;
    aMv[1][3] = 0.;
    aMv[2][3] = 0.;

    eMc = eMca_ * aMv;
  }
  else
  {
    if (projModel == vpCameraParameters::perspectiveProjWithDistortion)
      name =  "eMc_" + cameraName + "_with_distorsion";
    else
      name =  "eMc_" + cameraName + "_without_distorsion";

    char filename[FILENAME_MAX];
    sprintf(filename, "%s", VISP_NAOQI_EXTRINSIC_CAMERA_FILE);

    if (p.parse(eMc,filename, name) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
      std::cout << "Cannot found the Homogeneous matrix named " << name << " in the file " <<  filename << std::endl;
      // exit(0);
    }
    else
      std::cout << "Read correctly the Homogeneous matrix named " << name << std::endl;
  }

  return eMc;

}


/*!
  Return the extrinsic camera parameters corresponding to the camera that is selected using setCamera().
  \warning The grabber should be open prior calling this function.

  \param cameraName : Name of the Camera (Null string = camera corresponding to the camera that is selected using setCamera() )
  \param projModel : Model that is used.(default = Projection with distorsion )

  \return The extrinsic camera parameters (Homogeneous matrix)

 */
vpHomogeneousMatrix
vpNaoqiGrabber::get_eMc(vpCameraParameters::vpCameraParametersProjType projModel, std::string cameraName ) const
{

  vpHomogeneousMatrix eMc;
  // std::string name;
  vpXmlParserHomogeneousMatrix p; // Create a XML parser

  if (cameraName == "")
    cameraName = m_cameraName;

  eMc = getExtrinsicCameraParameters(cameraName, projModel);

  return eMc;
}

/*!
  Set a camera parameter.
  \param parameterId : Camera parameter requested.
  \param value : Value requested.

  \return True if succesfull.
*/

bool vpNaoqiGrabber::setCameraParameter(const int& parameterId, const int& value)
{

  bool result = m_pVideo.call<bool>("setCameraParameter", m_handle, parameterId, value);

  return result;
}

