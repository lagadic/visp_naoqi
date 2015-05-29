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
#include <alcommon/albroker.h>

// ViSP includes
#include <visp/vpImageConvert.h>
#include <visp/vpXmlParserCamera.h>

#include <visp_naoqi/vpNaoqiConfig.h>
#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>

/*!
  Default constructor that set the default parameters as:
  - robot ip address: "198.18.0.1"
  - robot port: 9559
  - camera framerate: 30 fps
  */
vpNaoqiGrabber::vpNaoqiGrabber()
  : m_videoProxy(NULL), m_robotIp("198.18.0.1"),
    m_robotPort(9559), m_fps(30), m_isOpen(false), m_width(0), m_height(0),
    m_img(), m_cameraName("CameraLeft"), m_cameraId (0), m_cameraMulti(false)
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
  \param camera_id : Camera identifier; CameraLeft(0), CameraRight(1),CameraLeftEye(2), CameraRightEye(3)
 */
void vpNaoqiGrabber::setCamera(const int &camera_id)
{

  // TODO: add specialisation Romeo/Nao
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
    m_cameraId = camera_id;
  }
  else if (camera_id == 3)
  {
    m_cameraName = "CameraRightEye";
    m_cameraId = camera_id;
  }
  else if (camera_id == 4)
  {
    m_cameraName = "Camera3D";
    m_cameraId = camera_id;
  }

  return;

}

/*!
  Select the cameras to use.
  \param cameras_id : The cameras identifiers: 0 = head-front cameras, 1 = eye's cameras.
 */
void vpNaoqiGrabber::setCamerasMulti(const int &cameras_id)
{
  m_cameraMulti = true;
  m_cameraId = cameras_id;


  return;
}

void vpNaoqiGrabber::open()
{
  if (! m_isOpen) {
    // Create a proxy to ALVideoDevice on the robot
    m_videoProxy = new AL::ALVideoDeviceProxy(m_robotIp, m_robotPort);
    // Subscribe a client image requiring 320*240 and BGR colorspace
    m_clientName = "subscriberID";
    m_videoProxy->unsubscribeAllInstances(m_clientName);

    if (m_cameraName.find("Left") != std::string::npos)
      m_clientName = m_videoProxy->subscribeCamera(m_clientName, 0, AL::kQVGA, AL::kBGRColorSpace, m_fps);
    else if (m_cameraName.find("Right") != std::string::npos)
      m_clientName = m_videoProxy->subscribeCamera(m_clientName, 1, AL::kQVGA, AL::kBGRColorSpace, m_fps);
    else if (m_cameraName.find("3D") != std::string::npos)
      m_clientName = m_videoProxy->subscribeCamera(m_clientName, 2, AL::kQVGA, AL::kBGRColorSpace, m_fps);
    //m_clientName = m_videoProxy->subscribe(m_clientName, AL::k4VGA, AL::kBGRColorSpace, m_fps);

    //    // Select the camera left(0) or right(1)
    //    if (m_cameraName.find("Left") != std::string::npos)
    //      m_videoProxy->setCameraParameter(m_clientName, AL::kCameraSelectID, 0);
    //    else
    //      m_videoProxy->setCameraParameter(m_clientName, AL::kCameraSelectID, 1);



    // Select Camera Front(0) or Eyes(1)
    //    boost::shared_ptr<AL::ALBroker> broker = AL::ALBroker::createBroker("Broker", "", 0, m_robotIp, 9559);
    //    AL::ALProxy *proxy = new AL::ALProxy(broker, "ALVideoDevice");

    //    if (m_cameraName.find("3D") == std::string::npos)
    //    {
    //    if (m_cameraName.find("Eye") != std::string::npos)
    //      proxy->callVoid("setCameraGroup", 1, true);
    //    else
    //      proxy->callVoid("setCameraGroup", 0, true);

    //}
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

    // vpTime::sleepMs(5000);


    //    double t_initial = vpTime::measureTimeSecond();
    //    while (vpTime::measureTimeSecond() < t_initial+7.0)
    //    {
    //       proxy->call("getCameraGroup", true);
    //      std::cout << "Camera: " << a<< std::endl;

    //    }
    //    delete proxy;
    //    proxy = NULL;

    m_img = m_videoProxy->getImageRemote(m_clientName);
    // vpTime::sleepMs(2000);

    //  try {
    m_width  = (int) m_img[0];
    m_height = (int) m_img[1];
    //      }
    //      catch(...) {
    //        std::cout << "Catch an exception 2" << std::endl;
    //      }
    m_videoProxy->releaseImage(m_clientName);



    m_isOpen = true;
  }
}


void vpNaoqiGrabber::openMulti()
{

  if (!m_cameraMulti)
  {
    std::cout << "You have to call the function vpNaoqiGrabber::setCamerasMulti() before vpNaoqiGrabber::openMulti()." << std::endl;
    std::cout << "Please check and modify your code." << std::endl;
    exit(0);

  }

  if (! m_isOpen) {
    // Create a proxy to ALVideoDevice on the robot
    m_videoProxy = new AL::ALVideoDeviceProxy(m_robotIp, m_robotPort);
    // Subscribe a client image requiring 320*240 and BGR colorspace
    m_clientName = "subscriberIDMulti";
    m_videoProxy->unsubscribeAllInstances(m_clientName);

    AL::ALValue camerasId =   AL::ALValue::array(0, 1);
    AL::ALValue camerasResolution = AL::ALValue::array(AL::kQVGA, AL::kQVGA);
    AL::ALValue camerasColorSpace = AL::ALValue::array(AL::kBGRColorSpace, AL::kBGRColorSpace);

    m_clientName = m_videoProxy->subscribeCameras(m_clientName, camerasId , camerasResolution, camerasColorSpace, m_fps);

    //    boost::shared_ptr<AL::ALBroker> broker = AL::ALBroker::createBroker("Broker", "", 0, m_robotIp, 9559);
    //    AL::ALProxy *proxy = new AL::ALProxy(broker, "ALVideoDevice");

    //    // Select Camera Front(0) or Eyes(1)
    //    if (m_cameraId)
    //      proxy->callVoid("setCameraGroup", 1, true);
    //    else
    //      proxy->callVoid("setCameraGroup", 0, true);

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
    m_img = m_videoProxy->getImagesRemote(m_clientName);
    m_width  = (int) m_img[0][0];
    m_height = (int) m_img[0][1];
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
  //m_videoProxy->releaseImage(m_clientName);

  I.resize(m_height, m_width);

  vpImageConvert::BGRToGrey(img_buffer, (unsigned char *)I.bitmap, m_width, m_height);
}

// /*!

//  The image is copied.

// */
//void vpNaoqiGrabber::acquire(vpImage<vpRGBa> &I)
//{
//  struct timeval timestamp;
//  acquire(I, timestamp);
//}

/////*!

//  The image is copied.

// */
//void vpNaoqiGrabber::acquire(vpImage<vpRGBa> &I, struct timeval &timestamp)
//{
//  if (! m_isOpen)
//    open();

//  /* Retrieve an image from the camera.
//   * The image is returned in the form of a container object, with the
//   * following fields:
//   * 0 = width
//   * 1 = height
//   * 2 = number of layers
//   * 3 = colors space index (see alvisiondefinitions.h)
//   * 4 = time stamp (seconds)
//   * 5 = time stamp (micro seconds)
//   * 6 = image buffer (size of width * height * number of layers)
//   */
//  m_img = m_videoProxy->getImageRemote(m_clientName);

//  m_width  = (int) m_img[0];
//  m_height = (int) m_img[1];
//  double tv_sec  = (double)m_img[4];
//  double tv_usec = (double)m_img[5];
//  timestamp.tv_sec  = (unsigned long) tv_sec;
//  timestamp.tv_usec = (unsigned long) tv_usec;


//  //  // Access the image buffer (6th field) and assign it to the ViSP image container
//  //  unsigned char *img_buffer = (unsigned char *) m_img[6].GetBinary();
//  //  I.resize(m_height, m_width);
//  //  memcpy(I.bitmap, img_buffer,4*m_height*m_width);

//  cv::Mat Img = cv::Mat(cv::Size(m_width, m_height), CV_8UC3);
//  Img.data = (unsigned char*) m_img[6].GetBinary();
//  vpImageConvert::convert(Img, I);


//}




/*!

  The image is copied.

 */
void vpNaoqiGrabber::acquireMulti(vpImage<unsigned char> &Ia,vpImage<unsigned char> &Ib)
{
  struct timeval timestamp_a;
  struct timeval timestamp_b;
  acquireMulti(Ia, Ib, timestamp_a, timestamp_b);
}

/*!

  The image is copied.

 */
void vpNaoqiGrabber::acquireMulti(vpImage<unsigned char> &Ia, vpImage<unsigned char> &Ib, struct timeval &timestamp_a, struct timeval &timestamp_b)
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
  m_img = m_videoProxy->getImagesRemote(m_clientName);

  m_width  = (int) m_img[0][0];
  m_height = (int) m_img[0][1];

  // Left Image
  double tv_sec  = (double)m_img[0][4];
  double tv_usec = (double)m_img[0][5];
  timestamp_a.tv_sec  = (unsigned long) tv_sec;
  timestamp_a.tv_usec = (unsigned long) tv_usec;
  //Rigth image
  tv_sec  = (double)m_img[1][4];
  tv_usec = (double)m_img[1][5];
  timestamp_a.tv_sec  = (unsigned long) tv_sec;
  timestamp_a.tv_usec = (unsigned long) tv_usec;

  // Access the image buffer (6th field) and assign it to the ViSP image container
  unsigned char *img_buffer_a = (unsigned char *) m_img[0][6].GetBinary();
  unsigned char *img_buffer_b = (unsigned char *) m_img[1][6].GetBinary();

  // Tells to ALVideoDevice that it can give back the image buffer to the
  // driver. Optional after a getImageRemote but MANDATORY after a getImageLocal.
  //m_videoProxy->releaseImage(m_clientName);

  Ia.resize(m_height, m_width);
  Ib.resize(m_height, m_width);

  vpImageConvert::BGRToGrey(img_buffer_a, (unsigned char *)Ia.bitmap, m_width, m_height);
  vpImageConvert::BGRToGrey(img_buffer_b, (unsigned char *)Ib.bitmap, m_width, m_height);
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

  The image is not copied. Here we just update to pointer to the NaoQi image.

  \warning Should be improved to detect the type cv::CV_8UC3, cv::CV_8UC1
 */

void vpNaoqiGrabber::acquireMulti(cv::Mat &Ia, cv::Mat &Ib)
{
  struct timeval timestamp_a;
  struct timeval timestamp_b;
  acquireMulti(Ia, Ib, timestamp_a, timestamp_b);
}

/*!

  The image is not copied. Here we just update to pointer to the NaoQi image.

  \warning Should be improved to detect the type cv::CV_8UC3, cv::CV_8UC1
 */
void vpNaoqiGrabber::acquireMulti(cv::Mat &Ia, cv::Mat &Ib, struct timeval &timestamp_a,struct timeval &timestamp_b)
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
  m_img = m_videoProxy->getImagesRemote(m_clientName);

  m_width  = (int) m_img[0][0];
  m_height = (int) m_img[0][1];

  // Left Image
  double tv_sec  = (double)m_img[0][4];
  double tv_usec = (double)m_img[0][5];
  timestamp_a.tv_sec  = (unsigned long) tv_sec;
  timestamp_a.tv_usec = (unsigned long) tv_usec;
  //Rigth image
  tv_sec  = (double)m_img[1][4];
  tv_usec = (double)m_img[1][5];
  timestamp_a.tv_sec  = (unsigned long) tv_sec;
  timestamp_a.tv_usec = (unsigned long) tv_usec;

  // Access the image buffer (6th field) and assign it to the opencv image container
  Ia.data = (unsigned char*) m_img[0][6].GetBinary();
  Ib.data = (unsigned char*) m_img[1][6].GetBinary();
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
  }

  return cam;
}





/*!
  Return the extrinsic camera parameters corresponding to the camera that is selected using setCamera().
  \warning The grabber should be open prior calling this function.

  \param cameraName : Name of the Camera (Null string = camera corresponding to the camera that is selected using setCamera() )
  \param projModel : Model that is used.(default = Projection with distorsion )

  \return The extrinsic camera parameters (Homogeneous matrix)

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
  vpHomogeneousMatrix eMc = g.get_eMc();
}

 */
vpHomogeneousMatrix
vpNaoqiGrabber::get_eMc(vpCameraParameters::vpCameraParametersProjType projModel, std::string cameraName ) const
{

  vpHomogeneousMatrix eMc;
  std::string name;
  vpXmlParserHomogeneousMatrix p; // Create a XML parser

  if (cameraName == "")
    cameraName = m_cameraName;


  if (cameraName == "CameraLeftEye" || cameraName == "CameraRightEye" )
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

    }

    else
      std::cout << "Read correctly the Homogeneous matrix named " << name << std::endl;
  }

  return eMc;
}


bool vpNaoqiGrabber::setCameraParameter(const int& parameterId, const int& value)
{
  bool result = m_videoProxy->setCameraParameter(m_clientName, parameterId, value);
  return result;
}

