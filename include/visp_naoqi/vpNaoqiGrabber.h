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
#ifndef vpNaoqiGrabber_h
#define vpNaoqiGrabber_h

#include <iostream>
#include <string>
#include <sys/time.h>

// Aldebaran includes
#include <alerror/alerror.h>
#include <alvision/alvisiondefinitions.h>
#include <alproxies/alvideodeviceproxy.h>

// Opencv includes for cv::Mat interface
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// ViSP includes
#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpHomogeneousMatrix.h>

/*!
  This class allows to get images from the robot remotely.

  The following example shows how to use this class to grab gray level images.
  \code
  #include <iostream>
  #include <visp_naoqi/vpNaoqiGrabber.h>

  int main()
  {
    vpImage<unsigned char> I;

    vpNaoqiGrabber g;
    g.setRobotIp("131.254.13.37");
    g.setFramerate(15);
    g.setCamera(0);
    g.open();
    std::cout << "Image size: " << g.getWidth() << " " << g.getHeight() << std::endl;
    while(1)
    {
      g.acquire(I);
    }
  }
  \endcode
 */
class vpNaoqiGrabber
{
public:
  vpNaoqiGrabber();
  virtual ~vpNaoqiGrabber();

  void acquire(vpImage<unsigned char> &I);
  void acquire(vpImage<unsigned char> &I, struct timeval &timestamp);
  //void acquire(vpImage<vpRGBa> &I);
  //void acquire(vpImage<vpRGBa> &I, struct timeval &timestamp);
  void acquire(cv::Mat &I);
  void acquire(cv::Mat &I, struct timeval &timestamp);

  void acquireMulti(vpImage<unsigned char> &Ia,vpImage<unsigned char> &Ib);
  void acquireMulti(vpImage<unsigned char> &Ia, vpImage<unsigned char> &Ib, struct timeval &timestamp_a, timeval &timestamp_b);

  void acquireMulti(cv::Mat &Ia, cv::Mat &Ib);
  void acquireMulti(cv::Mat &Ia, cv::Mat &Ib, struct timeval &timestamp_a, timeval &timestamp_b);

  /*!
    Destroy the connexion to the video proxy.
   */
  void cleanup();

  vpCameraParameters getCameraParameters(vpCameraParameters::vpCameraParametersProjType projModel=vpCameraParameters::perspectiveProjWithDistortion) const;

  vpCameraParameters getCameraParameters( const int & resolution, const std::string &cameraName, vpCameraParameters::vpCameraParametersProjType projModel=vpCameraParameters::perspectiveProjWithDistortion) const;


  vpHomogeneousMatrix get_eMc(vpCameraParameters::vpCameraParametersProjType projModel=vpCameraParameters::perspectiveProjWithDistortion, std::string cameraName = "" ) const;

  std::string getCameraName(){return m_cameraName;}

  std::string getClientName(){return m_clientName;}
  /*!

     \return Image width.
   */
  unsigned int getWidth() const
  {
    return (unsigned int)m_width;
  }
  /*!

     \return Image height.
   */
  unsigned int getHeight() const
  {
    return (unsigned int)m_height;
  }
  /*!
    Return the video device proxy used to grab images.

    \code
    #include <visp_naoqi/vpNaoqiGrabber.h>

    int main()
    {
      vpNaoqiGrabber g;
      ...
      g.open();
      AL::ALVideoDeviceProxy *videoProxy = g.getProxy();
    }
    \endcode
    \return The address of the video proxy.
   */
  AL::ALVideoDeviceProxy *getProxy() const
  {
    return m_videoProxy;
  }

  /*!
    Return the ip address used to access to the robot.
   */
  std::string getRobotIp() const { return m_robotIp; }
  /*!
    Return the Ethernet port used to access to the robot.
   */
  int getRobotPort() const { return m_robotPort; }

  /*!
    Open the connection with the robot.
    All the parameters should be set before calling this function.
    \code
    #include <visp_naoqi/vpNaoqiGrabber.h>

    int main()
    {
      vpNaoqiGrabber g;
      g.setRobotIp("131.254.13.37");
      g.setFramerate(15);
      g.open();
    }
    \endcode
   */
  void open();

  /*!
    Open the connection with the robot for stereo vision.
    All the parameters should be set before calling this function.
    \code
    #include <visp_naoqi/vpNaoqiGrabber.h>

    int main()
    {
      vpNaoqiGrabber g;
      g.setRobotIp("131.254.13.37");
      g.setFramerate(15);
      g.setCamerasMulti(0); // 0 to activate the front-head cameras, 1 for the eyes cameras
      g.openMulti();
    }
    \endcode
   */
  void openMulti();

  void setCamera(const int &camera_id);
  void setCamerasMulti(const int &cameras_id);

  /*!
    Set the camera framerate.
    In the constructor, the default framerate is set to 30 Hz.
    \param fps : New framerate in Hz.

    \sa open()
    */
  void setFramerate(int fps)
  {
    m_fps = fps;
  }

  /*!
    Set the robot ip address.
    In the constructor, the default ip is set to "198.18.0.1".
    \param robotIp : New robot ip address.

    \sa open()
  */
  void setRobotIp(const std::string &robotIp)
  {
    m_robotIp = robotIp;
  }

  /*!
    Set the robot port.
    In the constructor, the default port is set to 9559.
    \param robotPort : New robot port.

    \sa open()
  */
  void setRobotPort(int robotPort)
  {
    m_robotPort = robotPort;
  }

  /*!
    Set a camera parameter.
    \param parameterId : Camera parameter requested.
    \param value : Value requested.

    \return True if succesfull.
  */

  bool setCameraParameter( const int& parameterId, const int& value);



protected:
  AL::ALVideoDeviceProxy *m_videoProxy; //!< Video proxy
  std::string m_robotIp; //!< Robot Ethernet address
  int m_robotPort; //!< Robot Ethernet port
  std::string m_clientName; //!< Client name
  int m_fps; //!< Requested frame per second
  bool m_isOpen; //!< Proxy opened status
  int m_width; //!<  Image width
  int m_height; //!< Image height
  AL::ALValue m_img; //!< Image data
  std::string m_cameraName; //!< Camera name
  int m_cameraId; //!< Camera identifier

  // Multi stream
  bool m_cameraMulti;


};

#endif
