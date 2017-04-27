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

#include <qi/log.hpp>
#include <qi/applicationsession.hpp>
#include <qi/anyobject.hpp>

// Opencv includes for cv::Mat interface
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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

    vpNaoqiGrabber g(session);
    g.setRobotIp("131.254.13.37");
    g.setFramerate(30);
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

protected:
  qi::AnyObject m_pVideo; //!< Video proxy
  qi::AnyObject m_pMemory; //!< Memory proxy
  std::string m_handle; //!< Handle Video proxy
  int m_fps; //!< Requested frame per second
  bool m_isOpen; //!< Proxy opened status
  int m_width; //!<  Image width
  int m_height; //!< Image height
  cv::Mat m_img; //!< Image data
  std::string m_cameraName; //!< Camera name
  int m_cameraId; //!< Camera identifier
  int m_resolution;  //!< Resolution camera
  std::string m_robotType; //!< Nao, Pepper or Romeo
  bool m_romeo; //!< True if robot is Romeo
  bool m_pepper; //!< True if robot is Pepper
  bool m_nao; //!< True if robot is Nao

  // Multi stream
  bool m_cameraMulti;

public:
  vpNaoqiGrabber(const qi::SessionPtr &session);
  virtual ~vpNaoqiGrabber();

  void acquire(vpImage<unsigned char> &I);
  void acquire(vpImage<unsigned char> &I, struct timeval &timestamp);
  void acquire(cv::Mat &I);
  void acquire(cv::Mat &I, struct timeval &timestamp);

  void cleanup();

  vpCameraParameters getCameraParameters(vpCameraParameters::vpCameraParametersProjType projModel=vpCameraParameters::perspectiveProjWithDistortion) const;
  vpCameraParameters getCameraParameters(const int & resolution, const std::string &cameraName, vpCameraParameters::vpCameraParametersProjType projModel=vpCameraParameters::perspectiveProjWithDistortion) const;

  static vpCameraParameters getIntrinsicCameraParameters(const int & resolution, const std::string &cameraName, vpCameraParameters::vpCameraParametersProjType projModel);
  static vpHomogeneousMatrix getExtrinsicCameraParameters(const std::string &cameraName, vpCameraParameters::vpCameraParametersProjType projModel) ;

  vpHomogeneousMatrix get_eMc(vpCameraParameters::vpCameraParametersProjType projModel=vpCameraParameters::perspectiveProjWithDistortion, std::string cameraName = "" ) const;

  std::string getCameraName(){return m_cameraName;}
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
      qi::AnyObject *videoProxy = g.getProxy();
    }
    \endcode
    \return The address of the video proxy.
   */
     qi::AnyObject getProxy() const
    {
      return m_pVideo;
    }


  void open();

  void setCamera(const int &camera_id);

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
    Set the camera resolution.
    \param resolution : Index camera resolution.

    \sa open()
  */
  void setCameraResolution(const int &resolution)
  {
    m_resolution = resolution;
  }

  bool setCameraParameter( const int& parameterId, const int& value);

};

#endif
