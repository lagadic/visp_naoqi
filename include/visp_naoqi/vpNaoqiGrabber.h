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
#ifndef vpNaoqiGrabber_h
#define vpNaoqiGrabber_h

#include <iostream>
#include <string>
#include <sys/time.h>

// Aldebaran includes
#include <alproxies/alvideodeviceproxy.h>
#include <alerror/alerror.h>

// Opencv includes for cv::Mat interface
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// ViSP includes
#include <visp/vpImage.h>

/*!
  This class allows to get images from the robot remotely.
 */
class vpNaoqiGrabber
{
public:
  vpNaoqiGrabber();
  virtual ~vpNaoqiGrabber();
  void setRobotIp(const std::string &robotIp)
  {
    m_robotIp = robotIp;
  }
  void setRobotPort(int robotPort)
  {
    m_robotPort = robotPort;
  }

  void acquire(vpImage<unsigned char> &I);
  void acquire(vpImage<unsigned char> &I, struct timeval &timestamp);
  void acquire(cv::Mat &I);
  void acquire(cv::Mat &I, struct timeval &timestamp);
  void cleanup();
  unsigned int getWidth()
  {
    return (unsigned int)m_width;
  }
  unsigned int getHeight()
  {
    return (unsigned int)m_height;
  }
  void open();
  void setFramerate(int fps)
  {
    m_fps = fps;
  }

  void release();

protected:
  AL::ALVideoDeviceProxy *m_camProxy;
  std::string m_robotIp;
  int m_robotPort;
  std::string m_clientName;
  int m_fps;
  bool m_isOpen;
  int m_width;
  int m_height;
};

#endif
