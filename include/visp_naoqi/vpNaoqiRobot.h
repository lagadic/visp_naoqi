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
 * This class allows to control the robot remotely.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
#ifndef vpNaoqiRobot_h
#define vpNaoqiRobot_h

#include <iostream>
#include <string>

// ViSP includes
#include <visp/vpRobotException.h>

// Aldebaran includes
#include <alproxies/almotionproxy.h>
#include <alerror/alerror.h>



/*!
  This class allows to get images from the robot remotely.
 */
class vpNaoqiRobot
{
public:

  vpNaoqiRobot();
  virtual ~vpNaoqiRobot();
  void cleanup();

  std::vector<std::string> getJointNames(const std::string &chainName);

  void open();
  void setCollisionProtection(bool protection)
  {
    m_collisionProtection = protection;
  };

  void setRobotIp(const std::string &robotIp)
  {
    m_robotIp = robotIp;
  }

  void setStiffness(const std::vector<std::string> &jointNames, float stiffness);
  void setStiffness(const std::string &chainName, float stiffness);

  void setVelocity(const std::vector<std::string> &jointNames, const std::vector<float> &jointVel);
  void setVelocity(const std::string &chainName, const std::vector<float> &jointVel);

  void stop(const std::vector<std::string> &jointNames);
  void stop(const std::string &chainName);

protected:
  AL::ALMotionProxy *m_motionProxy;
  std::string m_robotIp;
  bool m_isOpen;
  bool m_collisionProtection;
};

#endif
