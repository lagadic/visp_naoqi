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
#include <visp/vpMatrix.h>



/*!
  This class allows to get images from the robot remotely.
 */
class vpNaoqiRobot
{
public:

  vpNaoqiRobot();
  virtual ~vpNaoqiRobot();
  void cleanup();
  void open();
  void setCollisionProtection(bool protection)
  {
    m_collisionProtection = protection;
  }

  void setRobotIp(const std::string &robotIp)
  {
    m_robotIp = robotIp;
  }

  /*!
    Get the name of all the joints of the chain.

    \param chainName : Name of the chain. Allowed values are "Head",
    "LArm" for left arm and "RArm" for right arm.

    \return The name of the joints.
   */
  std::vector<std::string> getJointNames(const std::string &chainName);

  void setStiffness(const std::vector<std::string> &jointNames, float stiffness);
  void setStiffness(const std::string &chainName, float stiffness);

  void setVelocity(const std::vector<std::string> &jointNames, const std::vector<float> &jointVel);
  void setVelocity(const std::string &chainName, const std::vector<float> &jointVel);

  void stop(const std::vector<std::string> &jointNames);
  void stop(const std::string &chainName);



  /*!
    Get the angles of all the joints of the chain.

    \param names :  Names the joints, chains, “Body”, “JointActuators”, “Joints” or “Actuators”.
    \param useSensors :  If true, sensor angles will be returned.

    \return 	Joint angles in radians.
   */
  std::vector<float> getAngles(const AL::ALValue& names, const bool& useSensors);


  /*!
    Get the angles of all the joints of the chain.

    \param names :  Names the joints, chains, “Body”, “JointActuators”, “Joints” or “Actuators”.
    \param angles :  One or more angles in radians.
    \param fractionMaxSpeed :  The fraction of maximum speed to use.

   */
  void setAngles(const AL::ALValue& names, const AL::ALValue& angles, const float& fractionMaxSpeed);

  /*!
    Get the Jacobian specifying and end effector name

    \param endEffectorName : Name of the end effector. Allowed values are "Head",
    "LArm" for left arm and "RArm" for right arm.

    \return The actual jacobian.
   */
 vpMatrix getJacobian(const std::string &endEffectorName);

protected:
  AL::ALMotionProxy *m_motionProxy;
  std::string m_robotIp;
  bool m_isOpen;
  bool m_collisionProtection;
};

#endif
