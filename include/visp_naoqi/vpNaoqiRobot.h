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
 * Giovanni Claudio
 *
 *****************************************************************************/
#ifndef vpNaoqiRobot_h
#define vpNaoqiRobot_h

#include <iostream>
#include <string>


//// Aldebaran includes
#include <qi/applicationsession.hpp>
#include <qi/anyobject.hpp>

//// ViSP includes
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpMatrix.h>
#include <visp/vpRobotException.h>

#define BOOST_SIGNALS_NO_DEPRECATION_WARNING


/*!
  This class allows to control the robot remotely.

  The following example shows how to use this class to move NeckYaw and NeckPitch joints in velocity.
  \code
  #include <visp_naoqi/vpNaoqiRobot.h>

  int main()
  {
    vpNaoqiRobot robot;
    robot.open();

    // Test with a vector of joints
    std::vector<std::string> jointNames;
    jointNames.push_back("NeckYaw");
    jointNames.push_back("NeckPitch");

    std::vector<float> jointVel( jointNames.size() );
    for (unsigned int i=0; i < jointVel.size(); i++)
      jointVel[i] = vpMath::rad(2);

    robot.setStiffness(jointNames, 1.f);

    double t_initial = vpTime::measureTimeSecond();
    while (vpTime::measureTimeSecond() < t_initial+3)
    {
      robot.setVelocity(jointNames, jointVel);
    }

    robot.stop(jointNames);
  }
  \endcode

 */
class vpNaoqiRobot
{

public:
  typedef enum {
    Romeo,
    Nao,
    Pepper,
    Unknown
  } RobotType;

protected:
  qi::AnyObject m_pMemory; //!< Memory proxy
  qi::AnyObject m_pMotion;  //!< Motion proxy
  qi::AnyObject m_pepper_control;  //!< Proxy to Pepper_control
  bool m_isOpen; //!< Proxy opened status
  bool m_collisionProtection; //!< Collition protection enabling status
  std::string m_robotName; //!< Name of the robot
  RobotType m_robotType; //!< Indicate if the robot is Romeo, Nao or Pepper


public:

  vpNaoqiRobot(const qi::SessionPtr &session);
  virtual ~vpNaoqiRobot();

  /*!
    Destroy the connexion to the motion proxy.
   */
  void cleanup();

  std::vector<float> getAngles(const std::string &name, const bool& useSensors=true) const;
  std::vector<float> getAngles(const std::vector<std::string> &name, const bool& useSensors=true) const;

  vpHomogeneousMatrix get_cMe(const std::string &endEffectorName);
  vpMatrix get_eJe(const std::string &chainName) const;
  vpMatrix get_eJe(const std::string &chainName, vpMatrix & tJe) const;
  std::vector <vpMatrix> get_d_eJe(const std::string &chainName) const;

  std::vector<std::string> getBodyNames(const std::string &names) const;
  vpColVector getJointMin(const std::string &name) const;
  vpColVector getJointMax(const std::string &name) const;
  void getJointMinAndMax(const std::vector<std::string>& names, vpColVector &min, vpColVector &max) const;

  vpColVector getPosition(const std::string &names, const bool &useSensors=true) const;
  void getPosition(const std::vector<std::string> &names, std::vector<float> &q, const bool& useSensors=true) const;

  qi::AnyObject * getMotionProxy();

  std::vector<std::vector<float>> getLimits(const std::string & name) const;

  std::string getRobotName() const { return m_robotName; }

  RobotType getRobotType() const { return m_robotType; }

  vpHomogeneousMatrix getTransform(const std::string &name, const unsigned int &reference) const;

  vpColVector getJointVelocity(const std::vector<std::string> &names) const;

  void getJointVelocity(const std::vector<std::string> &names, std::vector<float> &jointVel) const;

  void open();

  void moveTo(const float& x, const float& y, const float& theta) const;

  /*!
    Enable/disable the collision protection.
    In the constructor, the collision protection is enabled by default.
    \param protection : true to enable collision protection, false to disable.
   */
  void setCollisionProtection(bool protection)
  {
    m_collisionProtection = protection;
  }

  void setExternalCollisionProtectionEnabled(const std::string& name, const bool& enable) const;

  void setPosition(const std::string &name, const float &angles, const float &fractionMaxSpeed) const;
  void setPosition(const std::vector<std::string> &names, const vpColVector &jointPosition, const float &fractionMaxSpeed) const;
  void setPosition(const std::vector<std::string> &names, const std::vector<float> &jointPosition, const float &fractionMaxSpeed) const;

  void setStiffness(const std::string &names, const float &stiffness) const;
  void setStiffness(const std::vector<std::string> &names, const std::vector<float> &stiffness) const;
  void setStiffness(const std::vector<std::string> &names, const float &stiffness) const;

  //void setVelocity_eachJoint(const AL::ALValue& names, const AL::ALValue &jointVel, bool verbose=false);
  //void setVelocity_eachJoint(const AL::ALValue& names, const vpColVector &jointVel, bool verbose=false);
  //void setVelocity_eachJoint(const AL::ALValue& names, const std::vector<float> &jointVel, bool verbose=false);
  // void setVelocity(const AL::ALValue& names, const std::vector<float> &jointVel, bool verbose=false);
  void setVelocity(const std::vector<std::string> &names, const vpColVector &jointVel) const;
 // void setVelocity(const AL::ALValue& names, const AL::ALValue &jointVel, bool verbose=false);
  void setVelocity(const std::vector<std::string> &names, const std::vector<float> &jointVel) const;

  void setBaseVelocity(const std::vector<float> &jointVel) const;
  void setBaseVelocity(const vpColVector &jointVel) const;
  void setBaseVelocity(const float &vx,const float &vy,const float &wz) const;

  void startPepperControl() const;
  void stop(const std::string &name) const;
  void stop(const std::vector<std::string> &names) const;
  void stopPepperControl() const;
  void stopBase() const;



};

#endif
