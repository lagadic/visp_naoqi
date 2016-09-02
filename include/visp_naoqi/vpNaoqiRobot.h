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


// Aldebaran includes
#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alcommon/alproxy.h>
#include <alerror/alerror.h>


// ViSP includes
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

  vpNaoqiRobot();
  virtual ~vpNaoqiRobot();

  /*!
    Destroy the connexion to the motion proxy.
   */
  void cleanup();

  /*!
    Get the Transformation matrix to the end-effectors

    \param endEffectorName : Name of the end effector. Allowed values are "CameraLeft",
    "CameraRight" for the fixed camera on the head, "RHand" and "LHand" for end-effector on the arms.

    \return The transformation matrix to the end-effectors (computed from the last joint of the chain ending with the end-effector. )
   */
  vpHomogeneousMatrix get_cMe(const std::string &endEffectorName);



  /*!
    Get the Jacobian specifying an end effector chain name.

    \param chainName : Name of the end effector. Allowed values are "Head",
    "LArm" for left arm and "RArm" for right arm.

    \return The actual jacobian with respect to the end effector.
   */
  vpMatrix get_eJe(const std::string &chainName) const;



  /*!
    Get the Jacobian specifying an end effector chain name.

    \param chainName : Name of the end effector. Allowed values are "Head",
    "LArm" for left arm and "RArm" for right arm.

    \param tJe : Jacobian with respect to the torso

    \return The actual jacobian with respect to the end effector.
   */
  vpMatrix get_eJe(const std::string &chainName, vpMatrix & tJe) const;
  /*!
    Get the derivative of the kinematic jacobian specifying an end effector chain name.

    \param chainName : Name of the end effector. Allowed values are "Head",
    "LArm" for left arm and "RArm" for right arm.

    \return The actual derivative of the kinematic jacobian.
   */
  std::vector <vpMatrix> get_d_eJe(const std::string &chainName) const;
  /*!
    Get the name of all the joints of the chain.

    \param names :  Names the joints, chains, "Body", "JointActuators",
    "Joints" or "Actuators".

    \return The name of the joints.
   */
  std::vector<std::string> getBodyNames(const std::string &names) const;
  vpColVector getJointMin(const AL::ALValue& names) const;
  vpColVector getJointMax(const AL::ALValue& names) const;
  void getJointMinAndMax(const std::vector<std::string>& names, vpColVector &min, vpColVector &max) const;

  /*!
    Get the position of all the joints of the chain.

    \param names :  Names the joints, chains, "Body", "JointActuators",
    "Joints" or "Actuators".
    \param useSensors :  If true, sensor positions will be returned. If
    false, it will be the command.

    \return Joint position in radians.
   */
  vpColVector getPosition(const AL::ALValue& names, const bool& useSensors=true) const;

  /*!
    Get the position of all the joints of the chain.

    \param names :  Names the joints, chains, "Body", "JointActuators",
    "Joints" or "Actuators".
    \param useSensors :  If true, sensor positions will be returned. If
    false, it will be the command.
    \param q : Joint position in radians.
   */
  void getPosition(const AL::ALValue& names, std::vector<float>& q, const bool& useSensors=true) const;


  /*!
   Return the video proxy used to grab images.

   \code
   #include <visp_naoqi/vpNaoqiRobot.h>

   int main()
   {
     vpNaoqiRobot robot;
     ...
     robot.open();
     AL::ALMotionProxy *motionProxy = robot.getProxy();
   }
   \endcode
   \return The address of the video proxy.
  */
  AL::ALMotionProxy *getProxy() const
  {
    return m_motionProxy;
  }

  /*!
   Return the ip address used to access to the robot.
  */
  std::string getRobotIp() const { return m_robotIp; }
  /*!
   Return the name of the robot.
   Values could be "romeoH37", "naoH25", "naoH21", "naoT14", "naoT2"...
  */
  std::string getRobotName() const { return m_robotName; }

  /*!

     Return the type of the robot (Romeo, Nao, Pepper).
   */
  RobotType getRobotType() const { return m_robotType; }

  /*!
    Get the joints velocities.

    \param names :  Names of the joints, chains, "Body", "JointActuators",
    "Joints" or "Actuators".

    \return Joint velocities in radians/s.
   */
  vpColVector getJointVelocity(const std::vector<std::string> &names) const;

  /*!
    Get the joints velocities.

    \param names :  Names of the joints, chains, "Body", "JointActuators",
    "Joints" or "Actuators".
    \param useSensors :  Joint velocities in radians/s.

   */
  void getJointVelocity(const std::vector<std::string> &names, std::vector<float> &jointVel) const;

  /*!
    Open the connection with the robot.
    All the parameters should be set before calling this function.
    \code
    #include <visp_naoqi/vpNaoqiRobot.h>

    int main()
    {
      vpNaoqiRobot robot;
      robot.setRobotIp("131.254.13.37");
      robot.open();
    }
    \endcode
   */
  void open();

  /*!
    Enable/disable the collision protection.
    In the constructor, the collision protection is enabled by default.
    \param protection : true to enable collision protection, false to disable.
   */
  void setCollisionProtection(bool protection)
  {
    m_collisionProtection = protection;
  }

  /*!
    Set the position of all the joints of the chain.

    \param names :  Names the joints, chains, "Body", "JointActuators", "Joints" or "Actuators".
    \param angles :  One or more joint positions in radians.
    \param fractionMaxSpeed : The fraction of maximum speed to use. Value should be comprised between 0 and 1.

   */
  void setPosition(const AL::ALValue& names, const AL::ALValue& angles, const float& fractionMaxSpeed);
  /*!
    Set the position of all the joints of the chain.

    \param names :  Names the joints, chains, "Body", "JointActuators", "Joints" or "Actuators".
    \param jointPosition :  One or more joint positions in radians.
    \param fractionMaxSpeed : The fraction of maximum speed to use. Value should be comprised between 0 and 1.

   */
  void setPosition(const AL::ALValue& names, const vpColVector &jointPosition, const float& fractionMaxSpeed);
  /*!
    Set the position of all the joints of the chain.

    \param names :  Names the joints, chains, "Body", "JointActuators", "Joints" or "Actuators".
    \param jointPosition :  One or more joint positions in radians.
    \param fractionMaxSpeed : The fraction of maximum speed to use. Value should be comprised between 0 and 1.

   */
  void setPosition(const AL::ALValue& names, const std::vector<float> &jointPosition, const float& fractionMaxSpeed);
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

  void setStiffness(const AL::ALValue& names, float stiffness);

  void setVelocity_eachJoint(const AL::ALValue& names, const AL::ALValue &jointVel, bool verbose=false);
  void setVelocity_eachJoint(const AL::ALValue& names, const vpColVector &jointVel, bool verbose=false);
  void setVelocity_eachJoint(const AL::ALValue& names, const std::vector<float> &jointVel, bool verbose=false);
  void setVelocity(const AL::ALValue& names, const std::vector<float> &jointVel, bool verbose=false);
  void setVelocity(const AL::ALValue& names, const vpColVector &jointVel, bool verbose=false);
  void setVelocity(const AL::ALValue& names, const AL::ALValue &jointVel, bool verbose=false);

  void stop(const AL::ALValue& names) const;

protected:
  AL::ALMotionProxy *m_motionProxy; //!< Motion proxy
  AL::ALProxy *m_proxy; //!< General Proxy
  std::string m_robotIp; //!<  Robot Ethernet address
  bool m_isOpen; //!< Proxy opened status
  bool m_collisionProtection; //!< Collition protection enabling status
  std::string m_robotName; //!< Name of the robot
  RobotType m_robotType; //!< Indicate if the robot is Romeo, Nao or Pepper
  AL::ALMemoryProxy * m_memProxy; //!< Memory Proxy

};

#endif
