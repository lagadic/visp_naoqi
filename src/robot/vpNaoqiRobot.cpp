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
 * This class allows to the robot remotely.
 *
 * Authors:
 * Fabien Spindler
 * Giovanni Claudio
 *
 *****************************************************************************/



#include <visp_naoqi/vpNaoqiRobot.h>

/*!
  Default constructor that set the default parameters as:
  - robot ip address: "198.18.0.1"
  - collision protection: enabled
  */
vpNaoqiRobot::vpNaoqiRobot()
  : m_motionProxy(NULL), m_robotIp("198.18.0.1"), m_isOpen(false), m_collisionProtection(true)
{

}

/*!
  Destructor that call cleanup().
 */
vpNaoqiRobot::~vpNaoqiRobot()
{
  cleanup();
}

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
void vpNaoqiRobot::open()
{
  if (! m_isOpen) {
    cleanup();

    // Create a proxy to control the robot
    m_motionProxy = new AL::ALMotionProxy(m_robotIp);

    int success;
    if (m_collisionProtection == false)
      success = m_motionProxy->setCollisionProtectionEnabled("Arms", "False");
    else
      m_motionProxy->setCollisionProtectionEnabled("Arms", "True");
    std::cout << "Collision protection is " << m_collisionProtection << std::endl;

    // Set Trapezoidal interpolation
    AL::ALValue config;

    AL::ALValue one = AL::ALValue::array(std::string("ENABLE_DCM_LIKE_CLAMPING"),AL::ALValue(0));
    AL::ALValue two = AL::ALValue::array(std::string("CONTROL_USE_ACCELERATION_INTERPOLATOR"),AL::ALValue(1));
    AL::ALValue tree = AL::ALValue::array(std::string("CONTROL_JOINT_MAX_ACCELERATION"),AL::ALValue(2.5));
    config.arrayPush(one);
    config.arrayPush(two);
    config.arrayPush(tree);

    m_motionProxy->setMotionConfig(config);
    std::cout << "Trapezoidal interpolation is on " << std::endl;

    m_isOpen = true;
  }
}

void vpNaoqiRobot::cleanup()
{
  if (m_motionProxy != NULL) {
    delete m_motionProxy;
  }
  m_isOpen = false;
}

/*!
  Set the stiffness to a chain name, or to a specific joint.
  \param names :  Names the joints, chains, "Body", "JointActuators",
  "Joints" or "Actuators".
  \param stiffness : Stiffness parameter that should be between
  0 (no stiffness) and 1 (full stiffness).
 */
void vpNaoqiRobot::setStiffness(const AL::ALValue& names, float stiffness)
{
  m_motionProxy->setStiffnesses(names, AL::ALValue( stiffness ));
}

/*!
  Apply a velocity vector to a vector of joints.
  \param names :  Names the joints, chains, "Body", "JointActuators",
  "Joints" or "Actuators".
  \param jointVel : Joint velocity vector with values expressed in rad/s.
  \param verbose : If true activates printings.
 */

void vpNaoqiRobot::setVelocity(const AL::ALValue& names, const vpColVector &jointVel, bool verbose)
{
  std::vector<float> jointVel_(jointVel.size());
  for (unsigned int i=0; i< jointVel.size(); i++)
    jointVel_[i] = jointVel[i];
  setVelocity(names, jointVel_);
}

/*!
  Apply a velocity vector to a vector of joints.

  \todo Improve the function to be able to pass just one joint as names argument.

  \param names :  Names the joints, chains, "Body", "JointActuators",
  "Joints" or "Actuators".
  \param jointVel : Joint velocity vector with values expressed in rad/s.
  \param verbose : If true activates printings.
 */
void vpNaoqiRobot::setVelocity(const AL::ALValue &names, const AL::ALValue &jointVel, bool verbose)
{
  if (names.getSize() != jointVel.getSize() ) {
    throw vpRobotException (vpRobotException::readingParametersError,
                            "The dimensions of the joint array and the velocities array do not match.");
  }
  std::vector<std::string> jointNames;
  if (names.isString()) // Suppose to be a chain
    jointNames = m_motionProxy->getBodyNames(names);
  else if (names.isArray()) // Supposed to be a vector of joints
    jointNames = names; // it a vector of joints
  else
    throw vpRobotException (vpRobotException::readingParametersError,
                            "Unable to decode the joint chain.");

  for (unsigned i = 0 ; i < jointVel.getSize() ; ++i)
  {
    std::string jointName = jointNames[i];
    if (verbose)
      std::cout << "Joint name: " << jointName << std::endl;

    float vel = jointVel[i];

    if (verbose)
      std::cout << "Desired velocity =  " << vel << "rad/s to the joint " << jointName << "." << std::endl;

    //Get the limits of the joint
    AL::ALValue limits = m_motionProxy->getLimits(jointName);

    AL::ALValue angle = AL::ALValue( 0.0f);
    unsigned int applymotion = 0;

    if (vel > 0.0f)
    {
      //Reach qMax
      angle = limits[0][1];
      applymotion = 1;
      if (verbose)
        std::cout << "Reach qMax (" << angle << ") ";
    }

    else if (vel < 0.0f)
    {
      //Reach qMin
      angle = limits[0][0];
      applymotion = 1;
      if (verbose)
        std::cout << "Reach qMin (" << angle << ") ";
    }

    if (applymotion)
    {
      float fraction = fabs( float (vel/float(limits[0][2])));
      if (fraction >= 1.0 )
      {
        if (verbose) {
          std::cout << "Given velocity is too high: " <<  vel << "rad/s for " << jointName << "." << std::endl;
          std::cout << "Max allowed is: " << limits[0][2] << "rad/s for "<< std::endl;
        }
        fraction = 1.0;
      }
      m_motionProxy->setAngles(jointName,angle,fraction);
      if (verbose)
        std::cout << "SET VELOCITY TO " <<  vel << "rad/s for " << jointName << "Angle: "<< angle
                  << ". Fraction " <<  fraction << std::endl;
    }
    else
      m_motionProxy->changeAngles(jointName,0.0f,0.1f);
  }
}

/*!
  Stop the velocity applied to the joints.
  \param names :  Names the joints, chains, "Body", "JointActuators",
  "Joints" or "Actuators" to stop.
 */
void vpNaoqiRobot::stop(const AL::ALValue &names)
{
  std::vector<std::string> jointNames;
  if (names.isString()) // Suppose to be a chain
    jointNames = m_motionProxy->getBodyNames(names);
  else if (names.isArray()) // Supposed to be a vector of joints
    jointNames = names; // it a vector of joints
  else
    throw vpRobotException (vpRobotException::readingParametersError,
                            "Unable to decode the joint chain.");

  for (unsigned i = 0 ; i < jointNames.size() ; ++i)
    m_motionProxy->changeAngles(jointNames[i], 0.0f, 0.1f);
}

/*!
  Get the name of all the joints of the chain.

  \param names : Names the joints, chains, "Body", "JointActuators",
  "Joints" or "Actuators".

  \return The name of the joints.
 */
std::vector<std::string>
vpNaoqiRobot::getBodyNames(const std::string &names) const
{
  if (! m_isOpen)
    throw vpRobotException (vpRobotException::readingParametersError,
                            "The connexion with the robot was not open");
  std::vector<std::string> jointNames = m_motionProxy->getBodyNames(names);
  return jointNames;
}

/*!
  Get minimal joint values for a joint chain.

  \return A vector that contains the minimal joint values
  of the chain. All the values are expressed in radians.

  \param names : Names the joints, chains, "Body", "JointActuators",
  "Joints" or "Actuators".

  \return The min angle of each joint of the chain.
*/
vpColVector
vpNaoqiRobot::getJointMin(const AL::ALValue& names) const
{
  if (! m_isOpen)
    throw vpRobotException (vpRobotException::readingParametersError,
                            "The connexion with the robot was not open");
  AL::ALValue limits = m_motionProxy->getLimits(names);
  vpColVector min_limits(limits.getSize());
  for (unsigned int i=0; i<limits.getSize(); i++)
  {
    min_limits[i] = limits[i][0];
  }
  return min_limits;
}

/*!
  Get minimal joint values for a joint chain.

  \return A vector that contains the minimal joint values
  of the chain. All the values are expressed in radians.

  \param names : Names the joints, chains, "Body", "JointActuators",
  "Joints" or "Actuators".

  \return The min angle of each joint of the chain.
*/
vpColVector
vpNaoqiRobot::getJointMax(const AL::ALValue& names) const
{
  if (! m_isOpen)
    throw vpRobotException (vpRobotException::readingParametersError,
                            "The connexion with the robot was not open");
  AL::ALValue limits = m_motionProxy->getLimits(names);
  vpColVector max_limits(limits.getSize());
  for (unsigned int i=0; i<limits.getSize(); i++)
  {
    max_limits[i] = limits[i][1];
  }
  return max_limits;
}

vpColVector vpNaoqiRobot::getPosition(const AL::ALValue& names, const bool& useSensors) const
{
  std::vector<float> sensorAngles = m_motionProxy->getAngles(names, useSensors);
  vpColVector q(sensorAngles.size());
  for(unsigned int i=0; i<sensorAngles.size(); i++)
    q[i] = sensorAngles[i];
  return q;
}


void vpNaoqiRobot::setPosition(const AL::ALValue& names, const AL::ALValue& angles, const float& fractionMaxSpeed)
{ 
  m_motionProxy->setAngles(names, angles, fractionMaxSpeed);
}

void vpNaoqiRobot::setPosition(const AL::ALValue& names, const vpColVector &jointPosition, const float& fractionMaxSpeed)
{
  std::vector<float> angles(jointPosition.size());
  for (unsigned int i=0; i<angles.size(); i++)
    angles[i] = jointPosition[i];

  m_motionProxy->setAngles(names, angles, fractionMaxSpeed);
}

vpMatrix vpNaoqiRobot::get_eJe(const std::string &chainName) const
{
  vpMatrix eJe;

  if (chainName == "Head")
  {
    std::vector<float> q = m_motionProxy->getAngles(chainName,true);

    //std::cout << "Joint value:" << q << std::endl;

    const unsigned int nJoints= q.size();

    eJe.resize(6,nJoints);

    double d3 = 0.09511;

    eJe[0][0]= d3*cos(q[4])*sin(q[2]);
    eJe[1][0]= -d3*sin(q[2])*sin(q[4]);
    eJe[2][0]= 0;
    eJe[3][0]= cos(q[2] + q[3])*sin(q[4]);
    eJe[4][0]= cos(q[2] + q[3])*cos(q[4]);
    eJe[5][0]=  -sin(q[2] + q[3]);

    eJe[0][1]= d3*sin(q[3])*sin(q[4]);
    eJe[1][1]= d3*cos(q[4])*sin(q[3]);
    eJe[2][1]= d3*cos(q[3]);
    eJe[3][1]=  cos(q[4]);
    eJe[4][1]= -sin(q[4]);
    eJe[5][1]= 0;

    eJe[0][2]= 0;
    eJe[1][2]= 0;
    eJe[2][2]= 0;
    eJe[3][2]= cos(q[4]);
    eJe[4][2]= -sin(q[4]);
    eJe[5][2]= 0;

    eJe[0][3]= 0;
    eJe[1][3]= 0;
    eJe[2][3]= 0;
    eJe[3][3]= 0;
    eJe[4][3]= 0;
    eJe[5][3]= 1;
  }
  else if (chainName == "RArm")
  {
    throw vpRobotException (vpRobotException::readingParametersError,
                            "Jacobian RArm not avaible yet");
  }
  else if (chainName == "LArm")
  {
    throw vpRobotException (vpRobotException::readingParametersError,
                            "Jacobian LArm not avaible yet");
  }
  else
  {

    throw vpRobotException (vpRobotException::readingParametersError,
                            "End-effector name not recognized. Please choose one above 'Head', 'Larm' or 'Rarm' ");

  }
  return eJe;

}


  vpHomogeneousMatrix vpNaoqiRobot::getTransfEndEffector(const std::string &endEffectorName)
  {
    vpHomogeneousMatrix cMe;

    // Transformation matrix from HeadRoll to CameraLeft
    if (endEffectorName == "CameraLeft")
    {
      cMe[0][0] = -1.;
      cMe[1][0] = 0.;
      cMe[2][0] =  0.;

      cMe[0][1] = 0.;
      cMe[1][1] = -1.;
      cMe[2][1] =  0.;

      cMe[0][2] = 0.;
      cMe[1][2] = 0.;
      cMe[2][2] =  1.;

      cMe[0][3] = 0.04;
      cMe[1][3] = 0.09938;
      cMe[2][3] =  0.11999;
    }

    else
    {
      throw vpRobotException (vpRobotException::readingParametersError,
                              "Transformation matrix that you requested is not implemented. Valid values: CameraLeft.");
    }
    return cMe;
  }
