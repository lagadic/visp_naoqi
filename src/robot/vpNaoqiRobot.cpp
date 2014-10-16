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
 *
 *****************************************************************************/



#include <visp_naoqi/vpNaoqiRobot.h>


vpNaoqiRobot::vpNaoqiRobot()
  : m_motionProxy(NULL), m_robotIp("198.18.0.1"), m_isOpen(false), m_collisionProtection(true)
{

}

vpNaoqiRobot::~vpNaoqiRobot()
{
  cleanup();
}

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

void vpNaoqiRobot::setStiffness(const std::vector<std::string> &jointNames, float stiffness)
{
  m_motionProxy->setStiffnesses(jointNames, AL::ALValue( stiffness ));
}


void vpNaoqiRobot::setStiffness(const std::string &chainName, float stiffness)
{
  // Get the names of all the joints in the group.
  std::vector<std::string> jointNames = m_motionProxy->getBodyNames(chainName);
  if (jointNames.size()>0)
    m_motionProxy->setStiffnesses(chainName, AL::ALValue( stiffness ));
  else  {
    throw vpRobotException (vpRobotException::readingParametersError,
                            "The name of the chain is not valid.");
  }

}

void vpNaoqiRobot::setVelocity(const std::vector<std::string> &jointNames, const std::vector<float> &jointVel)
{
  if (jointNames.size() != jointVel.size() ) {
    throw vpRobotException (vpRobotException::readingParametersError,
                            "The dimensions of the joint array and the velocities array do not match.");
  }

  for (unsigned i = 0 ; i < jointNames.size() ; ++i)
  {
    std::string jointName = jointNames.at(i);
    float vel = jointVel.at(i);

    std::cout << "Desired velocity =  " << vel << "rad/s to the joint " << jointName << "." << std::endl;
    std::cout << "\n";
    //Get the limits of the joint
    AL::ALValue limits = m_motionProxy->getLimits(jointName);

    AL::ALValue angle = AL::ALValue( 0.0f);
    unsigned int applymotion = 0;

    if (vel > 0.0f)
    {
      //Reach qMax
      angle = limits[0][1];
      applymotion = 1;
      std::cout << "Reach qMax (" << angle << ") ";
    }

    else if (vel < 0.0f)
    {
      //Reach qMin
      angle = limits[0][0];
      applymotion = 1;
      std::cout << "Reach qMin (" << angle << ") ";
    }

    if (applymotion)
    {
      float fraction = fabs( float (vel/float(limits[0][2])));
      if (fraction >= 1.0 )
      {
        std::cout << "Given velocity is too high: " <<  vel << "rad/s for " << jointName << "." << std::endl;
        std::cout << "Max allowed is: " << limits[0][2] << "rad/s for "<< std::endl;
        fraction = 1.0;
      }
      m_motionProxy->setAngles(jointName,angle,fraction);
      std::cout << "SET VELOCITY TO " <<  vel << "rad/s for " << jointName << "Angle: "<< angle << ". Fraction "
                <<  fraction << std::endl;


    }
    else
      m_motionProxy->changeAngles(jointName,0.0f,0.1f);
  }

}



void vpNaoqiRobot::setVelocity(const std::string &chainName, const std::vector<float> &jointVel)
{

  // Get the names of all the joints in the group.
  std::vector<std::string> jointNames = m_motionProxy->getBodyNames(chainName);
  if (jointNames.size()>0)
    vpNaoqiRobot::setVelocity(jointNames, jointVel);
  else  {
    throw vpRobotException (vpRobotException::readingParametersError,
                            "The name of the chain is not valid.");
  }

}

void vpNaoqiRobot::stop(const std::vector<std::string> &jointNames)
{
  //std::vector<float> jointVel(jointNames.size(), 0);
  for (unsigned i = 0 ; i < jointNames.size() ; ++i)
    m_motionProxy->changeAngles(jointNames[i],0.0f,0.1f);
}

void vpNaoqiRobot::stop(const std::string &chainName)
{
  // Get the names of all the joints in the group.
  std::vector<std::string> jointNames = m_motionProxy->getBodyNames(chainName);
  if ( jointNames.size() )
    vpNaoqiRobot::stop(jointNames);
  else {
    throw vpRobotException (vpRobotException::readingParametersError,
                            "The name of the chain is not valid.");
  }
}

/*!
  Get the name of all the joints of the chain.

  \param chainName : Name of the chain. Allowed values are "Head",
  "LArm" for left arm and "RArm" for right arm.

  \return The name of the joints.
 */
std::vector<std::string> vpNaoqiRobot::getJointNames(const std::string &chainName)
{
  if (! m_isOpen)
    throw vpRobotException (vpRobotException::readingParametersError,
                            "The connexion with the robot was not open");
  std::vector<std::string> jointNames = m_motionProxy->getBodyNames(chainName);
  return jointNames;
}

std::vector<float> vpNaoqiRobot::getAngles(const AL::ALValue& names, const bool& useSensors)
{
  std::vector<float> sensorAngles = m_motionProxy->getAngles(names, useSensors);
  return sensorAngles;
}


void vpNaoqiRobot::setAngles(const AL::ALValue& names, const AL::ALValue& angles, const float& fractionMaxSpeed)
{
  m_motionProxy->setAngles(names, angles, fractionMaxSpeed);
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


vpMatrix vpNaoqiRobot::getJacobian(const std::string &lastJointName)
{

  vpMatrix eJe;

  if (lastJointName == "Head")
  {

    std::vector<float> q = m_motionProxy->getAngles(lastJointName,true);

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
  else if (lastJointName == "RArm")
  {
    throw vpRobotException (vpRobotException::readingParametersError,
                            "Jacobian RArm not avaible yet");
  }

  else if (lastJointName == "LArm")
  {

    std::vector<float> q = m_motionProxy->getAngles(lastJointName,true);

    std::cout << "Joint value:" << q << std::endl;

    const unsigned int nJoints = q.size()-1; // -1 because we don't consider the last joint LHand

    float q10, q11, q12, q13, q14, q15;

    q10 = q[1]; //LShoulderYaw
    q11 = q[2]; //LElbowRoll
    q12 = q[3]; //LElbowYaw
    q13 = q[4]; //LWristRoll
    q14 = q[5]; //LWristYaw
    q15 = q[6]; //LWristPitch


    float g9 = 2.7053 ;
    float b9 = 0.0279;
    float alpha9 = 1.729254202933720;
    float d9 =  0.0758;
    float phi9 = 3.068152048225621;
    float phi10 = 0.430457;
    float phi11 = 0.17452;
    float r9 = 0.1765;
    float r11 =  0.205;
    float r13 =  0.1823;

    eJe.resize(6,nJoints);

    eJe[0][0]=r13*sin(phi11 + q11)*sin(phi10)*sin(q10)*sin(q13)*sin(q15) - r13*cos(phi10)*cos(q13)*sin(q10)*sin(q12)*sin(q15) - r13*cos(q10)*cos(q13)*sin(phi10)*sin(q12)*sin(q15) + r11*cos(phi11 + q11)*cos(phi10)*cos(q10)*cos(q13)*sin(q15) - r11*cos(phi11 + q11)*cos(q13)*sin(phi10)*sin(q10)*sin(q15) - r13*sin(phi11 + q11)*cos(phi10)*cos(q10)*sin(q13)*sin(q15) + r13*cos(phi10)*cos(q15)*sin(q10)*sin(q12)*sin(q13)*sin(q14) + r13*cos(q10)*cos(q15)*sin(phi10)*sin(q12)*sin(q13)*sin(q14) + r13*cos(phi11 + q11)*cos(phi10)*cos(q10)*cos(q12)*cos(q13)*sin(q15) - r11*sin(phi11 + q11)*cos(phi10)*cos(q10)*cos(q14)*cos(q15)*sin(q12) - r11*cos(phi11 + q11)*cos(phi10)*cos(q10)*cos(q15)*sin(q13)*sin(q14) - r13*sin(phi11 + q11)*cos(phi10)*cos(q10)*cos(q13)*cos(q15)*sin(q14) - r11*sin(phi11 + q11)*cos(phi10)*cos(q10)*cos(q12)*sin(q13)*sin(q15) - r13*cos(phi11 + q11)*cos(q12)*cos(q13)*sin(phi10)*sin(q10)*sin(q15) + r11*sin(phi11 + q11)*cos(q14)*cos(q15)*sin(phi10)*sin(q10)*sin(q12) + r11*cos(phi11 + q11)*cos(q15)*sin(phi10)*sin(q10)*sin(q13)*sin(q14) + r13*sin(phi11 + q11)*cos(q13)*cos(q15)*sin(phi10)*sin(q10)*sin(q14) + r11*sin(phi11 + q11)*cos(q12)*sin(phi10)*sin(q10)*sin(q13)*sin(q15) - r11*sin(phi11 + q11)*cos(phi10)*cos(q10)*cos(q12)*cos(q13)*cos(q15)*sin(q14) - r13*cos(phi11 + q11)*cos(phi10)*cos(q10)*cos(q12)*cos(q15)*sin(q13)*sin(q14) + r11*sin(phi11 + q11)*cos(q12)*cos(q13)*cos(q15)*sin(phi10)*sin(q10)*sin(q14) + r13*cos(phi11 + q11)*cos(q12)*cos(q15)*sin(phi10)*sin(q10)*sin(q13)*sin(q14);
    eJe[0][1]=r11*sin(phi11 + q11)*cos(q13)*sin(q15) + r13*cos(phi11 + q11)*sin(q13)*sin(q15) + r11*cos(phi11 + q11)*cos(q14)*cos(q15)*sin(q12) + r13*cos(phi11 + q11)*cos(q13)*cos(q15)*sin(q14) + r11*cos(phi11 + q11)*cos(q12)*sin(q13)*sin(q15) + r13*sin(phi11 + q11)*cos(q12)*cos(q13)*sin(q15) - r11*sin(phi11 + q11)*cos(q15)*sin(q13)*sin(q14) + r11*cos(phi11 + q11)*cos(q12)*cos(q13)*cos(q15)*sin(q14) - r13*sin(phi11 + q11)*cos(q12)*cos(q15)*sin(q13)*sin(q14);
    eJe[0][2]=-r13*sin(q12)*(cos(q13)*sin(q15) - cos(q15)*sin(q13)*sin(q14));
    eJe[0][3]=r13*sin(q13)*sin(q15) + r13*cos(q13)*cos(q15)*sin(q14);
    eJe[0][4]=0;
    eJe[0][5]=0;
    eJe[0][6]=0;
    eJe[1][0]=r13*sin(phi11 + q11)*cos(q15)*sin(phi10)*sin(q10)*sin(q13) - r13*cos(phi10)*cos(q13)*cos(q15)*sin(q10)*sin(q12) - r13*cos(q10)*cos(q13)*cos(q15)*sin(phi10)*sin(q12) + r11*cos(phi11 + q11)*cos(phi10)*cos(q10)*cos(q13)*cos(q15) - r11*cos(phi11 + q11)*cos(q13)*cos(q15)*sin(phi10)*sin(q10) - r13*sin(phi11 + q11)*cos(phi10)*cos(q10)*cos(q15)*sin(q13) - r13*cos(phi10)*sin(q10)*sin(q12)*sin(q13)*sin(q14)*sin(q15) - r13*cos(q10)*sin(phi10)*sin(q12)*sin(q13)*sin(q14)*sin(q15) + r13*cos(phi11 + q11)*cos(phi10)*cos(q10)*cos(q12)*cos(q13)*cos(q15) - r11*sin(phi11 + q11)*cos(phi10)*cos(q10)*cos(q12)*cos(q15)*sin(q13) - r13*cos(phi11 + q11)*cos(q12)*cos(q13)*cos(q15)*sin(phi10)*sin(q10) + r11*sin(phi11 + q11)*cos(phi10)*cos(q10)*cos(q14)*sin(q12)*sin(q15) + r11*cos(phi11 + q11)*cos(phi10)*cos(q10)*sin(q13)*sin(q14)*sin(q15) + r13*sin(phi11 + q11)*cos(phi10)*cos(q10)*cos(q13)*sin(q14)*sin(q15) + r11*sin(phi11 + q11)*cos(q12)*cos(q15)*sin(phi10)*sin(q10)*sin(q13) - r11*sin(phi11 + q11)*cos(q14)*sin(phi10)*sin(q10)*sin(q12)*sin(q15) - r11*cos(phi11 + q11)*sin(phi10)*sin(q10)*sin(q13)*sin(q14)*sin(q15) - r13*sin(phi11 + q11)*cos(q13)*sin(phi10)*sin(q10)*sin(q14)*sin(q15) + r11*sin(phi11 + q11)*cos(phi10)*cos(q10)*cos(q12)*cos(q13)*sin(q14)*sin(q15) + r13*cos(phi11 + q11)*cos(phi10)*cos(q10)*cos(q12)*sin(q13)*sin(q14)*sin(q15) - r11*sin(phi11 + q11)*cos(q12)*cos(q13)*sin(phi10)*sin(q10)*sin(q14)*sin(q15) - r13*cos(phi11 + q11)*cos(q12)*sin(phi10)*sin(q10)*sin(q13)*sin(q14)*sin(q15);
    eJe[1][1]=r11*sin(phi11 + q11)*cos(q13)*cos(q15) + r13*cos(phi11 + q11)*cos(q15)*sin(q13) + r11*cos(phi11 + q11)*cos(q12)*cos(q15)*sin(q13) + r13*sin(phi11 + q11)*cos(q12)*cos(q13)*cos(q15) - r11*cos(phi11 + q11)*cos(q14)*sin(q12)*sin(q15) - r13*cos(phi11 + q11)*cos(q13)*sin(q14)*sin(q15) + r11*sin(phi11 + q11)*sin(q13)*sin(q14)*sin(q15) + r13*sin(phi11 + q11)*cos(q12)*sin(q13)*sin(q14)*sin(q15) - r11*cos(phi11 + q11)*cos(q12)*cos(q13)*sin(q14)*sin(q15);
    eJe[1][2]=-r13*sin(q12)*(cos(q13)*cos(q15) + sin(q13)*sin(q14)*sin(q15));
    eJe[1][3]=r13*cos(q15)*sin(q13) - r13*cos(q13)*sin(q14)*sin(q15);
    eJe[1][4]=0;
    eJe[1][5]=0;
    eJe[1][6]=0;
    eJe[2][0]=r13*cos(phi10)*cos(q14)*sin(q10)*sin(q12)*sin(q13) - r11*sin(phi11 + q11)*sin(phi10)*sin(q10)*sin(q12)*sin(q14) + r13*cos(q10)*cos(q14)*sin(phi10)*sin(q12)*sin(q13) - r11*cos(phi11 + q11)*cos(phi10)*cos(q10)*cos(q14)*sin(q13) - r13*sin(phi11 + q11)*cos(phi10)*cos(q10)*cos(q13)*cos(q14) + r11*sin(phi11 + q11)*cos(phi10)*cos(q10)*sin(q12)*sin(q14) + r11*cos(phi11 + q11)*cos(q14)*sin(phi10)*sin(q10)*sin(q13) + r13*sin(phi11 + q11)*cos(q13)*cos(q14)*sin(phi10)*sin(q10) - r11*sin(phi11 + q11)*cos(phi10)*cos(q10)*cos(q12)*cos(q13)*cos(q14) - r13*cos(phi11 + q11)*cos(phi10)*cos(q10)*cos(q12)*cos(q14)*sin(q13) + r11*sin(phi11 + q11)*cos(q12)*cos(q13)*cos(q14)*sin(phi10)*sin(q10) + r13*cos(phi11 + q11)*cos(q12)*cos(q14)*sin(phi10)*sin(q10)*sin(q13);
    eJe[2][1]=r13*cos(phi11 + q11)*cos(q13)*cos(q14) - r11*cos(phi11 + q11)*sin(q12)*sin(q14) - r11*sin(phi11 + q11)*cos(q14)*sin(q13) - r13*sin(phi11 + q11)*cos(q12)*cos(q14)*sin(q13) + r11*cos(phi11 + q11)*cos(q12)*cos(q13)*cos(q14);
    eJe[2][2]=r13*cos(q14)*sin(q12)*sin(q13);
    eJe[2][3]=r13*cos(q13)*cos(q14);
    eJe[2][4]=0;
    eJe[2][5]=0;
    eJe[2][6]=0;
    eJe[3][0]=cos(phi10 + q10)*sin(phi11 + q11)*cos(q13)*sin(q15) + sin(phi10 + q10)*cos(q12)*cos(q14)*cos(q15) - sin(phi10 + q10)*sin(q12)*sin(q13)*sin(q15) + cos(phi10 + q10)*cos(phi11 + q11)*cos(q14)*cos(q15)*sin(q12) + cos(phi10 + q10)*cos(phi11 + q11)*cos(q12)*sin(q13)*sin(q15) - cos(phi10 + q10)*sin(phi11 + q11)*cos(q15)*sin(q13)*sin(q14) - sin(phi10 + q10)*cos(q13)*cos(q15)*sin(q12)*sin(q14) + cos(phi10 + q10)*cos(phi11 + q11)*cos(q12)*cos(q13)*cos(q15)*sin(q14);
    eJe[3][1]=sin(phi11 + q11)*cos(q14)*cos(q15)*sin(q12) - cos(phi11 + q11)*cos(q13)*sin(q15) + cos(phi11 + q11)*cos(q15)*sin(q13)*sin(q14) + sin(phi11 + q11)*cos(q12)*sin(q13)*sin(q15) + sin(phi11 + q11)*cos(q12)*cos(q13)*cos(q15)*sin(q14);
    eJe[3][2]=cos(q12)*cos(q14)*cos(q15) - sin(q12)*sin(q13)*sin(q15) - cos(q13)*cos(q15)*sin(q12)*sin(q14);
    eJe[3][3]=cos(q15)*sin(q13)*sin(q14) - cos(q13)*sin(q15);
    eJe[3][4]=cos(q14)*cos(q15);
    eJe[3][5]=-sin(q15);
    eJe[3][6]=0;
    eJe[4][0]=cos(phi10 + q10)*sin(phi11 + q11)*cos(q13)*cos(q15) - sin(phi10 + q10)*cos(q12)*cos(q14)*sin(q15) - sin(phi10 + q10)*cos(q15)*sin(q12)*sin(q13) + cos(phi10 + q10)*cos(phi11 + q11)*cos(q12)*cos(q15)*sin(q13) - cos(phi10 + q10)*cos(phi11 + q11)*cos(q14)*sin(q12)*sin(q15) + cos(phi10 + q10)*sin(phi11 + q11)*sin(q13)*sin(q14)*sin(q15) + sin(phi10 + q10)*cos(q13)*sin(q12)*sin(q14)*sin(q15) - cos(phi10 + q10)*cos(phi11 + q11)*cos(q12)*cos(q13)*sin(q14)*sin(q15);
    eJe[4][1]=sin(phi11 + q11)*cos(q12)*cos(q15)*sin(q13) - cos(phi11 + q11)*cos(q13)*cos(q15) - sin(phi11 + q11)*cos(q14)*sin(q12)*sin(q15) - cos(phi11 + q11)*sin(q13)*sin(q14)*sin(q15) - sin(phi11 + q11)*cos(q12)*cos(q13)*sin(q14)*sin(q15);
    eJe[4][2]=cos(q13)*sin(q12)*sin(q14)*sin(q15) - cos(q15)*sin(q12)*sin(q13) - cos(q12)*cos(q14)*sin(q15);
    eJe[4][3]=- cos(q13)*cos(q15) - sin(q13)*sin(q14)*sin(q15);
    eJe[4][4]=-cos(q14)*sin(q15);
    eJe[4][5]=-cos(q15);
    eJe[4][6]=0;
    eJe[5][0]=cos(phi10 + q10)*cos(phi11 + q11)*cos(q12)*cos(q13)*cos(q14) - cos(phi10 + q10)*cos(phi11 + q11)*sin(q12)*sin(q14) - cos(phi10 + q10)*sin(phi11 + q11)*cos(q14)*sin(q13) - sin(phi10 + q10)*cos(q13)*cos(q14)*sin(q12) - sin(phi10 + q10)*cos(q12)*sin(q14);
    eJe[5][1]=cos(phi11 + q11)*cos(q14)*sin(q13) - sin(phi11 + q11)*sin(q12)*sin(q14) + sin(phi11 + q11)*cos(q12)*cos(q13)*cos(q14);
    eJe[5][2]=- cos(q12)*sin(q14) - cos(q13)*cos(q14)*sin(q12);
    eJe[5][3]=cos(q14)*sin(q13);
    eJe[5][4]=-sin(q14);
    eJe[5][5]=0;
    eJe[5][6]=1;



  }
  else
  {

    throw vpRobotException (vpRobotException::readingParametersError,
                            "End-effector name not recognized. Please choose one above 'Head', 'Larm' or 'Rarm' ");

  }
return eJe;

}
