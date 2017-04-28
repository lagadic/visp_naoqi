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
// VispNaoqi
#include <visp_naoqi/vpNaoqiRobot.h>
//#include <visp_naoqi/vpNaoqiConfig.h>
#include "al/from_any_value.hpp"
// Visp
#include <visp/vpVelocityTwistMatrix.h>


// Aldebaran SDK

#ifdef VISP_NAOQI_HAVE_MATAPOD
#  include <romeo.hh>
#  include <pepper.hh>
#  include <metapod/tools/print.hh>
#  include <metapod/tools/initconf.hh>
#  include <metapod/algos/jac_point_chain.hh>
#  include <metapod/tools/jcalc.hh>
#  include <metapod/algos/djac.hh>
#  include <metapod/algos/rnea.hh>

using namespace metapod;

typedef double LocalFloatType;
typedef romeo<LocalFloatType> RomeoModel;
typedef pepper<LocalFloatType> PepperModel;
#endif

/*!
  Default constructor that set the default parameters as:
  - robot ip address: "198.18.0.1"
  - collision protection: enabled
  */
vpNaoqiRobot::vpNaoqiRobot(const qi::SessionPtr &session)
  : m_pMemory(session->service("ALMemory")), m_pMotion(session->service("ALMotion")), m_pepper_control(session->service("pepper_control")), m_isOpen(false), m_collisionProtection(true),
    m_robotName(""), m_robotType(Unknown)
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

    // Get the robot type
    m_robotName = m_pMemory.call<std::string>("getData", "RobotConfig/Body/Type");
    std::transform(m_robotName.begin(), m_robotName.end(), m_robotName.begin(), ::tolower);

    if (m_robotName == "nao")
      m_robotType = Nao;
    else if (m_robotName == "romeo")
      m_robotType = Romeo;
    else if (m_robotName == "pepper" || m_robotName == "juliette")
    {
      m_robotName = "pepper";
      m_robotType = Pepper;
    }
    else
    {
      std::cout << "Unknown robot" << std::endl;
    }

    std::cout << "Connected to a " << m_robotName << " robot." << std::endl;

    // TODO: Understand how to use setMotionConfig with AnyValueVector
    //        if ( m_robotType != Pepper)
    //        {
    //            bool success = m_pVideo.call<bool>("setCollisionProtectionEnabled", "Arms", false);
    //            if (success)
    //                std::cout << "Collision protection is disabled" << std::endl;

    //            qi::AnyValueVector config;
    //            AL::ALValue one = AL::ALValue::array(std::string("CONTROL_JOINT_MAX_ACCELERATION"),AL::ALValue(5.0));
    //            config.push_back(one);

    //            // Set Trapezoidal interpolation
    //            qi::AnyValueVector config;

    //            AL::ALValue one = AL::ALValue::array(std::string("CONTROL_USE_ACCELERATION_INTERPOLATOR"),AL::ALValue(1));
    //            AL::ALValue two = AL::ALValue::array(std::string("CONTROL_JOINT_MAX_ACCELERATION"),AL::ALValue(5.0));

    //            config.arrayPush(one);
    //            config.arrayPush(two);

    //            m_motionProxy->setMotionConfig(config);

    //            std::cout << "Trapezoidal interpolation is on " << std::endl;
    //        }
    //        //On nao, we have joint coupled limits (http://doc.aldebaran.com/2-1/family/robots/joints_robot.html) on the head and ankle.
    //        //Motion and DCM have clamping. We have to remove motion clamping.
    //        if (m_robotName.find("nao") != std::string::npos)
    //        {
    //            AL::ALValue config_;
    //            AL::ALValue setting = AL::ALValue::array(std::string("ENABLE_DCM_LIKE_CLAMPING"),AL::ALValue(0));
    //            config_.arrayPush(setting);
    //        }

    m_isOpen = true;
  }
}

void vpNaoqiRobot::cleanup()
{

  if (m_robotType == Pepper)
  {
    m_pepper_control.call<void >("stopJoint");
    m_pepper_control.call<void >("stop");
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
void vpNaoqiRobot::setStiffness(const std::string &names, const float &stiffness) const
{
  m_pMotion.call<void>("setStiffnesses", names, stiffness );
}



/*!
  Apply a velocity vector to a vector of joints.
  \param names :  Names the joints, chains, "Body", "JointActuators",
  "Joints" or "Actuators".
  \param jointVel : Joint velocity vector with values expressed in rad/s.
  \param verbose : If true activates printings.
 */

//void vpNaoqiRobot::setVelocity(const AL::ALValue& names, const std::vector<float> &jointVel, bool verbose)
//{
//  setVelocity(names, (AL::ALValue)(jointVel), verbose);
//}


void vpNaoqiRobot::setVelocity(const std::vector<std::string> &names, const std::vector<float> &jointVel) const
{
  if (m_robotType == Pepper)
    m_pepper_control.async<void >("setDesJointVelocity", names, jointVel);
  else
  {
    std::cout << " The velocity controller for Romeo and Nao is not yet implemented" << std::endl;
    exit(0);
  }


}


///*!
//  Apply a velocity vector to a vector of joints.Use just one call to apply the velocities.
//  \param names :  Names the joints, chains, "Body", "JointActuators",
//  "Joints" or "Actuators".
//  \param jointVel : Joint velocity vector with values expressed in rad/s.
//  \param verbose : If true activates printings.
// */

//void vpNaoqiRobot::setVelocity(const AL::ALValue& names, const vpColVector &jointVel, bool verbose)
//{
//  std::vector<float> jointVel_(jointVel.size());
//  for (unsigned int i=0; i< jointVel.size(); i++)
//    jointVel_[i] = jointVel[i];
//  setVelocity(names, jointVel_, verbose);
//}

/*!
  Apply a velocity vector to a vector of joints. Use just one call to apply the velocities.

  \param names :  Names the joints, chains, "Body", "JointActuators",
  "Joints" or "Actuators".
  \param jointVel : Joint velocity vector with values expressed in rad/s.
  \param verbose : If true activates printings.
 */
//void vpNaoqiRobot::setVelocity(const AL::ALValue &names, const AL::ALValue &jointVel, bool verbose)
//{
//  std::vector<std::string> jointNames;
//  if (names.isString()) // Suppose to be a chain
//    jointNames = m_motionProxy->getBodyNames(names);
//  else if (names.isArray()) // Supposed to be a vector of joints
//    jointNames = names; // it a vector of joints
//  else
//    throw vpRobotException (vpRobotException::readingParametersError,
//                            "Unable to decode the joint chain.");
//
//  if (jointNames.size() != jointVel.getSize() ) {
//    throw vpRobotException (vpRobotException::readingParametersError,
//                            "The dimensions of the joint array and the velocities array do not match.");
//  }
//
//  if (m_robotType == Romeo || m_robotType == Nao)
//  {
//    AL::ALValue jointListStop;
//    AL::ALValue jointListMove;
//    AL::ALValue angles;
//    std::vector<float> fractions;
//
//    for (unsigned i = 0 ; i < jointVel.getSize() ; ++i)
//    {
//      std::string jointName = jointNames[i];
//      if (verbose)
//        std::cout << "Joint name: " << jointName << std::endl;
//
//      float vel = jointVel[i];
//
//      if (verbose)
//        std::cout << "Desired velocity =  " << vel << "rad/s to the joint " << jointName << "." << std::endl;
//
//      //Get the limits of the joint
//      std::vector<std::vector<float>> limits = getLimits(jointName);
//
//      if (vel == 0.0f)
//      {
//        if (verbose)
//          std::cout << "Stop the joint" << std::endl ;
//
//        jointListStop.arrayPush(jointName);
//      }
//      else
//      {
//
//        if (vel > 0.0f)
//        {
//          //Reach qMax
//          angles.arrayPush(limits[0][1]);
//          if (verbose)
//            std::cout << "Reach qMax (" << limits[0][1] << ") ";
//        }
//
//        else if (vel < 0.0f)
//        {
//          //Reach qMin
//          angles.arrayPush(limits[0][0]);
//          if (verbose)
//            std::cout << "Reach qMin (" << limits[0][0] << ") ";
//        }
//
//
//        jointListMove.arrayPush(jointName);
//        float fraction = fabs( float (vel/float(limits[0][2])));
//        if (fraction >= 1.0 )
//        {
//          if (verbose) {
//            std::cout << "Given velocity is too high: " <<  vel << "rad/s for " << jointName << "." << std::endl;
//            std::cout << "Max allowed is: " << limits[0][2] << "rad/s for "<< std::endl;
//          }
//          fraction = 1.0;
//        }
//
//        fractions.push_back(fraction);
//
//      }
//    }
//    if (verbose) {
//      std::cout << "Apply Velocity to joints " << jointListMove << std::endl;
//      std::cout << "Stop List joints: " << jointListStop << std::endl;
//      std::cout << "with fractions " << angles << std::endl;
//      std::cout << "to angles " << fractions << std::endl;
//    }
//
//    if (jointListMove.getSize()>0)
//    {
//      m_proxy->callVoid("setAngles", jointListMove, angles, fractions);
//    }
//
//    if (jointListStop.getSize()>0)
//    {
//      std::vector<float> zeros( jointListStop.getSize() );
//      m_proxy->callVoid("changeAngles", jointListStop, zeros, 0.1f);
//
//    }
//  } // End Romeo
//  else if (m_robotType == Pepper)
//  {
//    std::vector<float> vel(jointNames.size());
//    jointVel.ToFloatArray(vel);
//    m_pepper_control.async<void >("setDesJointVelocity", jointNames, vel);
//  }
//
//}

/*!
  Stop joints.
  \param names : Vector with the joint names.
 */
void vpNaoqiRobot::stop(const std::vector<std::string> &names) const
{
  if (m_robotType == Romeo || m_robotType == Nao)
  {
   // for (unsigned i = 0 ; i < names.size() ; ++i)
    std::vector<float> zeros(names.size(), 0.0);
    m_pMotion.async<void>("changeAngles", names, zeros, 0.1f);
  }
  else if (m_robotType == Pepper) //TODO: Add function to Pepper_control to stop only desired joints
    m_pepper_control.call<void >("stopJoint");
}



/*!
  Stop joint in a chain.
  \param names : Chain or joint name.
 */
void vpNaoqiRobot::stop(const std::string &name) const
{
  if (m_robotType == Romeo || m_robotType == Nao)
  {
    std::vector<std::string> names = getBodyNames(name);
    // for (unsigned i = 0 ; i < names.size() ; ++i)
    std::vector<float> zeros(names.size(), 0.0);
    m_pMotion.async<void>("changeAngles", names, zeros, 0.1f);
  }
  else if (m_robotType == Pepper) //TODO: Add function to Pepper_control to stop only desired joints
    m_pepper_control.call<void >("stopJoint");
}

/*!
  Stop the service pepper_control.
 */
void vpNaoqiRobot::stopPepperControl() const
{
  m_pepper_control.call<void >("stop");
}

/*!
  Start the service pepper_control.
 */
void vpNaoqiRobot::startPepperControl() const
{
  m_pepper_control.call<void >("start");
}


/*!
  Stop the velocity of the base.
 */
void vpNaoqiRobot::stopBase() const
{
  m_pMotion.async<void>("move", 0.0, 0.0, 0.0);
}


/*!
  Apply a velocity Vx, Vy, Wz to Pepper.
  \param vel : Joint velocity vector with values expressed in rad/s.
 */

void vpNaoqiRobot::setBaseVelocity(const vpColVector &jointVel) const
{
  std::vector<float> jointVel_(jointVel.size());
  for (unsigned int i=0; i< jointVel.size(); i++)
    jointVel_[i] = jointVel[i];
  setBaseVelocity(jointVel_);
}

/*!
  Apply a velocity Vx, Vy, Wz to Pepper.
  \param vel : Joint velocity vector with values expressed in rad/s.
 */

void vpNaoqiRobot::setBaseVelocity(const std::vector<float> &jointVel) const
{
  if (m_robotType == Pepper)
  {
    if (jointVel.size() == 3)
      m_pMotion.async<void>("move", jointVel[0], jointVel[1], jointVel[2]);
    else
      std::cerr << "ERROR: Cannot apply velocity to the base. Check the size of the vector, it has to be of size 3."
                << std::endl
                << "Current size: " << jointVel.size() << std::endl;
  }
}


/*!
  Apply a velocity Vx, Vy, Wz to Pepper.
  \param vel : Joint velocity vector with values expressed in rad/s.
 */

void vpNaoqiRobot::setBaseVelocity(const float &vx,const float &vy,const float &wz) const
{
  if (m_robotType == Pepper)
      m_pMotion.async<void>("move", vx, vy, wz);
}


/*!
  Get the value of all the joints of the chain.
  \param names :  Names the joints, chains, “Body”, “JointActuators”, “Joints” or “Actuators”.
  \return The value of the joints.
 */
std::vector<float> vpNaoqiRobot::getAngles(const std::string &name, const bool& useSensors) const
{
  return m_pMotion.call<std::vector<float> >("getAngles", name, useSensors );
}


/*!
  Get the value of all the joints in the vector.
  \param names :  Vector containing the names of the joints.
  \return The value of the joints.
 */
std::vector<float> vpNaoqiRobot::getAngles(const std::vector<std::string> &name, const bool& useSensors) const
{
  return m_pMotion.call<std::vector<float> >("getAngles", name, useSensors );
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
  std::vector<std::string> jointNames = m_pMotion.call<std::vector<std::string> >("getBodyNames", names);
  return jointNames;
}

/*!
  Get max joint values for a joint chain.

  \return A vector that contains the max joint limit values
  of the chain. All the values are expressed in radians.

  \param names : Names of the chains.

  \return The max angle of each joint of the chain.
*/
vpColVector
vpNaoqiRobot::getJointMax(const std::string &name) const
{
  if (! m_isOpen)
    throw vpRobotException (vpRobotException::readingParametersError,
                            "The connexion with the robot was not open");
  //Get the limits of the joint
  std::vector<std::vector<float>> limits = getLimits(name);
  vpColVector max_limits(limits.size());
  for (unsigned int i=0; i<limits.size(); i++)
  {
    max_limits[i] = limits[i][1];
  }
  return max_limits;
}

/*!
  Get min joint values for a joint chain.

  \return A vector that contains the minimal joint values
  of the chain. All the values are expressed in radians.

  \param names : Name of the chain.

  \return The min angle of each joint of the chain.
*/
vpColVector
vpNaoqiRobot::getJointMin(const std::string &name) const
{
  if (! m_isOpen)
    throw vpRobotException (vpRobotException::readingParametersError,
                            "The connexion with the robot was not open");
  //Get the limits of the joint
  std::vector<std::vector<float>> limits = getLimits(name);
  vpColVector min_limits(limits.size());
  for (unsigned int i=0; i<limits.size(); i++)
  {
    min_limits[i] = limits[i][0];
  }
  return min_limits;
}


/*!
  Get min joint values for vector of joints.

  \param names : Vector of joint names.
  \param min : Vector of the lower limits of the joints
  \param max : Vector of the upper limits of the joints

*/
void
vpNaoqiRobot::getJointMinAndMax(const std::vector<std::string> &names, vpColVector &min, vpColVector &max ) const
{
  if (! m_isOpen)
    throw vpRobotException (vpRobotException::readingParametersError,
                            "The connexion with the robot was not open");

  for (unsigned int i=0; i<names.size(); i++)
  {
    //Get the limits of the joint
    std::vector<std::vector<float>> limits = getLimits(names[i]);
    //std::cout << limits << std::endl;
    min[i] = limits[0][0];
    max[i] = limits[0][1];
  }
  return;
}


/*!
  Get the position of all the joints of the chain.

  \param names :  Names the joints, chains, "Body", "JointActuators",
  "Joints" or "Actuators".
  \param useSensors :  If true, sensor positions will be returned. If
  false, it will be the command.

  \return Joint position in radians.
 */

vpColVector vpNaoqiRobot::getPosition(const std::string &name, const bool &useSensors) const
{
  std::vector<float> sensorAngles = getAngles(name, useSensors);
  vpColVector q(sensorAngles.size());
  for(unsigned int i=0; i<sensorAngles.size(); i++)
    q[i] = sensorAngles[i];
  return q;
}

/*!
  Get the position of all the joints in the vector.

  \param names :  Names the joints.
  \param useSensors :  If true, sensor positions will be returned. If
  false, it will be the command.
  \param q : Joint position in radians.
 */

void vpNaoqiRobot::getPosition(const std::vector<std::string> &names, std::vector<float>& q, const bool& useSensors) const
{
  q = getAngles(names, useSensors);
  return;
}


/*!
  Get the minAngle (rad), maxAngle (rad), maxVelocity (rad.s-1) and maxTorque (N.m). for a given joint or actuator in the body.
  \param name :  Name of a joint, chain, “Body”, “JointActuators”, “Joints” or “Actuators”.
  \return Vector containing the minAngle, maxAngle, maxVelocity and maxTorque for all the joints specified.
 */
std::vector<std::vector<float>> vpNaoqiRobot::getLimits(const std::string & name) const
{
  std::vector<std::vector<float>> values;

  qi::AnyValue limits = m_pMotion.call<qi::AnyValue>("getLimits", name);
  naoqi::tools::fromAnyValueToFloatVectorOfVector(limits,values);

  return values;

}


/*!
  Set the position of all the joints of the chain.

  \param names :  Names the chain.
  \param angles :  Joint positions in radians.
  \param fractionMaxSpeed : The fraction of maximum speed to use. Value should be comprised between 0 and 1.

 */
void vpNaoqiRobot::setPosition(const std::string &name, const std::vector<float> &angles, const float& fractionMaxSpeed) const
{ 
  m_pMotion.async<void>("setAngles", name, angles, fractionMaxSpeed);
}

/*!
  Set joint positions.

  \param names :  vpColVector with the names the joints.
  \param jointPosition :  Joint positions in radians.
  \param fractionMaxSpeed : The fraction of maximum speed to use. Value should be comprised between 0 and 1.

 */
void vpNaoqiRobot::setPosition(const std::vector<std::string> &names, const vpColVector &jointPosition, const float &fractionMaxSpeed) const
{
  std::vector<float> angles(jointPosition.size());
  for (unsigned int i=0; i<angles.size(); i++)
    angles[i] = jointPosition[i];

  m_pMotion.async<void>("setAngles", names, angles, fractionMaxSpeed);
}

/*!
   Set joint positions.

  \param names :  std::vector with the names the joints.
  \param jointPosition :  Joint positions in radians.
  \param fractionMaxSpeed : The fraction of maximum speed to use. Value should be comprised between 0 and 1.

 */
void vpNaoqiRobot::setPosition(const std::vector<std::string> &names, const std::vector<float> &jointPosition, const float &fractionMaxSpeed) const
{
  m_pMotion.async<void>("setAngles", names, jointPosition, fractionMaxSpeed);
}

/*!
  Get the Jacobian specifying an end effector chain name.

  \param chainName : Name of the end effector. Allowed values are "Head",
  "LArm" for left arm and "RArm" for right arm.

  \return The actual jacobian with respect to the end effector.
 */
vpMatrix vpNaoqiRobot::get_eJe(const std::string &chainName) const
{
  vpMatrix tJe;
  return get_eJe(chainName,tJe);
}


/*!
  Get the Jacobian specifying an end effector chain name.

  \param chainName : Name of the end effector. Allowed values are "Head",
  "LArm" for left arm and "RArm" for right arm.

  \param tJe : Jacobian with respect to the torso

  \return The actual jacobian with respect to the end effector.
 */
vpMatrix vpNaoqiRobot::get_eJe(const std::string &chainName, vpMatrix &tJe) const
{
  vpMatrix eJe;

  if (chainName == "Head" && m_robotType == Romeo)
  {
#ifdef VISP_NAOQI_HAVE_MATAPOD
    //Jacobian matrix w.r.t the torso
    //vpMatrix tJe;

    //confVector q for Romeo has size 24 (6 + 18dof of the robot, we don't consider the Legs and the fingers)
    RomeoModel::confVector q;

    // Get the names of the joints in the chain we want to control
    std::vector<std::string> jointNames = getBodyNames(chainName);

    // Get the angles of the joints in the chain we want to control
    std::vector<float> qmp = getAngles("Head", true );

    const unsigned int nJoints= qmp.size();

    //Resize the Jacobians
    eJe.resize(6,nJoints);
    tJe.resize(6,nJoints);

    // Create a robot instance of Metapod
    RomeoModel robot;

    //Get the index of the position of the q of the first joint of the chain in the confVector
    int index_confVec = (boost::fusion::at_c<RomeoModel::NeckYaw_link>(robot.nodes).q_idx);


    // Copy the angle values of the joint in the confVector in the right position
    // In the first 6 positions there is the FreeFlyer. We used the index_confVec to copy the rigth values.
    for(unsigned int i=0;i<nJoints;i++)
      q[i+index_confVec] = qmp[i];

    // Compute the Jacobian tJe
    jcalc< RomeoModel>::run(robot, q, RomeoModel::confVector::Zero());

    static const bool includeFreeFlyer = true;
    static const int offset = 0;
    typedef jac_point_chain<RomeoModel, RomeoModel::Torso_link, RomeoModel::HeadRoll_link, offset, includeFreeFlyer> jac;
    jac::Jacobian J = jac::Jacobian::Zero();
    jac::run(robot, q, Vector3dTpl<LocalFloatType>::Type(0,0,0), J);

    for(unsigned int i=0;i<3;i++)
      for(unsigned int j=0;j<nJoints;j++)
      {
        tJe[i][j]=J(i+3, index_confVec +j);
        tJe[i+3][j]=J(i, index_confVec +j);
      }

    //std::cout << "metapod_Jac:" <<std::endl << J;

    // Now we want to transform tJe to eJe

    vpHomogeneousMatrix torsoMHeadRoll(m_motionProxy->getTransform(jointNames[nJoints-1], 0, true));// get transformation  matrix between torso and HeadRoll
    vpVelocityTwistMatrix HeadRollVLtorso(torsoMHeadRoll.inverse());

    for(unsigned int i=0; i< 3; i++)
      for(unsigned int j=0; j< 3; j++)
        HeadRollVLtorso[i][j+3] = 0;

    // Transform the matrix
    eJe = HeadRollVLtorso *tJe;


#else
    throw vpRobotException (vpRobotException::readingParametersError,
                            "Metapod is not installed");
#endif


  }


  else if (chainName == "LArm"  && m_robotType == Romeo)
  {
#ifdef VISP_NAOQI_HAVE_MATAPOD
    //Jacobian matrix w.r.t the torso
    //vpMatrix tJe;

    //confVector q for Romeo has size 24 (6 + 18dof of the robot, we don't consider the Legs and the fingers)
    RomeoModel::confVector q;

    // Get the names of the joints in the chain we want to control
    std::vector<std::string> jointNames = getBodyNames(chainName);
    jointNames.pop_back(); // Delete last joints LHand, that we don't consider in the servo

    // Get the angles of the joints in the chain we want to control
    std::vector<float> qmp = getAngles(chainName, true );
    qmp.pop_back(); // we don't consider the last joint LHand

    const unsigned int nJoints = qmp.size();

    //Resize the Jacobians
    eJe.resize(6,nJoints);
    tJe.resize(6,nJoints);

    // Create a robot instance of Metapod
    RomeoModel robot;

    //Get the index of the position of the q of the first joint of the chain in the confVector
    int index_confVec = (boost::fusion::at_c<RomeoModel::LShoulderPitch_link>(robot.nodes).q_idx);

    // Copy the angle values of the joint in the confVector in the right position
    // In this case is +6 because in the first 6 positions there is the FreeFlyer
    for(unsigned int i=0;i<nJoints;i++)
      q[i+index_confVec] = qmp[i];

    // Compute the Jacobian tJe
    jcalc< RomeoModel>::run(robot, q, RomeoModel::confVector::Zero());

    static const bool includeFreeFlyer = true; // I don't consider the first six FreeFlyer
    static const int offset = 0;
    typedef jac_point_chain<RomeoModel, RomeoModel::Torso_link, RomeoModel::LWristPitch_link, offset, includeFreeFlyer> jac;
    jac::Jacobian J = jac::Jacobian::Zero();
    jac::run(robot, q, Vector3dTpl<LocalFloatType>::Type(0,0,0), J);

    for(unsigned int i=0;i<3;i++)
      for(unsigned int j=0;j<nJoints;j++)
      {
        tJe[i][j]=J(i+3,index_confVec+j); // Since the FreeFlyer are not activated the columns of the Jacobian relative to the LArm
        tJe[i+3][j]=J(i,index_confVec+j); // are from 0->6

      }

    //std::cout << "metapod_Jac:" <<std::endl << J << std::endl;


    // Now we want to transform tJe to eJe
    vpHomogeneousMatrix torsoMLWristP(m_motionProxy->getTransform(jointNames[nJoints-1], 0, true));
    vpVelocityTwistMatrix torsoVLWristP(torsoMLWristP.inverse());

    for(unsigned int i=0; i< 3; i++)
      for(unsigned int j=0; j< 3; j++)
        torsoVLWristP[i][j+3] = 0;
    // Transform the matrix
    eJe = torsoVLWristP *tJe;



#else
    throw vpRobotException (vpRobotException::readingParametersError,
                            "Metapod is not installed");

#endif // #ifdef VISP_NAOQI_HAVE_MATAPOD
  }



  else if (chainName == "LArm_t"  && m_robotType == Romeo)
  {
#ifdef VISP_NAOQI_HAVE_MATAPOD
    //Jacobian matrix w.r.t the torso
    //vpMatrix tJe;

    //confVector q for Romeo has size 24 (6 + 18dof of the robot, we don't consider the Legs and the fingers)
    RomeoModel::confVector q;

    // Get the names of the joints in the chain we want to control
    std::vector<std::string> jointNames;
    jointNames.push_back("TrunkYaw");
    std::vector<std::string> jointNamesLArm = getBodyNames("LArm");
    jointNamesLArm.pop_back(); // Delete last joints LHand, that we don't consider in the servo

    jointNames.insert(jointNames.end(),jointNamesLArm.begin(),jointNamesLArm.end());

    // Get the angles of the joints in the chain we want to control
    std::vector<float> qmp = getAngles(jointNames, true );

    const unsigned int nJoints = qmp.size();

    std::cout << "Number joints: " << nJoints << std::endl;

    //Resize the Jacobians
    eJe.resize(6,nJoints);
    tJe.resize(6,nJoints);

    // Create a robot instance of Metapod
    RomeoModel robot;

    //Get the index of the position of the q of the first joint of the chain in the confVector
    int index_confVec = (boost::fusion::at_c<RomeoModel::Torso_link>(robot.nodes).q_idx);

    // Copy the angle values of the joint in the confVector in the right position
    // In this case is +6 because in the first 6 positions there is the FreeFlyer
    for(unsigned int i=0;i<nJoints;i++)
      q[i+index_confVec] = qmp[i];

    // Compute the Jacobian tJe
    jcalc< RomeoModel>::run(robot, q, RomeoModel::confVector::Zero());

    static const bool includeFreeFlyer = true; // I don't consider the first six FreeFlyer
    static const int offset = 0;
    typedef jac_point_chain<RomeoModel, RomeoModel::TrunkYaw_link, RomeoModel::LWristPitch_link, offset, includeFreeFlyer> jac;
    jac::Jacobian J = jac::Jacobian::Zero();
    jac::run(robot, q, Vector3dTpl<LocalFloatType>::Type(0,0,0), J);

    for(unsigned int i=0;i<3;i++)
      for(unsigned int j=0;j<nJoints;j++)
      {
        tJe[i][j]=J(i+3,index_confVec+j); // Since the FreeFlyer are not activated the columns of the Jacobian relative to the LArm
        tJe[i+3][j]=J(i,index_confVec+j); // are from 0->6
      }

    //std::cout << "metapod_Jac:" <<std::endl << J << std::endl;
    vpHomogeneousMatrix tMLWristP(m_motionProxy->getTransform(jointNames[jointNames.size()-1], 2, true));// get transformation  matrix base torso and LWrist

    vpVelocityTwistMatrix LWristPVTrunk(tMLWristP.inverse());

    for(unsigned int i=0; i< 3; i++)
      for(unsigned int j=0; j< 3; j++)
        LWristPVTrunk[i][j+3] = 0;
    // Transform the matrix
    eJe = LWristPVTrunk *tJe;

#else
    throw vpRobotException (vpRobotException::readingParametersError,
                            "Metapod is not installed");

#endif // #ifdef VISP_NAOQI_HAVE_MATAPOD
  }




  else if (chainName == "RArm"  && m_robotType == Romeo)
  {
#ifdef VISP_NAOQI_HAVE_MATAPOD
    //Jacobian matrix w.r.t the torso
    //vpMatrix tJe;

    //confVector q for Romeo has size 24 (6 + 18dof of the robot, we don't consider the Legs and the fingers)
    RomeoModel::confVector q;

    // Get the names of the joints in the chain we want to control
    std::vector<std::string> jointNames = getBodyNames(chainName);
    jointNames.pop_back(); // Delete last joints LHand, that we don't consider in the servo

    // Get the angles of the joints in the chain we want to control
    std::vector<float> qmp = getAngles(chainName, true );
    qmp.pop_back(); // we don't consider the last joint LHand

    const unsigned int nJoints = qmp.size();

    //Resize the Jacobians
    eJe.resize(6,nJoints);
    tJe.resize(6,nJoints);

    // Create a robot instance of Metapod
    RomeoModel robot;

    //Get the index of the position of the q of the first joint of the chain in the confVector
    int index_confVec = (boost::fusion::at_c<RomeoModel::RShoulderPitch_link>(robot.nodes).q_idx);

    // Copy the angle values of the joint in the confVector in the right position
    // In this case is +6 because in the first 6 positions there is the FreeFlyer
    for(unsigned int i=0;i<nJoints;i++)
      q[i+index_confVec] = qmp[i];

    // Compute the Jacobian tJe
    jcalc< RomeoModel>::run(robot, q, RomeoModel::confVector::Zero());

    static const bool includeFreeFlyer = true;
    static const int offset = 0;
    typedef jac_point_chain<RomeoModel, RomeoModel::Torso_link, RomeoModel::RWristPitch_link, offset, includeFreeFlyer> jac;
    jac::Jacobian J = jac::Jacobian::Zero();
    jac::run(robot, q, Vector3dTpl<LocalFloatType>::Type(0,0,0), J);

    for(unsigned int i=0;i<3;i++)
      for(unsigned int j=0;j<nJoints;j++)
      {
        tJe[i][j]=J(i+3,index_confVec+j);
        tJe[i+3][j]=J(i,index_confVec+j);
      }


    // Now we want to transform tJe to eJe
    vpHomogeneousMatrix torsoMRWristP(m_pMotion_.call<std::vector<float> >("getTransform", jointNames[nJoints-1], 0, true ));

    vpVelocityTwistMatrix torsoVRWristP(torsoMRWristP.inverse());

    for(unsigned int i=0; i< 3; i++)
      for(unsigned int j=0; j< 3; j++)
        torsoVRWristP[i][j+3] = 0;

    // Transform the matrix
    eJe = torsoVRWristP *tJe;

#else
    throw vpRobotException (vpRobotException::readingParametersError,
                            "Metapod is not installed");

#endif // #ifdef VISP_NAOQI_HAVE_MATAPOD
  }

  else if (chainName == "LEye" && m_robotType == Romeo)
  {
#ifdef VISP_NAOQI_HAVE_MATAPOD
    //Jacobian matrix w.r.t the torso
    //vpMatrix tJe;

    //confVector q for Romeo has size 28 (6 + 22dof of the robot, we don't consider the Legs and the fingers)
    RomeoModel::confVector q;

    // Get the names of the joints in the chain we want to control (Head + Eye)
    std::vector<std::string> jointNames = getBodyNames("Head");
    std::vector<std::string> jointNamesEye = getBodyNames(chainName);

    jointNames.insert(jointNames.end(), jointNamesEye.begin(), jointNamesEye.end());


    // Get the angles of the joints in the chain we want to control
    std::vector<float> qmp = getAngles(jointNames, true);

    const unsigned int nJoints= qmp.size();

    //Resize the Jacobians
    eJe.resize(6,nJoints);
    tJe.resize(6,nJoints);

    // Create a robot instance of Metapod
    RomeoModel robot;

    //Get the index of the position of the q of the first joint of the chain in the confVector
    int index_confVec = (boost::fusion::at_c<RomeoModel::NeckYaw_link>(robot.nodes).q_idx);


    // Copy the angle values of the joint in the confVector in the right position
    // In the first 6 positions there is the FreeFlyer. We used the index_confVec to copy the rigth values.
    for(unsigned int i=0;i<nJoints;i++)
      q[i+index_confVec] = qmp[i];

    // Compute the Jacobian tJe
    jcalc< RomeoModel>::run(robot, q, RomeoModel::confVector::Zero());

    static const bool includeFreeFlyer = true;
    static const int offset = 0;
    typedef jac_point_chain<RomeoModel, RomeoModel::Torso_link, RomeoModel::LEyePitch_link, offset, includeFreeFlyer> jac;
    jac::Jacobian J = jac::Jacobian::Zero();
    jac::run(robot, q, Vector3dTpl<LocalFloatType>::Type(0,0,0), J);

    for(unsigned int i=0;i<3;i++)
      for(unsigned int j=0;j<nJoints;j++)
      {
        tJe[i][j]=J(i+3, index_confVec +j);
        tJe[i+3][j]=J(i, index_confVec +j);

      }

    //std::cout << "metapod_Jac:" <<std::endl << J;

    // Now we want to transform tJe to eJe
    vpHomogeneousMatrix torsoMLEye(m_pMotion_.call<std::vector<float> >("getTransform", jointNames[nJoints-1], 0, true ));// get transformation  matrix between torso and LEye
    vpVelocityTwistMatrix LEyeVLtorso(torsoMLEye.inverse());

    for(unsigned int i=0; i< 3; i++)
      for(unsigned int j=0; j< 3; j++)
        LEyeVLtorso[i][j+3] = 0;


    // Transform the matrix
    eJe = LEyeVLtorso * tJe;


#else
    throw vpRobotException (vpRobotException::readingParametersError,
                            "Metapod is not installed");
#endif


  }

  else if (chainName == "LEye_t" && m_robotType == Romeo) // Consider the trunk
  {
#ifdef VISP_NAOQI_HAVE_MATAPOD
    //Jacobian matrix w.r.t the torso
    //vpMatrix tJe;

    //confVector q for Romeo has size 28 (6 + 22dof of the robot, we don't consider the Legs and the fingers)
    RomeoModel::confVector q;

    // Get the names of the joints in the chain we want to control (Trunk + Head + Eye)
    std::vector<std::string> jointNamesTrunk;
    jointNamesTrunk.push_back("TrunkYaw");
    std::vector<std::string> jointNamesHead = getBodyNames("Head");
    std::vector<std::string> jointNamesEye = getBodyNames("LEye");

    // Get the angles of the joints in the chain we want to control
    std::vector<float> qmp_trunk = getAngles("TrunkYaw", true);
    qmp_trunk[0] = -qmp_trunk[0];
    std::vector<float> qmp_head = getAngles(jointNamesHead, true);
    std::vector<float> qmp_leye = getAngles(jointNamesEye, true);

    //std::cout << "Values joint: qmp_trunk " << std::endl << qmp_trunk <<std::endl ;
    //std::cout << "Values joint: qmp_head " << std::endl << qmp_head <<std::endl ;
    //std::cout << "Values joint: qmp_leye " << std::endl << qmp_leye <<std::endl ;

    const unsigned int nJoints = qmp_trunk.size() + qmp_head.size() + qmp_leye.size();

    //std::cout << "nJoints " << std::endl << nJoints <<std::endl ;

    //Resize the Jacobians
    eJe.resize(6,nJoints);
    tJe.resize(6,nJoints);

    // Create a robot instance of Metapod
    RomeoModel robot;

    //Get the index of the position of the q of the first joint of the chain in the confVector
    int index_confVec_trunk = (boost::fusion::at_c<RomeoModel::Torso_link>(robot.nodes).q_idx);
    int index_confVec_head = (boost::fusion::at_c<RomeoModel::NeckYaw_link>(robot.nodes).q_idx);
    int index_confVec_leye = (boost::fusion::at_c<RomeoModel::LEyeYaw_link>(robot.nodes).q_idx);

    // Copy the angle values of the joint in the confVector in the right position
    // In the first 6 positions there is the FreeFlyer. We used the index_confVec to copy the rigth values.

    //std::cout << "index_confVec_trunk: " <<std::endl << index_confVec_trunk <<std::endl;
    // std::cout << "index_confVec_head: " <<std::endl << index_confVec_head <<std::endl;
    //std::cout << "index_confVec_leye: " <<std::endl << index_confVec_leye <<std::endl;

    for(unsigned int i=0;i<jointNamesTrunk.size();i++)
      q[i+index_confVec_trunk] = qmp_trunk[i];

    for(unsigned int i=0;i<jointNamesHead.size();i++)
      q[i+index_confVec_head] = qmp_head[i];

    for(unsigned int i=0;i<jointNamesEye.size();i++)
      q[i+index_confVec_leye] = qmp_leye[i];


    // std::cout << "q: " <<std::endl << q;

    // Compute the Jacobian tJe
    jcalc< RomeoModel>::run(robot, q, RomeoModel::confVector::Zero());

    static const bool includeFreeFlyer = true;
    static const int offset = 0;
    typedef jac_point_chain<RomeoModel, RomeoModel::TrunkYaw_link, RomeoModel::LEyePitch_link, offset, includeFreeFlyer> jac;
    jac::Jacobian J = jac::Jacobian::Zero();
    jac::run(robot, q, Vector3dTpl<LocalFloatType>::Type(0,0,0), J);
    //std::cout << "Metapod_Jac:" <<std::endl << J <<std::endl;

    // Copy jacobian trunk
    for(unsigned int i=0;i<3;i++)
      for(unsigned int j=0;j<jointNamesTrunk.size();j++)
      {
        tJe[i][j]=J(i+3, index_confVec_trunk +j);
        tJe[i+3][j]=J(i, index_confVec_trunk +j);
      }

    // Copy jacobian Head
    for(unsigned int i=0;i<3;i++)
      for(unsigned int j=0;j<jointNamesHead.size();j++)
      {
        tJe[i][j+1]=J(i+3, index_confVec_head +j);
        tJe[i+3][j+1]=J(i, index_confVec_head +j);
      }

    // Copy REye
    for(unsigned int i=0;i<3;i++)
      for(unsigned int j=0;j<jointNamesEye.size();j++)
      {
        tJe[i][j+5]=J(i+3, index_confVec_leye +j);
        tJe[i+3][j+5]=J(i, index_confVec_leye +j);
      }

    //std::cout << "tJe: " <<std::endl << tJe <<std::endl;
    // Now we want to transform tJe to eJe
    vpHomogeneousMatrix tMREye(m_pMotion_.call<std::vector<float> >("getTransform", jointNamesEye[jointNamesEye.size()-1], 2, true ));// get transformation  matrix base torso and LEye
    // vpHomogeneousMatrix tMTrunk(m_motionProxy->getTransform("TrunkYaw", 0, true));// get transformation  matrix base torso and TrunkYaw

    vpVelocityTwistMatrix REyeVTrunk(tMREye.inverse());

    for(unsigned int i=0; i< 3; i++)
      for(unsigned int j=0; j< 3; j++)
        REyeVTrunk[i][j+3] = 0;

    // Transform the matrix
    eJe = REyeVTrunk * tJe;

#else
    throw vpRobotException (vpRobotException::readingParametersError,
                            "Metapod is not installed");
#endif
  }

  else if (chainName == "REye" && m_robotType == Romeo)
  {
#ifdef VISP_NAOQI_HAVE_MATAPOD
    //Jacobian matrix w.r.t the torso
    //vpMatrix tJe;

    //confVector q for Romeo has size 28 (6 + 22dof of the robot, we don't consider the Legs and the fingers)
    RomeoModel::confVector q;

    // Get the names of the joints in the chain we want to control (Head + Eye)
    std::vector<std::string> jointNamesHead = getBodyNames("Head");
    std::vector<std::string> jointNamesEye = getBodyNames(chainName);

    //jointNames.insert(jointNames.end(), jointNamesEye.begin(), jointNamesEye.end());


    // Get the angles of the joints in the chain we want to control
    std::vector<float> qmp_head = getAngles(jointNamesHead, true );
    std::vector<float> qmp_leye = getAngles(jointNamesEye, true );

    const unsigned int nJoints= qmp_head.size() + qmp_leye.size();

    //Resize the Jacobians
    eJe.resize(6,nJoints);
    tJe.resize(6,nJoints);

    // Create a robot instance of Metapod
    RomeoModel robot;

    //Get the index of the position of the q of the first joint of the chain in the confVector
    int index_confVec = (boost::fusion::at_c<RomeoModel::NeckYaw_link>(robot.nodes).q_idx);
    int index_confVec_leye = (boost::fusion::at_c<RomeoModel::REyeYaw_link>(robot.nodes).q_idx);

    // Copy the angle values of the joint in the confVector in the right position
    // In the first 6 positions there is the FreeFlyer. We used the index_confVec to copy the rigth values.
    for(unsigned int i=0;i<jointNamesHead.size();i++)
      q[i+index_confVec] = qmp_head[i];

    for(unsigned int i=0;i<jointNamesEye.size();i++)
      q[i+index_confVec_leye] = qmp_leye[i];


    // std::cout << "q: " <<std::endl << q;

    // Compute the Jacobian tJe
    jcalc< RomeoModel>::run(robot, q, RomeoModel::confVector::Zero());

    static const bool includeFreeFlyer = true;
    static const int offset = 0;
    typedef jac_point_chain<RomeoModel, RomeoModel::Torso_link, RomeoModel::REyePitch_link, offset, includeFreeFlyer> jac;
    jac::Jacobian J = jac::Jacobian::Zero();
    jac::run(robot, q, Vector3dTpl<LocalFloatType>::Type(0,0,0), J);
    // std::cout << "metapod_Jac:" <<std::endl << J <<std::endl;

    // Copy jacobian Head
    for(unsigned int i=0;i<3;i++)
      for(unsigned int j=0;j<jointNamesHead.size();j++)
      {
        tJe[i][j]=J(i+3, index_confVec +j);
        tJe[i+3][j]=J(i, index_confVec +j);
      }

    // std::cout << "tJe: " <<std::endl << tJe;
    // Copy REye
    for(unsigned int i=0;i<3;i++)
      for(unsigned int j=0;j<jointNamesEye.size();j++)
      {
        tJe[i][j+4]=J(i+3, index_confVec_leye +j);
        tJe[i+3][j+4]=J(i, index_confVec_leye +j);
      }

    //std::cout << "tJe: " <<std::endl << tJe <<std::endl;

    // Now we want to transform tJe to eJe // CHECK IF 0 or 2
    vpHomogeneousMatrix torsoMREye(m_pMotion_.call<std::vector<float> >("getTransform", jointNamesEye[jointNamesEye.size()-1], 2, true ));// get transformation  matrix between torso and REye
    vpVelocityTwistMatrix REyeVLtorso(torsoMREye.inverse());

    for(unsigned int i=0; i< 3; i++)
      for(unsigned int j=0; j< 3; j++)
        REyeVLtorso[i][j+3] = 0;

    // Transform the matrix
    eJe = REyeVLtorso * tJe;

#else
    throw vpRobotException (vpRobotException::readingParametersError,
                            "Metapod is not installed");
#endif
  }

  else if (chainName == "Head" && m_robotType == Pepper)
  {
#ifdef VISP_NAOQI_HAVE_MATAPOD
    //Jacobian matrix w.r.t the torso
    //vpMatrix tJe;

    // Create a robot instance of Metapod
    PepperModel robot;

    //confVector q for Pepper
    PepperModel::confVector q;

    // Get the names of the joints in the chain we want to control
    std::vector<std::string> jointNames = getBodyNames(chainName);

    // Get the angles of the joints in the chain we want to control
    std::vector<float> qmp = getAngles("Head", true );

    const unsigned int nJoints= qmp.size();

    //Resize the Jacobians
    eJe.resize(6,nJoints);
    tJe.resize(6,nJoints);

    //Get the index of the position of the q of the first joint of the chain in the confVector
    int index_confVec = (boost::fusion::at_c<PepperModel::Neck>(robot.nodes).q_idx);

    // Copy the angle values of the joint in the confVector in the right position
    // In the first 6 positions there is the FreeFlyer. We used the index_confVec to copy the rigth values.
    for(unsigned int i=0;i<nJoints;i++)
      q[i+index_confVec] = qmp[i];

    // Compute the Jacobian tJe
    jcalc< PepperModel>::run(robot, q, PepperModel::confVector::Zero());

    static const bool includeFreeFlyer = true;
    static const int offset = 0;
    typedef jac_point_chain<PepperModel, PepperModel::torso, PepperModel::Head, offset, includeFreeFlyer> jac;
    jac::Jacobian J = jac::Jacobian::Zero();
    jac::run(robot, q, Vector3dTpl<LocalFloatType>::Type(0,0,0), J);

    for(unsigned int i=0;i<3;i++)
      for(unsigned int j=0;j<nJoints;j++)
      {
        tJe[i][j]=J(i+3, index_confVec +j);
        tJe[i+3][j]=J(i, index_confVec +j);

      }

    // std::cout << "metapod_Jac:" <<std::endl << J;

    // Now we want to transform tJe to eJe
    vpHomogeneousMatrix torsoMHead(m_pMotion_.call<std::vector<float> >("getTransform", jointNames[nJoints-1], 0, true ));// get transformation  matrix between torso and Head
    vpVelocityTwistMatrix HeadVLtorso(torsoMHead.inverse());

    for(unsigned int i=0; i< 3; i++)
      for(unsigned int j=0; j< 3; j++)
        HeadVLtorso[i][j+3] = 0;

    // Transform the matrix
    eJe = HeadVLtorso *tJe;


#else
    throw vpRobotException (vpRobotException::readingParametersError,
                            "Metapod is not installed");
#endif

  }



  else if (chainName == "RArm"  && m_robotType == Pepper)
  {
#ifdef VISP_NAOQI_HAVE_MATAPOD

    //confVector q for Pepper
    PepperModel::confVector q;

    // Get the names of the joints in the chain we want to control
    std::vector<std::string> jointNames = getBodyNames(chainName);
    jointNames.pop_back(); // Delete last joints LHand, that we don't consider in the servo

    // std::cout << "jointNames: " <<std::endl << jointNames;

    // Get the angles of the joints in the chain we want to control
    std::vector<float> qmp = getAngles(chainName, true );
    qmp.pop_back(); // we don't consider the last joint LHand

    const unsigned int nJoints = qmp.size();

    //Resize the Jacobians
    eJe.resize(6,nJoints);
    tJe.resize(6,nJoints);

    // Create a robot instance of Metapod
    PepperModel robot;

    //Get the index of the position of the q of the first joint of the chain in the confVector
    int index_confVec = (boost::fusion::at_c<PepperModel::RShoulder>(robot.nodes).q_idx);

    // Copy the angle values of the joint in the confVector in the right position
    // In this case is +6 because in the first 6 positions there is the FreeFlyer
    for(unsigned int i=0;i<nJoints;i++)
      q[i+index_confVec] = qmp[i];

    // Compute the Jacobian tJe
    jcalc< PepperModel>::run(robot, q, PepperModel::confVector::Zero());

    static const bool includeFreeFlyer = true;
    static const int offset = 0;
    typedef jac_point_chain<PepperModel, PepperModel::torso, PepperModel::r_wrist, offset, includeFreeFlyer> jac;
    jac::Jacobian J = jac::Jacobian::Zero();
    jac::run(robot, q, Vector3dTpl<LocalFloatType>::Type(0,0,0), J);

    for(unsigned int i=0;i<3;i++)
      for(unsigned int j=0;j<nJoints;j++)
      {
        tJe[i][j]=J(i+3,index_confVec+j);
        tJe[i+3][j]=J(i,index_confVec+j);

      }
    //std::cout << "metapod_Jac:" <<std::endl << J;


    // Now we want to transform tJe to eJe
    vpHomogeneousMatrix torsoMRWristP(m_pMotion_.call<std::vector<float> >("getTransform", jointNames[nJoints-1], 0, true ));

    vpVelocityTwistMatrix torsoVRWristP(torsoMRWristP.inverse());

    for(unsigned int i=0; i< 3; i++)
      for(unsigned int j=0; j< 3; j++)
        torsoVRWristP[i][j+3] = 0;

    // Transform the matrix
    eJe = torsoVRWristP *tJe;


#else
    throw vpRobotException (vpRobotException::readingParametersError,
                            "Metapod is not installed");

#endif // #ifdef VISP_NAOQI_HAVE_MATAPOD
  }



  else
  {
    throw vpRobotException (vpRobotException::readingParametersError,
                            "End-effector name not recognized. Please choose one above 'Head', 'LEye', 'LArm' or 'RArm' ");
  }
  return eJe;
}

/*!
  Get the derivative of the kinematic jacobian specifying an end effector chain name.

  \param chainName : Name of the end effector. Allowed values are "Head",
  "LArm" for left arm and "RArm" for right arm.

  \return The actual derivative of the kinematic jacobian.
 */
std::vector <vpMatrix> vpNaoqiRobot::get_d_eJe(const std::string &chainName) const
{
  std::vector <vpMatrix> dtJe;

  if (chainName == "LArm" && m_robotType == Romeo)
  {
    // Get the names of the joints in the chain we want to control
    std::vector<std::string> jointNames = getBodyNames(chainName);
    jointNames.pop_back(); // Delete last joints LHand, that we don't consider in the servo

    // Derivatives of the Jacobians matrix w.r.t the torso
    dtJe.resize(jointNames.size());

#ifdef VISP_NAOQI_HAVE_MATAPOD

    //confVector q for Romeo has size 24 (6 + 18dof of the robot, we don't consider the Legs and the fingers)
    RomeoModel::confVector q;
    RomeoModel::confVector dq;

    // Get the angles of the joints in the chain we want to control
    std::vector<float> qmp = getAngles(chainName,true);
    qmp.pop_back(); // we don't consider the last joint LHand

    const unsigned int nJoints = qmp.size();

    // Resize the Jacobians
    // dtJe.resize(6,nJoints*nJoints);

    // Create a robot instance of Metapod
    RomeoModel robot;

    // Get the index of the position of the q of the first joint of the chain in the confVector
    int index_confVec = (boost::fusion::at_c<RomeoModel::LShoulderPitch_link>(robot.nodes).q_idx);

    // Copy the angle values of the joint in the confVector in the right position
    // In this case is +6 because in the first 6 positions there is the FreeFlyer
    for(unsigned int i=0;i<nJoints;i++)
      q[i+index_confVec] = qmp[i];

    //        for(unsigned int i=0;i<RomeoModel::NBDOF;i++)
    //            std::cout<<q[i] << std::endl;

    std::vector <std::string > names_larm = getBodyNames(chainName);
    names_larm.pop_back(); // We don't consider the last joint of the hand (open/close)
    // Get actual velocities
    vpColVector dq_ = getJointVelocity(names_larm);

    for(unsigned int i=0;i<nJoints;i++)
      dq[i+index_confVec] = dq_[i];



    // Compute the Jacobian tJe
    jcalc< RomeoModel>::run(robot, q, dq);
    rnea< RomeoModel>::run(robot, q, dq, RomeoModel::confVector::Zero());


    //        std::ofstream state_log("djac_jcalc_state.log", std::ofstream::out);
    //        printState<RomeoModel>(robot, state_log);
    //        state_log.close();

    typedef Eigen::Matrix<LocalFloatType, 6 * RomeoModel::NBBODIES, RomeoModel::NBDOF> dJacobian;
    dJacobian dJ = dJacobian::Zero();
    djac< RomeoModel>::run(robot, dJ);


    for (unsigned int k = 0; k < nJoints; k++)
    {
      //vpMatrix dJk(6,nJoints);

      dtJe.at(k).resize(6,nJoints);
      for(unsigned int i=0;i<3;i++)
        for(unsigned int j=0;j<nJoints;j++)
        {
          dtJe.at(k)[i][j]=dJ(6*k+i+3,index_confVec+j);
          dtJe.at(k)[i+3][j]=dJ(6*k + i,index_confVec+j);

        }

      //std::cout << "derivative dJ/d" << k << std::endl;
      //std::cout << dJk << std::endl;

      //dtJe.push_back(dJk);

    }

    //        const char result_file[] = "djac.log";
    //        std::ofstream log(result_file, std::ofstream::out);
    //        log << "derivative_of_the_kinematic_jacobian\n" << dJ << std::endl;
    //        log.close();

    //        state_log.open("djac_state.log", std::ofstream::out);
    //        printState<RomeoModel>(robot, state_log);
    //        state_log.close();


#else
    throw vpRobotException (vpRobotException::readingParametersError,
                            "Metapod is not installed");

#endif // #ifdef VISP_NAOQI_HAVE_MATAPOD



  }
  else
  {
    throw vpRobotException (vpRobotException::readingParametersError,
                            "End-effector name not recognized. Please choose one above 'LArm' or 'RArm' ");
  }

  return dtJe;

}


/*!
  Get the Transformation matrix to the end-effectors

  \param endEffectorName : Name of the end effector. Allowed values are "CameraLeft",
  "CameraRight" for the fixed camera on the head, "RHand" and "LHand" for end-effector on the arms.

  \return The transformation matrix to the end-effectors (computed from the last joint of the chain ending with the end-effector. )
 */

vpHomogeneousMatrix vpNaoqiRobot::get_cMe(const std::string &endEffectorName)
{

  std::cout << "Function get_cMe is deprecated. Use vpNaoqiGrabber::getExtrinsicCameraParameters instead." << std::endl;
  vpHomogeneousMatrix cMe;

  // Transformation matrix from CameraLeft to HeadRoll
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
    cMe[2][3] = -0.11999;
  }

  else if (endEffectorName == "CameraLeft_aldebaran")
  {

    vpHomogeneousMatrix eMc_ald;
    eMc_ald[0][3] = 0.11999;
    eMc_ald[1][3] = 0.04;
    eMc_ald[2][3] = 0.09938;

    vpHomogeneousMatrix cam_alMe_camvisp;
    for(unsigned int i=0; i<3; i++)
      cam_alMe_camvisp[i][i] = 0; // remove identity
    cam_alMe_camvisp[0][2] = 1.;
    cam_alMe_camvisp[1][0] = -1.;
    cam_alMe_camvisp[2][1] = -1.;

    cMe = (eMc_ald * cam_alMe_camvisp).inverse();

  }

  else
  {
    throw vpRobotException (vpRobotException::readingParametersError,
                            "Transformation matrix that you requested is not implemented. Valid values: CameraLeft.");
  }
  return cMe;
}


/*!
  Get the joints velocities.

  \param names :  Names of the joints, chains, "Body", "JointActuators",
  "Joints" or "Actuators".

  \return Joint velocities in radians/s.
 */
vpColVector vpNaoqiRobot::getJointVelocity(const std::vector <std::string> &names) const
{
  std::vector<std::string> list;

  for (unsigned int i = 0; i < names.size(); i++)
  {
    std::string key;
    if (m_robotType == Pepper )
      key = "Motion/Velocity/Sensor/" + names[i];
    else
      key = "Device/SubDeviceList/" + names[i] + "/Speed/Actuator/Value";

    list.push_back(key);
  }

  qi::AnyValue memData_vel = m_pMemory.call<qi::AnyValue>("getListData", list);
  qi::AnyReferenceVector memData_anyref = memData_vel.asListValuePtr();

  vpColVector vel(names.size());

  for(int i=0; i<memData_anyref.size();i++)
  {
    if(memData_anyref[i].content().kind() == qi::TypeKind_Float)
      vel[i] = memData_anyref[i].content().asFloat();
  }

  return vel;
}

/*!
  Get the joints velocities.

  \param names :  Vector with the joint names.
  \param names :  Vector to fill with the joint velocities.

 */
void vpNaoqiRobot::getJointVelocity(const std::vector <std::string> &names, std::vector <float> &jointVel) const
{

  std::vector<std::string> list;

  for (unsigned int i = 0; i < names.size(); i++)
  {
    std::string key;
    if (m_robotType == Pepper )
      key = "Motion/Velocity/Sensor/" + names[i];
    else
      key = "Device/SubDeviceList/" + names[i] + "/Speed/Actuator/Value";

    list.push_back(key);
  }

  qi::AnyValue memData_vel = m_pMemory.call<qi::AnyValue>("getListData", list);
  qi::AnyReferenceVector memData_anyref = memData_vel.asListValuePtr();

  for(int i=0; i<memData_anyref.size();i++)
  {
    if(memData_anyref[i].content().kind() == qi::TypeKind_Float)
      jointVel[i] = memData_anyref[i].content().asFloat();
  }
  return;

}



