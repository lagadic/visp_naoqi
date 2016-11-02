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
#include <visp_naoqi/vpNaoqiConfig.h>

// Visp
#include <visp/vpVelocityTwistMatrix.h>


// Aldebaran SDK
#include <alcommon/albroker.h>

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
vpNaoqiRobot::vpNaoqiRobot()
    : m_motionProxy(NULL),m_proxy(NULL), m_robotIp("198.18.0.1"), m_isOpen(false), m_collisionProtection(true),
      m_robotName(""), m_robotType(Unknown), m_memProxy(NULL), m_session(), m_pepper_control()
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

        int   success = m_motionProxy->setCollisionProtectionEnabled("Arms", false);
        if (success)
            std::cout << "Collision protection is disabled" << std::endl;

        m_memProxy = new AL::ALMemoryProxy(m_robotIp, 9559);

        // Check the type of the robot
        AL::ALValue robotConfig = m_motionProxy->getRobotConfig();
        m_robotName = std::string(robotConfig[1][0]);

        // Create a general proxy to motion to use new functions not implemented in the specialized proxy
        std::string 	myIP = "";		// IP du portable (voir /etc/hosts)
        int 	myPort = 0 ;			// Default broker port

        boost::shared_ptr<AL::ALBroker> broker = AL::ALBroker::createBroker("BrokerMotion", myIP, myPort, m_robotIp, 9559);
        m_proxy = new AL::ALProxy(broker, "ALMotion");

        if (m_robotName.find("romeo") != std::string::npos) {
            m_robotType = Romeo;
            std::cout << "This robot is Romeo" << std::endl;
        }
        else if (m_robotName.find("nao") != std::string::npos) {
            m_robotType = Nao;
            std::cout << "This robot is Nao" << std::endl;
        }
        else if (m_robotName.find("juliette") != std::string::npos) {
            m_robotType = Pepper;
            // Create a robot instance of Metapod
            std::cout << "This robot is Pepper" << std::endl;

            AL::ALValue config;
            AL::ALValue one = AL::ALValue::array(std::string("CONTROL_JOINT_MAX_ACCELERATION"),AL::ALValue(5.0));
            config.arrayPush(one);

            m_motionProxy->setMotionConfig(config);

            try
            {
                m_session = qi::makeSession();
                std::string ip_port = "tcp://" + m_robotIp + ":9559";
                m_session->connect(ip_port);
                m_pepper_control = m_session->service("pepper_control");
                m_pepper_control.call<void >("start");
            }
            catch (const AL::ALError &e)
            {
                std::cerr << "Caught exception: " << e.what() << std::endl;
                std::cerr << "Check if the module pepper_control is running on the robot. " << std::endl;
            }


            // qi::SessionPtr session = qi::makeSession();
            // std::string ip_port = "tcp://" + m_robotIp + ":9559";
            // session->connect(ip_port);
            // m_qiProxy = new qi::AnyObject();
            // *m_qiProxy = session->service("ALMotion");
        }
        else {
            std::cout << "Unknown robot" << std::endl;
        }

        if ( m_robotType != Pepper)
        {

          int   success = m_motionProxy->setCollisionProtectionEnabled("Arms", false);
          if (success)
              std::cout << "Collision protection is disabled" << std::endl;

            // Set Trapezoidal interpolation
            AL::ALValue config;

            AL::ALValue one = AL::ALValue::array(std::string("CONTROL_USE_ACCELERATION_INTERPOLATOR"),AL::ALValue(1));
            AL::ALValue two = AL::ALValue::array(std::string("CONTROL_JOINT_MAX_ACCELERATION"),AL::ALValue(5.0));

            config.arrayPush(one);
            config.arrayPush(two);

            m_motionProxy->setMotionConfig(config);

            std::cout << "Trapezoidal interpolation is on " << std::endl;
        }
        //On nao, we have joint coupled limits (http://doc.aldebaran.com/2-1/family/robots/joints_robot.html) on the head and ankle.
        //Motion and DCM have clamping. We have to remove motion clamping.
        if (m_robotName.find("nao") != std::string::npos)
        {
            AL::ALValue config_;
            AL::ALValue setting = AL::ALValue::array(std::string("ENABLE_DCM_LIKE_CLAMPING"),AL::ALValue(0));
            config_.arrayPush(setting);
        }

        m_isOpen = true;
    }
}

void vpNaoqiRobot::cleanup()
{
    if (m_motionProxy != NULL) {
        delete m_motionProxy;
        m_motionProxy = NULL;
    }

    if (m_proxy != NULL) {
        delete m_proxy;
        m_proxy = NULL;
    }

    if (m_memProxy != NULL) {
        delete m_memProxy;
        m_memProxy = NULL;
    }

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

void vpNaoqiRobot::setVelocity_eachJoint(const AL::ALValue& names, const vpColVector &jointVel, bool verbose)
{
    std::vector<float> jointVel_(jointVel.size());
    for (unsigned int i=0; i< jointVel.size(); i++)
        jointVel_[i] = jointVel[i];
    setVelocity_eachJoint(names, jointVel_);
}
/*!
  Apply a velocity vector to a vector of joints.
  \param names :  Names the joints, chains, "Body", "JointActuators",
  "Joints" or "Actuators".
  \param jointVel : Joint velocity vector with values expressed in rad/s.
  \param verbose : If true activates printings.
 */

void vpNaoqiRobot::setVelocity_eachJoint(const AL::ALValue& names, const std::vector<float> &jointVel, bool verbose)
{
    setVelocity_eachJoint(names, (AL::ALValue)(jointVel));
}
/*!
  Apply a velocity vector to a vector of joints.
  \param names :  Names the joints, chains, "Body", "JointActuators",
  "Joints" or "Actuators".
  \param jointVel : Joint velocity vector with values expressed in rad/s.
  \param verbose : If true activates printings.
 */

void vpNaoqiRobot::setVelocity(const AL::ALValue& names, const std::vector<float> &jointVel, bool verbose)
{
    setVelocity(names, (AL::ALValue)(jointVel), verbose);
}

/*!
  Apply a velocity vector to a vector of joints.

  \todo Improve the function to be able to pass just one joint as names argument.

  \param names :  Names the joints, chains, "Body", "JointActuators",
  "Joints" or "Actuators".
  \param jointVel : Joint velocity vector with values expressed in rad/s.
  \param verbose : If true activates printings.
 */
void vpNaoqiRobot::setVelocity_eachJoint(const AL::ALValue &names, const AL::ALValue &jointVel, bool verbose)
{
    std::vector<std::string> jointNames;
    if (names.isString()) // Suppose to be a chain
        jointNames = m_motionProxy->getBodyNames(names);
    else if (names.isArray()) // Supposed to be a vector of joints
        jointNames = names; // it a vector of joints
    else
        throw vpRobotException (vpRobotException::readingParametersError,
                                "Unable to decode the joint chain.");

    if (jointNames.size() != jointVel.getSize() ) {
        throw vpRobotException (vpRobotException::readingParametersError,
                                "The dimensions of the joint array and the velocities array do not match.");
    }

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
  Apply a velocity vector to a vector of joints.Use just one call to apply the velocities.
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
    setVelocity(names, jointVel_, verbose);
}



/*!
  Apply a velocity vector to a vector of joints. Use just one call to apply the velocities.

  \param names :  Names the joints, chains, "Body", "JointActuators",
  "Joints" or "Actuators".
  \param jointVel : Joint velocity vector with values expressed in rad/s.
  \param verbose : If true activates printings.
 */
void vpNaoqiRobot::setVelocity(const AL::ALValue &names, const AL::ALValue &jointVel, bool verbose)
{
    std::vector<std::string> jointNames;
    if (names.isString()) // Suppose to be a chain
        jointNames = m_motionProxy->getBodyNames(names);
    else if (names.isArray()) // Supposed to be a vector of joints
        jointNames = names; // it a vector of joints
    else
        throw vpRobotException (vpRobotException::readingParametersError,
                                "Unable to decode the joint chain.");

    if (jointNames.size() != jointVel.getSize() ) {
        throw vpRobotException (vpRobotException::readingParametersError,
                                "The dimensions of the joint array and the velocities array do not match.");
    }

    if (m_robotType == Romeo || m_robotType == Nao)
    {
      AL::ALValue jointListStop;
      AL::ALValue jointListMove;
      AL::ALValue angles;
      std::vector<float> fractions;

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


          if (vel == 0.0f)
          {
              if (verbose)
                  std::cout << "Stop the joint" << std::endl ;

              jointListStop.arrayPush(jointName);
          }
          else
          {

              if (vel > 0.0f)
              {
                  //Reach qMax
                  angles.arrayPush(limits[0][1]);
                  if (verbose)
                      std::cout << "Reach qMax (" << limits[0][1] << ") ";
              }

              else if (vel < 0.0f)
              {
                  //Reach qMin
                  angles.arrayPush(limits[0][0]);
                  if (verbose)
                      std::cout << "Reach qMin (" << limits[0][0] << ") ";
              }


              jointListMove.arrayPush(jointName);
              float fraction = fabs( float (vel/float(limits[0][2])));
              if (fraction >= 1.0 )
              {
                  if (verbose) {
                      std::cout << "Given velocity is too high: " <<  vel << "rad/s for " << jointName << "." << std::endl;
                      std::cout << "Max allowed is: " << limits[0][2] << "rad/s for "<< std::endl;
                  }
                  fraction = 1.0;
              }

              fractions.push_back(fraction);

          }
      }
      if (verbose) {
          std::cout << "Apply Velocity to joints " << jointListMove << std::endl;
          std::cout << "Stop List joints: " << jointListStop << std::endl;
          std::cout << "with fractions " << angles << std::endl;
          std::cout << "to angles " << fractions << std::endl;
      }

      if (jointListMove.getSize()>0)
      {
          m_proxy->callVoid("setAngles", jointListMove, angles, fractions);
      }

      if (jointListStop.getSize()>0)
      {
          std::vector<float> zeros( jointListStop.getSize() );
          m_proxy->callVoid("changeAngles", jointListStop, zeros, 0.1f);

      }
    } // End Romeo
    else if (m_robotType == Pepper)
    {
      std::vector<float> vel(jointNames.size());
      jointVel.ToFloatArray(vel);
      m_pepper_control.async<void >("setDesJointVelocity", jointNames, vel);
    }

}

/*!
  Stop the velocity applied to the joints.
  \param names :  Names the joints, chains, "Body", "JointActuators",
  "Joints" or "Actuators" to stop.
 */
void vpNaoqiRobot::stop(const AL::ALValue &names) const
{
    std::vector<std::string> jointNames;
    if (names.isString()) // Suppose to be a chain
        jointNames = m_motionProxy->getBodyNames(names);
    else if (names.isArray()) // Supposed to be a vector of joints
        jointNames = names; // it a vector of joints
    else
        throw vpRobotException (vpRobotException::readingParametersError,
                                "Unable to decode the joint chain.");
    if (m_robotType == Romeo || m_robotType == Nao)
    {
      for (unsigned i = 0 ; i < jointNames.size() ; ++i)
          m_motionProxy->changeAngles(jointNames[i], 0.0f, 0.1f);
    }
    else if (m_robotType == Pepper)
    {
      m_pepper_control.call<void >("stopJoint");
    }

}

/*!
  Stop the velocity of the base.
 */
void vpNaoqiRobot::stopBase() const
{
  m_motionProxy->move(0.0, 0.0, 0.0);
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
 if (jointVel.size() == 3)
  m_motionProxy->move(jointVel[0],jointVel[1],jointVel[2]);
 else
   std::cerr << "ERROR: Cannot apply velocity to the base. Check the size of the vector, it has to be of size 3." << std::endl
             << "Current size: " << jointVel.size() << std::endl;
}


/*!
  Apply a velocity Vx, Vy, Wz to Pepper.
  \param vel : Joint velocity vector with values expressed in rad/s.
 */

void vpNaoqiRobot::setBaseVelocity(const float &vx,const float &vy,const float &wz) const
{
  m_motionProxy->move(vx,vy,wz);
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
  Get min joint values for a joint chain.

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
  Get min joint values for vector of joints.

  \return A vector that contains the minimal joint values
  of the chain. All the values are expressed in radians.

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
        AL::ALValue limits = m_motionProxy->getLimits(names[i]);
        //std::cout << limits << std::endl;
        min[i] = limits[0][0];
        max[i] = limits[0][1];
    }
    return;
}



/*!
  Get max joint values for a joint chain.

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


/*!
Gets the angles of the joints

 \return Joint angles in radians (VpColVector).

 \param names : Names the joints, chains, “Body”, “JointActuators”, “Joints” or “Actuators”.
        useSensors – If true, sensor angles will be returned

*/

vpColVector vpNaoqiRobot::getPosition(const AL::ALValue& names, const bool& useSensors) const
{
    std::vector<float> sensorAngles = m_motionProxy->getAngles(names, useSensors);
    vpColVector q(sensorAngles.size());
    for(unsigned int i=0; i<sensorAngles.size(); i++)
        q[i] = sensorAngles[i];
    return q;
}



/*!
Gets the angles of the joints

 \param names : Names the joints, chains, “Body”, “JointActuators”, “Joints” or “Actuators”.
        useSensors – If true, sensor angles will be returned
        q: Joint angles in radians (std::vector)

*/

void vpNaoqiRobot::getPosition(const AL::ALValue& names, std::vector<float>& q, const bool& useSensors) const
{
    q = m_motionProxy->getAngles(names, useSensors);
    return;
}



/*!
Set the position of the joints

 \param names:  The name or names of joints, chains, “Body”, “JointActuators”, “Joints” or “Actuators”.

 \param angles: One or more angles in radians

 \param fractionMaxSpeed – The fraction of maximum speed to use

*/

void vpNaoqiRobot::setPosition(const AL::ALValue& names, const AL::ALValue& angles, const float& fractionMaxSpeed)
{ 
    m_motionProxy->setAngles(names, angles, fractionMaxSpeed);
}

/*!
Set the position of the joints using vpColVector of Visp

 \param names:  The name or names of joints, chains, “Body”, “JointActuators”, “Joints” or “Actuators”.

 \param jointPosition: One or more angles in radians (vpColVector)

 \param fractionMaxSpeed – The fraction of maximum speed to use

*/
void vpNaoqiRobot::setPosition(const AL::ALValue& names, const vpColVector &jointPosition, const float& fractionMaxSpeed)
{
    std::vector<float> angles(jointPosition.size());
    for (unsigned int i=0; i<angles.size(); i++)
        angles[i] = jointPosition[i];

    m_motionProxy->setAngles(names, angles, fractionMaxSpeed);
}

/*!
Set the position of the joints using vpColVector of Visp

 \param names:  The name or names of joints, chains, “Body”, “JointActuators”, “Joints” or “Actuators”.

 \param jointPosition: One or more angles in radians (vpColVector)

 \param fractionMaxSpeed – The fraction of maximum speed to use

*/
void vpNaoqiRobot::setPosition(const AL::ALValue& names, const std::vector<float> &jointPosition, const float& fractionMaxSpeed)
{
    m_motionProxy->setAngles(names, (AL::ALValue)(jointPosition), fractionMaxSpeed);
}


vpMatrix vpNaoqiRobot::get_eJe(const std::string &chainName) const
{
    vpMatrix tJe;
    return get_eJe(chainName,tJe);
}



/*!
  Get Jacobian eJe for a joint chain.

  \return Jacobian eJe starting from the torso

  \param chainName : Name of the ChainName, can be "Head", "LArm" or "RArm".

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
        std::vector<std::string> jointNames = m_motionProxy->getBodyNames(chainName);

        // Get the angles of the joints in the chain we want to control
        std::vector<float> qmp = m_motionProxy->getAngles("Head",true);

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
        std::vector<std::string> jointNames = m_motionProxy->getBodyNames(chainName);
        jointNames.pop_back(); // Delete last joints LHand, that we don't consider in the servo

        // Get the angles of the joints in the chain we want to control
        std::vector<float> qmp = m_motionProxy->getAngles(chainName,true);
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
        std::vector<std::string> jointNamesLArm = m_motionProxy->getBodyNames("LArm");
        jointNamesLArm.pop_back(); // Delete last joints LHand, that we don't consider in the servo

        jointNames.insert(jointNames.end(),jointNamesLArm.begin(),jointNamesLArm.end());

        // Get the angles of the joints in the chain we want to control
        std::vector<float> qmp = m_motionProxy->getAngles(jointNames,true);

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
        std::vector<std::string> jointNames = m_motionProxy->getBodyNames(chainName);
        jointNames.pop_back(); // Delete last joints LHand, that we don't consider in the servo

        // Get the angles of the joints in the chain we want to control
        std::vector<float> qmp = m_motionProxy->getAngles(chainName,true);
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
        vpHomogeneousMatrix torsoMRWristP(m_motionProxy->getTransform(jointNames[nJoints-1], 0, true));

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
        std::vector<std::string> jointNames = m_motionProxy->getBodyNames("Head");
        std::vector<std::string> jointNamesEye = m_motionProxy->getBodyNames(chainName);

        jointNames.insert(jointNames.end(), jointNamesEye.begin(), jointNamesEye.end());


        // Get the angles of the joints in the chain we want to control
        std::vector<float> qmp = m_motionProxy->getAngles(jointNames,true);

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
        vpHomogeneousMatrix torsoMLEye(m_motionProxy->getTransform(jointNames[nJoints-1], 0, true));// get transformation  matrix between torso and LEye
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
        std::vector<std::string> jointNamesHead = m_motionProxy->getBodyNames("Head");
        std::vector<std::string> jointNamesEye = m_motionProxy->getBodyNames("LEye");

        //std::cout << "NAmes joint: qmp_trunk " << std::endl << jointNamesTrunk <<std::endl ;
        //std::cout << "NAmes joint: qmp_head " << std::endl << jointNamesHead <<std::endl ;
        //std::cout << "NAmes joint: qmp_leye " << std::endl << jointNamesEye <<std::endl ;

        // Get the angles of the joints in the chain we want to control

        std::vector<float> qmp_trunk = m_motionProxy->getAngles("TrunkYaw",true);
        qmp_trunk[0] = -qmp_trunk[0];
        std::vector<float> qmp_head = m_motionProxy->getAngles(jointNamesHead,true);
        std::vector<float> qmp_leye = m_motionProxy->getAngles(jointNamesEye,true);

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
        vpHomogeneousMatrix tMREye(m_motionProxy->getTransform(jointNamesEye[jointNamesEye.size()-1],2, true));// get transformation  matrix base torso and LEye
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
        std::vector<std::string> jointNamesHead = m_motionProxy->getBodyNames("Head");
        std::vector<std::string> jointNamesEye = m_motionProxy->getBodyNames(chainName);

        //jointNames.insert(jointNames.end(), jointNamesEye.begin(), jointNamesEye.end());


        // Get the angles of the joints in the chain we want to control
        std::vector<float> qmp_head = m_motionProxy->getAngles(jointNamesHead,true);
        std::vector<float> qmp_leye = m_motionProxy->getAngles(jointNamesEye,true);

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


        // Now we want to transform tJe to eJe
        vpHomogeneousMatrix torsoMREye(m_motionProxy->getTransform(jointNamesEye[jointNamesEye.size()-1], 0, true));// get transformation  matrix between torso and LEye
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
        std::vector<std::string> jointNames = m_motionProxy->getBodyNames(chainName);


        // Get the angles of the joints in the chain we want to control
        std::vector<float> qmp = m_motionProxy->getAngles("Head",true);

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

        vpHomogeneousMatrix torsoMHead(m_motionProxy->getTransform(jointNames[nJoints-1], 0, true));// get transformation  matrix between torso and Head
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
        std::vector<std::string> jointNames = m_motionProxy->getBodyNames(chainName);
        jointNames.pop_back(); // Delete last joints LHand, that we don't consider in the servo

        std::cout << "jointNames: " <<std::endl << jointNames;

        // Get the angles of the joints in the chain we want to control
        std::vector<float> qmp = m_motionProxy->getAngles(chainName,true);
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
        vpHomogeneousMatrix torsoMRWristP(m_motionProxy->getTransform(jointNames[nJoints-1], 0, true));

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


std::vector <vpMatrix> vpNaoqiRobot::get_d_eJe(const std::string &chainName) const
{
    std::vector <vpMatrix> dtJe;

    if (chainName == "LArm" && m_robotType == Romeo)
    {


        // Get the names of the joints in the chain we want to control
        std::vector<std::string> jointNames = m_motionProxy->getBodyNames(chainName);
        jointNames.pop_back(); // Delete last joints LHand, that we don't consider in the servo

        // Derivatives of the Jacobians matrix w.r.t the torso
        dtJe.resize(jointNames.size());

#ifdef VISP_NAOQI_HAVE_MATAPOD

        //confVector q for Romeo has size 24 (6 + 18dof of the robot, we don't consider the Legs and the fingers)
        RomeoModel::confVector q;
        RomeoModel::confVector dq;

        // Get the angles of the joints in the chain we want to control
        std::vector<float> qmp = m_motionProxy->getAngles(chainName,true);
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
  Get Homogeneous matrix cMe: from the camera to the last joint of the chain (HeadRoll).

  \return Homogeneous matrix cMe: from the camera to the last joint of the chain (HeadRoll).

  \param endEffectorName : Name of the camera.

*/
vpHomogeneousMatrix vpNaoqiRobot::get_cMe(const std::string &endEffectorName)
{
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



vpColVector vpNaoqiRobot::getJointVelocity(const std::vector <std::string> &names) const
{

    AL::ALValue list;

    for (unsigned int i = 0; i < names.size(); i++)
    {
        std::string key;
        if (m_robotType == Pepper )
            key = "Motion/Velocity/Sensor/" + names[i];
        else
            key = "Device/SubDeviceList/" + names[i] + "/Speed/Actuator/Value";

        list.arrayPush(key);
    }

    AL::ALValue vel_a = m_memProxy->getListData(list);

    vpColVector vel(names.size());
    for(unsigned int i=0; i<names.size(); i++)
        vel[i] = vel_a[i];

    return vel;
}


void vpNaoqiRobot::getJointVelocity(const std::vector <std::string> &names, std::vector <float> &jointVel) const
{

    AL::ALValue list;

    for (unsigned int i = 0; i < names.size(); i++)
    {
        std::string key;
        if (m_robotType == Pepper )
            key = "Motion/Velocity/Sensor/" + names[i];
        else
            key = "Device/SubDeviceList/" + names[i] + "/Speed/Actuator/Value";

        list.arrayPush(key);
    }
    try
    {
        jointVel = m_memProxy->getListData(list);
    }
    catch (const AL::ALError &e)
    {
        std::cerr << "Caught exception: " << e.what() << std::endl;
    }

    return;

}



