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
 * This example demonstrates how to control the robot remotely in position and velocity.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*! \example motion.cpp */
#include <iostream>
#include <string>

#include <qi/session.hpp>

#include <visp/vpMath.h>
#include <visp/vpTime.h>
#include <visp/vpColVector.h>
#include <visp/vpVelocityTwistMatrix.h>

#include <visp_naoqi/vpNaoqiRobot.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>



int main(int argc, char* argv[])
{
  try
  {
    vpNaoqiRobot robot;
    robot.setRobotIp("192.168.0.24");
    robot.open();

    // Test Jacobian RArm
    {


      qi::SessionPtr session = qi::makeSession();
      session->connect("tcp://192.168.0.24:9559");
      qi::AnyObject proxy = session->service("pepper_control");

      proxy.call<void >("start");

      // Velocity end effector
      vpColVector v_o(6);
      v_o = 0.0;
      v_o[5] = 0.1; // vpMath::rad(5);

      const std::string chainName = "RArm";

      std::vector<std::string> jointNames = robot.getBodyNames(chainName);
      jointNames.pop_back(); // Delete last joints LHand, that we don't consider in the servo


      std::cout << "Test to apply a cartesian velocity to the object: " << v_o.t() << std::endl;
      vpColVector q_dot;

      //            // Constant transformation Target Frame to RArm end-effector (RWristPitch)
      //            vpHomogeneousMatrix oMe_RArm;

      //            std::string filename_transform = "transformation.xml";
      //            std::string name_transform = "qrcode_M_e_RArm";
      //            vpXmlParserHomogeneousMatrix pm; // Create a XML parser

      //            if (pm.parse(oMe_RArm, filename_transform, name_transform) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
      //                std::cout << "Cannot found the homogeneous matrix named " << name_transform << "." << std::endl;
      //                return 0;
      //            }
      //            else
      //                std::cout << "Homogeneous matrix " << name_transform <<": " << std::endl << oMe_RArm << std::endl;

      //            vpVelocityTwistMatrix oVe_RArm(oMe_RArm);
      //            vpMatrix oJo; // Jacobian in the target (=object) frame



      double t_initial = vpTime::measureTimeSecond();
      while (vpTime::measureTimeSecond() < t_initial+5)
      {
        //** Set task eJe matrix
        // Get the actual Jacobian of the Larm
        vpMatrix eJe_RArm = robot.get_eJe("RArm");

        std::cout << "Jacobian of the RArm: "<< std::endl << eJe_RArm << std::endl;

        //                oJo = oVe_RArm * eJe_RArm;
        //                q_dot = oJo.pseudoInverse() * v_o;
        q_dot = eJe_RArm.pseudoInverse() * v_o;

        std::vector<float> vel(q_dot.size());
        for (unsigned int i = 0; i<q_dot.size(); i++)
        {
          vel[i]=q_dot[i];
        }

        std::cout << "q_dot: " << q_dot.t() << std::endl;

        proxy.async<void >("setDesJointVelocity", jointNames, vel );
      }

      proxy.call<void >("stopJoint");
      proxy.call<void >("stop");

      return 0;
    }


    std::cout << "The end" << std::endl;
  }
  catch (const vpException &e)
  {
    std::cerr << "Caught exception: " << e.what() << std::endl;
  }
  catch (const AL::ALError &e)
  {
    std::cerr << "Caught exception: " << e.what() << std::endl;
  }

  return 0;
}

