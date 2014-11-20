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

#include <visp/vpMath.h>
#include <visp/vpTime.h>
#include <visp/vpColVector.h>
#include <visp/vpVelocityTwistMatrix.h>

#include <visp_naoqi/vpNaoqiRobot.h>



int main(int argc, char* argv[])
{
  try
  {
    vpNaoqiRobot robot;


    robot.open();


    // Test Jacobian RArm
    {
      // Velocity end effector
      vpColVector v_o(6);
      v_o = 0.0;
      v_o[2] = -0.01; // vpMath::rad(5);

      const std::string chainName = "RArm";

      std::vector<std::string> jointNames = robot.getBodyNames(chainName);
      jointNames.pop_back(); // Delete last joints LHand, that we don't consider in the servo


      std::cout << "Test to apply a cartesian velocity to the object: " << v_o.t() << std::endl;
      vpColVector q_dot;


      double t_initial = vpTime::measureTimeSecond();
      while (vpTime::measureTimeSecond() < t_initial+3)
      {
        //** Set task eJe matrix
        // Get the actual Jacobian of the Larm
        vpMatrix eJe_RArm = robot.get_eJe("RArm");

        std::cout << "Jacobian of the RArm: "<< std::endl << eJe_RArm << std::endl;

        q_dot = eJe_RArm.pseudoInverse() * v_o;

        std::cout << "q_dot: " << q_dot.t() << std::endl;

        robot.setVelocity(jointNames, q_dot);
      }

      robot.stop(chainName);
      return 0;
    }


    // Test Jacobian Head
    {
      // Velocity end effector
      vpColVector v_o(6);
      v_o = 0.0;
      v_o[4] =0.1; // vpMath::rad(5);

      const std::string chainName = "Head";

      std::vector<std::string> jointNames = robot.getBodyNames(chainName);

      std::cout << "Test to apply a cartesian velocity to the object: " << v_o.t() << std::endl;
      vpColVector q_dot;


      double t_initial = vpTime::measureTimeSecond();
      while (vpTime::measureTimeSecond() < t_initial+3)
      {
        //** Set task eJe matrix
        // Get the actual Jacobian of the Larm
        vpMatrix eJe_Head = robot.get_eJe("Head");

        std::cout << "Jacobian of the Head: "<< std::endl << eJe_Head << std::endl;



        q_dot = eJe_Head.pseudoInverse() * v_o;

        std::cout << "q_dot: " << q_dot.t() << std::endl;

        robot.setVelocity(jointNames, q_dot);
      }

      robot.stop(chainName);
      return 0;
    }




    {
      // Get the actual Jacobian of the Head
      vpMatrix eJe_Head = robot.get_eJe("Head");

      std::cout << "Jacobian of the Head: "<< std::endl << eJe_Head << std::endl;


      return 0;

    }




    // Test Jacobian LArm
    {
      // Velocity end effector
      vpColVector v_o(6);
      v_o = 0.0;
      v_o[4] =-0.1; // vpMath::rad(5);

      const std::string chainName = "LArm";

      std::vector<std::string> jointNames = robot.getBodyNames(chainName);
      jointNames.pop_back(); // Delete last joints LHand, that we don't consider in the servo


      std::cout << "Test to apply a cartesian velocity to the object: " << v_o.t() << std::endl;
      vpColVector q_dot;

      // Constant transformation Target Frame to LArm end-effector (LWristPitch)
      vpHomogeneousMatrix oMe_LArm;
      oMe_LArm[0][3] = -0.05;
      oMe_LArm[1][3] = 0.026;
      oMe_LArm[2][3] = 0.0;
      vpVelocityTwistMatrix oVe_LArm(oMe_LArm);
      vpMatrix oJo; // Jacobian in the target (=object) frame

      double t_initial = vpTime::measureTimeSecond();
      while (vpTime::measureTimeSecond() < t_initial+4)
      {
        //** Set task eJe matrix
        // Get the actual Jacobian of the Larm
        vpMatrix eJe_LArm = robot.get_eJe("LArm");

        std::cout << "Jacobian of the LArm: "<< std::endl << eJe_LArm << std::endl;

        oJo = oVe_LArm * eJe_LArm;
        std::cout << "Jacobian of the object: "<< std::endl << eJe_LArm << std::endl;

        q_dot = oJo.pseudoInverse() * v_o;

        std::cout << "q_dot: " << q_dot.t() << std::endl;

        robot.setVelocity(jointNames, q_dot);
      }

      robot.stop(chainName);
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

