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
#include <visp_naoqi/vpNaoqiGrabber.h>


/*!

   Connect to Nao or Romeo robot, and apply some motion.
   By default, this example connect to a robot with ip address: 198.18.0.1.
   If you want to connect on an other robot, run:

   ./motion -ip <robot ip address>

   Example:

   ./motion -ip 169.254.168.230
 */

int main(int argc, char* argv[])
{
  try
  {
    std::string opt_ip = "198.18.0.1";

    if (argc == 3) {
      if (std::string(argv[1]) == "-ip")
        opt_ip = argv[2];
    }

    vpNaoqiRobot robot;
    if (! opt_ip.empty()) {
      std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
      robot.setRobotIp(opt_ip);
    }

    robot.open();


    vpNaoqiGrabber g;
    g.setRobotIp(opt_ip);
    g.setFramerate(15);
    g.setCamera(3);
    g.open();


    //    {

    //      // Test Velocity controller LEye
    //      std::string chain = "LEye";

    //      std::cout << "Test " << chain << " velocity control" << std::endl;

    //      std::vector<std::string> jointNames = robot.getBodyNames(chain);

    //      std::cout << "___________________ " << jointNames << "___________________" << std::endl;


    //      robot.setStiffness(chain, 1.f);

    //      std::vector<float> vel;
    //      vel.resize(jointNames.size());


    //      // MOVE
    //      std::cout << "MoveYaw + " << std::endl;

    //      vel[0] = 0.4f;
    //      vel[1] = 0.0f;

    //      double t_initial = vpTime::measureTimeSecond();
    //      while (vpTime::measureTimeSecond() < t_initial+3)
    //      {
    //        robot.setVelocity (jointNames, vel);
    //      }

    //      robot.stop(chain);

    //      // MOVE
    //      std::cout << "MoveYaw - " <<  std::endl;

    //      vel[0] = -0.2f;
    //      vel[1] = 0.0f;

    //      t_initial = vpTime::measureTimeSecond();
    //      while (vpTime::measureTimeSecond() < t_initial+3)
    //      {
    //        robot.setVelocity (jointNames, vel);
    //      }

    //      robot.stop(chain);

    //      // MOVE
    //      std::cout << "MovePitch + " << std::endl;

    //      vel[0] = 0.0f;
    //      vel[1] = 0.1f;

    //      t_initial = vpTime::measureTimeSecond();
    //      while (vpTime::measureTimeSecond() < t_initial+3)
    //      {
    //        robot.setVelocity (jointNames, vel);
    //      }

    //      robot.stop(chain);

    //      // MOVE
    //      std::cout << "MovePitch - " << std::endl;

    //      vel[0] = 0.0f;
    //      vel[1] = -0.1f;

    //      t_initial = vpTime::measureTimeSecond();
    //      while (vpTime::measureTimeSecond() < t_initial+3)
    //      {
    //        robot.setVelocity (jointNames, vel);
    //      }

    //      robot.stop(chain);



    //      return 0;
    //    }



    // Test Jacobian Camera
//    {
//      // Velocity end effector
//      vpColVector q(2);
//      q[0]=0.0;
//      q[1]=-0.1;
//     std::vector<std::string> jointNamesEye = robot.getBodyNames("REye");


//      double t_initial = vpTime::measureTimeSecond();
//      while (vpTime::measureTimeSecond() < t_initial+3)
//      {

//        std::cout << "q_dot: " << q.t() << std::endl;

//        robot.setVelocity(jointNamesEye, q);
//      }

//      robot.stop(jointNamesEye);
//      return 0;
//    }











    // Test Jacobian Camera
    {
      // Velocity end effector
      vpColVector v_o(6);
      v_o = 0.0;
      v_o[3] =-0.1; // vpMath::rad(5);

      // Get the names of the joints in the chain we want to control (Head + Eye)
      std::vector<std::string> jointNames = robot.getBodyNames("Head");
      std::vector<std::string> jointNamesEye = robot.getBodyNames("REye");

      jointNames.insert(jointNames.end(), jointNamesEye.begin(), jointNamesEye.end());

      std::cout << "Controlling joint: " << jointNames << std::endl;

      vpMatrix eJe_LEye;
      vpMatrix cJc_LEye;
      std::cout << "Test to apply a cartesian velocity to the object: " << v_o.t() << std::endl;
      vpColVector q_dot;
      vpVelocityTwistMatrix cVe_LArm(g.get_eMc(vpCameraParameters::perspectiveProjWithDistortion,"CameraRightEye").inverse());


      double t_initial = vpTime::measureTimeSecond();
      while (vpTime::measureTimeSecond() < t_initial+2)
      {
        //** Set task eJe matrix
        // Get the actual Jacobian of the Larm
        eJe_LEye = robot.get_eJe("REye");

        // std::cout << "Jacobian of the LEye: "<< std::endl << eJe_LEye << std::endl;

        cJc_LEye = cVe_LArm * eJe_LEye;
        // std::cout << "Jacobian of the camera: "<< std::endl << cJc_LEye << std::endl;


        q_dot = cJc_LEye.inverseByLU() * v_o;

        std::cout << "q_dot: " << q_dot.t() << std::endl;

        robot.setVelocity(jointNames, q_dot);
      }

      robot.stop(jointNames);
      return 0;
    }





    // Test Jacobian Camera
    {
      // Velocity end effector
      vpColVector v_o(6);
      v_o = 0.0;
      v_o[2] =-0.01; // vpMath::rad(5);

      // Get the names of the joints in the chain we want to control (Head + Eye)
      std::vector<std::string> jointNames = robot.getBodyNames("Head");
      std::vector<std::string> jointNamesEye = robot.getBodyNames("LEye");

      jointNames.insert(jointNames.end(), jointNamesEye.begin(), jointNamesEye.end());


      vpMatrix eJe_LEye;
      vpMatrix cJc_LEye;
      std::cout << "Test to apply a cartesian velocity to the object: " << v_o.t() << std::endl;
      vpColVector q_dot;
      vpVelocityTwistMatrix cVe_LArm(g.get_eMc(vpCameraParameters::perspectiveProjWithDistortion,"CameraLeftEye").inverse());


      double t_initial = vpTime::measureTimeSecond();
      while (vpTime::measureTimeSecond() < t_initial+3)
      {
        //** Set task eJe matrix
        // Get the actual Jacobian of the Larm
        eJe_LEye = robot.get_eJe("LEye");

        std::cout << "Jacobian of the LEye: "<< std::endl << eJe_LEye << std::endl;

        cJc_LEye = cVe_LArm * eJe_LEye;
        std::cout << "Jacobian of the camera: "<< std::endl << cJc_LEye << std::endl;


        q_dot = cJc_LEye.inverseByLU() * v_o;

        std::cout << "q_dot: " << q_dot.t() << std::endl;

        robot.setVelocity(jointNames, q_dot);
      }

      robot.stop(jointNames);
      return 0;
    }














    // Test Jacobian Head
    {
      // Velocity end effector
      vpColVector v_o(6);
      v_o = 0.0;
      v_o[0] =0.05; // vpMath::rad(5);

      // Get the names of the joints in the chain we want to control (Head + Eye)
      std::vector<std::string> jointNames = robot.getBodyNames("Head");
      std::vector<std::string> jointNamesEye = robot.getBodyNames("LEye");

      jointNames.insert(jointNames.end(), jointNamesEye.begin(), jointNamesEye.end());


      std::cout << "Test to apply a cartesian velocity to the object: " << v_o.t() << std::endl;
      vpColVector q_dot;



      double t_initial = vpTime::measureTimeSecond();
      while (vpTime::measureTimeSecond() < t_initial+3)
      {
        //** Set task eJe matrix
        // Get the actual Jacobian of the Larm
        vpMatrix eJe_LEye = robot.get_eJe("LEye");

        std::cout << "Jacobian of the LEye: "<< std::endl << eJe_LEye << std::endl;



        q_dot = eJe_LEye.inverseByLU() * v_o;

        std::cout << "q_dot: " << q_dot.t() << std::endl;

        robot.setVelocity(jointNames, q_dot);
      }

      robot.stop(jointNames);
      return 0;
    }













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

