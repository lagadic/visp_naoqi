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

/*! \example motion_nao.cpp */
#include <iostream>
#include <string>

#include <visp/vpMath.h>
#include <visp/vpTime.h>
#include <visp/vpColVector.h>

#include <visp_naoqi/vpNaoqiRobot.h>


/*!

   Connect to Nao robot, and apply some motion.
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
    std::string opt_ip;

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

    if (robot.getRobotType() == vpNaoqiRobot::Nao) {

      {
        // Test with a vector of joints
        std::vector<std::string> jointNames;
        jointNames.push_back("HeadYaw");
        jointNames.push_back("HeadPitch");

        std::cout << "Test " << jointNames << " velocity control" << std::endl;

        vpColVector jointVel( jointNames.size() );
        for (unsigned int i=0; i < jointVel.size(); i++)
          jointVel[i] = vpMath::rad(10);

        robot.setStiffness(jointNames, 1.f);
        double t_initial = vpTime::measureTimeSecond();
        while (vpTime::measureTimeSecond() < t_initial+3)
        {
          robot.setVelocity(jointNames, jointVel);
        }

        robot.stop(jointNames);

        robot.setStiffness(jointNames, 0.f);
      }

      {
        // Test with a chain of joints
        std::string chain = "Head";

        std::cout << "Test " << chain << " velocity control" << std::endl;

        robot.setStiffness(chain, 1.f);

        std::cout << chain << " chain has the following joints: " << robot.getBodyNames( chain ) << std::endl;

        vpColVector jointVel( robot.getBodyNames( chain ).size() );
        for (unsigned int i=0; i < jointVel.size(); i++)
          jointVel[i] = vpMath::rad(-10);

        double t_initial = vpTime::measureTimeSecond();
        while (vpTime::measureTimeSecond() < t_initial+3)
        {
          robot.setVelocity(chain, jointVel);
        }

        robot.stop(chain);
        robot.setStiffness(chain, 0.f);
      }


      {
        // Get the position of the joints
        std::string names = "Body";
        std::cout << "Test " << names << " retrieving joint positions" << std::endl;

        vpColVector jointPositions = robot.getPosition(names);
        std::vector<std::string> jointNames = robot.getBodyNames(names);
        for (unsigned int i=0; i< jointPositions.size(); i++)
          std::cout << "Sensor " << jointNames[i] << " position: " << jointPositions[i] << std::endl;
      }

      {
        // Example showing how to set angles, using a fraction of max speed
        std::vector<std::string> jointNames;
        jointNames.push_back("HeadYaw");
        jointNames.push_back("HeadPitch");

        std::cout << "Test " << jointNames << " position control" << std::endl;

        std::vector<float> jointPos( jointNames.size() );
        for (unsigned int i=0; i < jointPos.size(); i++)
          jointPos[i] = vpMath::rad(0);

        float fractionMaxSpeed  = 0.1f;
        robot.setStiffness(jointNames, 1.f);
        qi::os::sleep(1.0f);
        robot.setPosition(jointNames, jointPos, fractionMaxSpeed);
        qi::os::sleep(2.0f); // Tempo to allow to reach the position
        robot.setStiffness(jointNames, 0.f);
      }
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

