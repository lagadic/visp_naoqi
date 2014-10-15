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

#include <visp_naoqi/vpNaoqiRobot.h>


int main(int argc, char* argv[])
{
  try
  {
    vpNaoqiRobot robot;


    robot.open();

//    {
//      // Test with a vector of joints
//      std::vector<std::string> jointNames;
//      jointNames.push_back("NeckYaw");
//      jointNames.push_back("NeckPitch");

//      std::vector<float> jointVel( jointNames.size() );
//      for (unsigned int i=0; i < jointVel.size(); i++)
//        jointVel[i] = vpMath::rad(2);

//      robot.setStiffness(jointNames, 1.f);

//      double t_initial = vpTime::measureTimeSecond();
//      while (vpTime::measureTimeSecond() < t_initial+3)
//      {
//        robot.setVelocity(jointNames, jointVel);
//      }

//      robot.stop(jointNames);
//    }


//    {
//      // Test with a chain of joints
//      std::string chain = "Head";

//      robot.setStiffness(chain, 1.f);

//      std::vector<float> jointVel( robot.getJointNames( chain ).size() );
//      for (unsigned int i=0; i < jointVel.size(); i++)
//        jointVel[i] = vpMath::rad(-2);


//      double t_initial = vpTime::measureTimeSecond();
//      while (vpTime::measureTimeSecond() < t_initial+3)
//      {
//        robot.setVelocity(chain, jointVel);
//      }

//      robot.stop(chain);

//    }


//    {
//      // Get the position of the joints
//      std::string names = "Body";
//      bool useSensors   = true;
//      std::vector<float> commandAngles = robot.getAngles(names, useSensors);
//      std::cout << "Sensor angles: " << std::endl << commandAngles << std::endl;

//    }



//    {
//      // Example showing how to set angles, using a fraction of max speed
//       std::vector<std::string> jointNames;
//       jointNames.push_back("NeckYaw");
//       jointNames.push_back("NeckPitch");

//       std::vector<float> jointPos( jointNames.size() );
//       for (unsigned int i=0; i < jointPos.size(); i++)
//       jointPos[i] = vpMath::rad(0);

//       float fractionMaxSpeed  = 0.1f;
//       robot.setStiffness(jointNames, 1.f);
//       qi::os::sleep(1.0f);
//       robot.setAngles(jointNames, jointPos, fractionMaxSpeed);

//    }


   {
      //Get the actual Jacobian of the Head
      vpMatrix eJe = robot.getJacobian("Head");
      std::cout << "Jacobian of the Head: "<< std::endl << eJe << std::endl;

   }

   {
      //Get Transformation matrix between Frame HeadRoll and CameraLeft

       vpHomogeneousMatrix cMe = robot.getTransfEndEffector("CameraLeft");
       std::cout << "Transformation matrix between Frame HeadRoll and CameraLeft is : "<< cMe << std::endl;



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

