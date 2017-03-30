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

/*! \example motion_romeo.cpp */
#include <iostream>
#include <string>

#include <visp_naoqi/vpNaoqiRobot.h>

#include <visp/vpMath.h>
#include <visp/vpTime.h>
#include <visp/vpColVector.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>
#include <visp/vpPlot.h>
#include <visp/vpDisplayX.h>

#include <qi/session.hpp>
#include <qi/applicationsession.hpp>
#include <qi/anymodule.hpp>

#include <iterator>

/*!

   Connect to Pepper robot, and apply some motion.
   By default, this example connect to a robot with ip address: 198.18.0.1.
   If you want to connect on an other robot, run:

   ./motion -ip <robot ip address>

   Example:

   ./motion -ip 131.254.10.126
 */
int main(int argc, char** argv)
{
  std::string opt_ip = "192.168.0.24";
  unsigned int joint_type = 0;

  try
  {
    for (unsigned int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--ip")
        opt_ip = argv[i+1];
      if (std::string(argv[i]) == "--joint")
        joint_type = atoi(argv[i+1]);
      else if (std::string(argv[i]) == "--help") {
        std::cout << "Usage: " << argv[0] << "[--ip <robot address>] " << std::endl;
        return 0;
      }
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


  std::cout << "Using IP: " << opt_ip << std::endl;
  std::cout << "Controlling joint num: " << joint_type << std::endl;



  /* 0 : HeadYaw, HeadPitch
   * 1: ShoulderPitch

  */

  vpNaoqiRobot robot;
  robot.setRobotIp(opt_ip);
  robot.open();

  vpImage<unsigned char> I(320, 320);
  vpDisplayX d(I);
  vpDisplay::setTitle(I, "ViSP viewer");


  if (robot.getRobotType() != vpNaoqiRobot::Pepper)
  {
    std::cout << "ERROR: You are not connected to Pepper, but to a different Robot. Check the IP. " << std::endl;
    return 0;
  }

  std::vector<std::string> jointNames;


  switch (joint_type) {
  case 0:
    jointNames = robot.getBodyNames("Head");
    break;

  case 1:
    jointNames.push_back("LShoulderPitch");
    jointNames.push_back("RShoulderPitch");
    break;

  case 2:
    jointNames.push_back("LShoulderRoll");
    jointNames.push_back("RShoulderRoll");
    break;

  case 3:
    jointNames.push_back("LElbowRoll");
    jointNames.push_back("RElbowRoll");
    break;

  case 4:
    jointNames.push_back("LElbowYaw");
    jointNames.push_back("RElbowYaw");
    break;

  case 5:
    jointNames.push_back("LWristYaw");
    jointNames.push_back("RWristYaw");
    break;

  case 6:
    jointNames.push_back("HipRoll");
    jointNames.push_back("HeadYaw");
    break;

  case 7:
    jointNames.push_back("HipPitch");
    jointNames.push_back("HeadYaw");


    break;

  default:
    jointNames = robot.getBodyNames("Head");
    break;

  }

  // Plotting

  vpPlot plotter_diff_vel (2);
  plotter_diff_vel.initGraph(0, 2);
  plotter_diff_vel.initGraph(1, 2);
  plotter_diff_vel.setTitle(0,  jointNames[0].c_str());
  plotter_diff_vel.setTitle(1,  jointNames[1].c_str());

  vpPlot plotter_error (2);
  plotter_error.initGraph(0, 1);
  plotter_error.initGraph(1, 1);

  plotter_error.setTitle(0,  jointNames[0].c_str());
  plotter_error.setTitle(1,  jointNames[1].c_str());

  std::vector<float> jointVel( 2 );
  //    for (unsigned int i=0; i < jointVel.size(); i++)
  //      jointVel[i] = vpMath::rad(5);

  //    jointVel[0] = 20.25;// vpMath::rad(1000);

  unsigned long loop_iter = 0;
  vpColVector vel_head (jointNames.size());

  std::vector<float> q(jointNames.size());
  std::vector<float> q_new(jointNames.size());

  double t_prev = vpTime::measureTimeSecond();
  double start = t_prev;

  robot.setStiffness(jointNames, 1.0);

  qi::SessionPtr session = qi::makeSession();
  session->connect("tcp://"+ opt_ip +":9559");
  qi::AnyObject proxy = session->service("pepper_control");

  proxy.call<void >("start");

  while(1)
  {
    vpMouseButton::vpMouseButtonType button;
    bool ret = vpDisplay::getClick(I, button, false);

    robot.getPosition(jointNames,q,true);

    // robot.setVelocity(jointNames, jointVel,true);
    double now_t = vpTime::measureTimeSecond();


    switch (joint_type) {
    case 0:
      jointVel[0] = 0.3* (sin(2*(now_t-start)));
      jointVel[1] = 0.3* (sin(2*(now_t-start)));
      break;

    case 1:
      jointVel[0] = 0.3* (sin(2*(now_t-start)));
      jointVel[1] = 0.3* (sin(2*(now_t-start)));
      break;
    case 2:
      jointVel[0] = 0.3* (sin(2*(now_t-start)));
      jointVel[1] = 0.3* (sin(2*(now_t-start)));
      break;
    case 3:
      jointVel[0] = 0.3* (sin(1.5*(now_t-start)));
      jointVel[1] = 0.3* (sin(1.5*(now_t-start)));
      break;
    case 4:
      jointVel[0] = 0.3* (sin(1.5*(now_t-start)));
      jointVel[1] = 0.3* (sin(1.5*(now_t-start)));
      break;
    case 5:
      jointVel[0] = 0.5* (sin(1.0*(now_t-start)));
      jointVel[1] = 0.5* (sin(1.0*(now_t-start)));
      break;
    case 6:
      jointVel[0] = 0.3* (sin(2.0*(now_t-start)));
      jointVel[1] = 0.0;
      break;
    case 7:
      jointVel[0] = 0.2* (sin(2.0*(now_t-start)));
      jointVel[1] = 0.0;
      break;
    default:
      break;

    }

    proxy.async<void >("setDesJointVelocity", jointNames, jointVel );

    vel_head = robot.getJointVelocity(jointNames);

    for (unsigned int i=0 ; i < jointNames.size() ; i++) {
      plotter_diff_vel.plot(i,1,loop_iter,jointVel[i] );//vpMath::rad(jointVel[i]));
      plotter_diff_vel.plot(i,0,loop_iter,vel_head[i]);

      plotter_error.plot(i,0,loop_iter,q[i]);
      loop_iter ++;
    }

    vpTime::sleepMs(50);

    if (ret && button == vpMouseButton::button3)
      break;
    if (ret && button == vpMouseButton::button1)
    {
      jointNames[0] = "LShoulderPitch";
      jointNames[1] = "RShoulderPitch";
      joint_type = 1;
    }

  }

  proxy.call<void >("stopJoint");

  proxy.call<void >("stop");


  robot.stop(jointNames);

  vpDisplay::getClick(I, true);

return 0;



}
